/*
 * leg_pd_poc.c
 *
 * Safe first outer-PD bridge for Ben's 3-DOF leg:
 *
 *   PC --USART1--> H723 --FDCAN1--> three Mini ODrives
 *                     ^
 *                     | USART10 at 1 Mbps
 *                   G030 output encoders + foot switch
 *
 * This file deliberately owns no pin initialisation.  Configure CubeMX as
 * documented in README.md, add this file to Core/Src, and use the small
 * main.c integration shown in h723_main_changes.md.
 *
 * Safety properties of this first POC:
 *   - H723 never enables an axis at boot.
 *   - ARM is accepted only after fresh, CRC-valid G030 data and all three
 *     motor heartbeats are present.
 *   - Any encoder-node fault/staleness, motor error, or PC command timeout
 *     commands zero torque and requests ODrive IDLE.
 *   - Torque is hard-clamped to 0.200 N m at the motor (about 5 A for
 *     Kt=0.04).  This is the user's validated continuous bench-current limit.
 *   - Position targets are limited to +/-10 degrees from the captured pose.
 *
 * Do not increase these limits before the direction-pulse test in README.md.
 */

#include "leg_pd_poc.h"

#include "main.h"
#include "fdcan.h"
#include "usart.h"
#include "bsp_fdcan.h"
#include "BMI088driver.h"
#include "ws2812.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Raw CAN1 receive diagnostics are maintained by bsp_fdcan.c before any
 * protocol decoding.  They prove whether the FDCAN1 peripheral receives
 * electrical CAN traffic at all. */
extern volatile uint32_t can1_raw_rx_count;
extern volatile uint16_t can1_last_rx_id;

/* --------- User configuration: edit only after the direction test -------- */

#define LEG_JOINTS                         3U

/* These must match the three Mini ODrive CAN node IDs on CAN1. */
static const uint8_t k_node_id[LEG_JOINTS] = {1U, 2U, 3U};

/*
 * At the direction-pulse test, a +pulse must make the reported q increase.
 * Flip only the affected value to -1.0f if it makes q decrease.
 */
/* Calibrated by the single-joint pulse tests: motor-positive torque decreases
 * every corresponding external output angle, so all three joints invert it. */
static const float k_motor_torque_sign[LEG_JOINTS] = {-1.0f, -1.0f, -1.0f};

/* Initial conservative outer PD gains, in motor N m / output-rad. */
static float g_kp[LEG_JOINTS] = {0.020f, 0.020f, 0.020f};
static float g_kd[LEG_JOINTS] = {0.0008f, 0.0008f, 0.0008f};

/* Never make the host command able to exceed this build-time hard limit. */
/* Continuous bench-validation ceiling: 0.200 N m ~= 5 A at Kt=0.04 N m/A.
 * Keep the default command cap below this so torque never increases merely by
 * arming.  The separately requested 8 A / 3 s burst is intentionally not
 * available through the normal GAINS command. */
#define HARD_TORQUE_CAP_NM                 0.200f
#define USER_TORQUE_CAP_DEFAULT_NM         0.006f
#define USER_TORQUE_CAP_MAX_NM             0.200f

#define DEG_TO_RAD_F                       0.01745329251994329577f

/* Joint limits are relative to the pose captured by ZERO.  Soft limits clamp
 * PC SET targets; hard limits cut torque and disarm before a mechanical end
 * stop.  They include the margin agreed from the measured mechanical travel. */
static const float k_soft_min_rad[LEG_JOINTS] = {
    -60.0f * DEG_TO_RAD_F, /* hip   */
    -60.0f * DEG_TO_RAD_F, /* knee  */
    -60.0f * DEG_TO_RAD_F  /* ankle */
};
static const float k_soft_max_rad[LEG_JOINTS] = {
     60.0f * DEG_TO_RAD_F, /* hip   */
     60.0f * DEG_TO_RAD_F, /* knee  */
     60.0f * DEG_TO_RAD_F  /* ankle */
};

static const float k_hard_min_rad[LEG_JOINTS] = {
    -90.0f * DEG_TO_RAD_F, /* hip   */
    -90.0f * DEG_TO_RAD_F, /* knee  */
    -90.0f * DEG_TO_RAD_F  /* ankle */
};

static const float k_hard_max_rad[LEG_JOINTS] = {
     90.0f * DEG_TO_RAD_F, /* hip   */
     90.0f * DEG_TO_RAD_F, /* knee  */
     90.0f * DEG_TO_RAD_F  /* ankle */
};

#define TARGET_SLEW_RAD_S                  (540.0f * 0.01745329251994329577f)
#define VELOCITY_FILTER_ALPHA              0.20f

#define G030_FRAME_BYTES                   19UC
#define G030_SOF0                          0xA5U
#define G030_SOF1                          0x5AU
#define G030_VERSION                       3U
#define G030_FLAGS_ALL_ENCODERS_VALID      0x0EU
#define G030_FLAG_FOOT_PRESSED             0x01U
#define G030_FLAG_NODE_FAULT               0x10U
#define G030_FRAME_MAX_AGE_MS              10U

#define PC_COMMAND_TIMEOUT_MS              250U
#define CAN_HEARTBEAT_MAX_AGE_MS           250U
/* Direction/strength calibration only.  0.050 N m is about 1.25 A for the
 * assumed 0.04 N m/A motor Kt: below the configured 2 A bench limit. */
#define PULSE_DURATION_MS                  500U
#define PULSE_MAX_TORQUE_NM                0.0500f

/*
 * DM-MC-Board02 has two PMOS-controlled power outputs.  Your copied project
 * drives both with PC13/PC14.  Keep this enabled only if the motors' 24 V is
 * actually supplied through those controlled XT30 outputs.  POWER ON is
 * deliberately a separate PC command; the controller never energises them at
 * boot or as a side effect of ARM.
 */
#define POC_USE_BOARD_MOTOR_POWER           1U
#define BOARD_MOTOR_POWER_PORT              GPIOC
#define BOARD_MOTOR_POWER_PINS              (GPIO_PIN_13 | GPIO_PIN_14)

/* Board02 pinout: PC15 is the common controlled 5 V rail enable ("5V EN").
 * It feeds the 5 V UART/CAN/PWM accessory rail, not the two XT30 motor-power
 * outputs.  It stays on after POWER OFF so G030 telemetry remains alive. */
#define BOARD_PERIPHERAL_5V_PORT            GPIOC
#define BOARD_PERIPHERAL_5V_PIN             GPIO_PIN_15

#define AXIS_STATE_IDLE                    1U
#define AXIS_STATE_CLOSED_LOOP_CONTROL     8U
#define CONTROL_MODE_TORQUE                1U
#define INPUT_MODE_PASSTHROUGH             1U

#define CAN_CMD_HEARTBEAT                  0x01U
#define CAN_CMD_SET_AXIS_STATE             0x07U
#define CAN_CMD_SET_CONTROLLER_MODE        0x0BU
#define CAN_CMD_SET_INPUT_TORQUE           0x0EU
#define CAN_CMD_CLEAR_ERRORS               0x18U

#define TWO_PI_F                           6.28318530717958647692f
#define RAD_TO_DEG_F                       57.2957795130823208768f

/* -------------------------- Internal state ------------------------------- */

typedef struct {
    volatile uint32_t axis_error;
    volatile uint8_t axis_state;
    volatile uint32_t last_heartbeat_ms;
    volatile uint32_t heartbeat_count;
} MotorState;

typedef struct {
    uint16_t last_raw;
    float unwrapped_turns;
    float zero_turns;
    float q_rad;
    float dq_rad_s;
    uint32_t last_sensor_timestamp_ms;
    bool initialised;
} OutputEncoderState;

typedef enum {
    POC_SAFE = 0,
    POC_ARMED,
    POC_FAULT
} PocMode;

typedef enum {
    ARM_SEQUENCE_IDLE = 0,
    ARM_SEQUENCE_SEND_COMMANDS,
    ARM_SEQUENCE_WAIT_CLOSED_LOOP
} ArmSequenceState;

static MotorState g_motor[LEG_JOINTS];
static OutputEncoderState g_encoder[LEG_JOINTS];

static volatile uint8_t g_g030_flags = 0U;
static volatile uint32_t g_g030_timestamp_ms = 0U;
static volatile uint32_t g_g030_last_rx_ms = 0U;
static volatile uint32_t g_g030_good_frames = 0U;
static volatile uint32_t g_g030_bad_frames = 0U;
static volatile uint16_t g_g030_raw[LEG_JOINTS] = {0U, 0U, 0U};

static volatile uint32_t g_can_tx_drop = 0U;
static volatile uint32_t g_pc_last_command_ms = 0U;
static volatile bool g_pc_line_ready = false;
static char g_pc_line[96];
static uint8_t g_pc_line_length = 0U;

static PocMode g_mode = POC_SAFE;
static bool g_zero_captured = false;
static bool g_imu_ok = false;
static uint8_t g_imu_error = 0U;
static float g_imu_gyro[3];
static float g_imu_accel[3];
static float g_imu_temperature = 0.0f;

static float g_target_requested_rad[LEG_JOINTS];
static float g_target_applied_rad[LEG_JOINTS];
static float g_user_torque_cap_nm = USER_TORQUE_CAP_DEFAULT_NM;
static float g_last_tau_motor_nm[LEG_JOINTS];

static uint32_t g_pulse_until_ms = 0U;
static int8_t g_pulse_joint = -1;
static float g_pulse_joint_torque_nm = 0.0f;
static bool g_idle_sent = false;
static ArmSequenceState g_arm_sequence = ARM_SEQUENCE_IDLE;
static uint8_t g_arm_sequence_frame = 0U;
static uint32_t g_arm_sequence_deadline_ms = 0U;
static bool g_motor_power_on = false;
static bool g_power_off_pending = false;
static uint32_t g_power_off_at_ms = 0U;
static char g_fault_reason[32] = "none";

/* DMA buffers must remain alive for the whole application. */
static uint8_t g_uart1_rx_dma[96];
static uint8_t g_uart10_rx_dma[64];
static uint8_t g_g030_parser[G030_FRAME_BYTES];
static uint8_t g_g030_parser_length = 0U;
static uint8_t g_uart1_tx_dma[320];
static volatile bool g_uart1_tx_busy = false;
static volatile uint32_t g_uart1_tx_started_ms = 0U;
static volatile uint32_t g_uart1_tx_timeout_count = 0U;

/* --------------------------- Small utilities ----------------------------- */

static bool g030_is_healthy(uint32_t now_ms);
static bool can_is_healthy(uint32_t now_ms);

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint16_t get_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t get_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0]
         | ((uint32_t)data[1] << 8)
         | ((uint32_t)data[2] << 16)
         | ((uint32_t)data[3] << 24);
}

static void put_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value);
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static void put_float_le(uint8_t *data, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32_le(data, bits);
}

static uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    while (length-- != 0U) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static uint16_t odrive_id(uint8_t node_id, uint8_t cmd_id)
{
    return (uint16_t)(((uint16_t)node_id << 5) | (cmd_id & 0x1FU));
}

static void board_motor_power_set(bool on)
{
#if POC_USE_BOARD_MOTOR_POWER
    HAL_GPIO_WritePin(BOARD_MOTOR_POWER_PORT, BOARD_MOTOR_POWER_PINS,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)on;
#endif
    g_motor_power_on = on;
}

static void board_peripheral_5v_enable(void)
{
    HAL_GPIO_WritePin(BOARD_PERIPHERAL_5V_PORT, BOARD_PERIPHERAL_5V_PIN,
                      GPIO_PIN_SET);
}

/*
 * WS2812 is deliberately updated from LegPdPoc_Service() at 10 Hz, never from
 * the 1 kHz PD loop.  The existing Board02 project drives it through SPI6.
 */
static void update_status_led(uint32_t now_ms)
{
    static uint32_t last_update_ms = 0U;
    const bool blink_fast = ((now_ms / 125U) & 1U) == 0U;
    const bool blink_slow = ((now_ms / 400U) & 1U) == 0U;

    if ((now_ms - last_update_ms) < 100U) return;
    last_update_ms = now_ms;

    if (g_mode == POC_FAULT) {
        /* Red fast blink: latched G030/CAN/configuration fault. */
        WS2812_Ctrl(blink_fast ? 80U : 0U, 0U, 0U);
    } else if (strcmp(g_fault_reason, "pc_timeout") == 0) {
        /* Purple fast blink: PC command watchdog disarmed the leg. */
        WS2812_Ctrl(blink_fast ? 70U : 0U, 0U, blink_fast ? 70U : 0U);
    } else if (g_mode == POC_ARMED) {
        /* Green steady: torque outer-PD active. */
        WS2812_Ctrl(0U, 70U, 0U);
    } else if (!g030_is_healthy(now_ms)) {
        /* Blue slow blink: waiting for valid G030 encoder packet. */
        WS2812_Ctrl(0U, 0U, blink_slow ? 70U : 0U);
    } else if (g_motor_power_on && !can_is_healthy(now_ms)) {
        /* Orange slow blink: output power on, waiting for CAN heartbeats. */
        WS2812_Ctrl(blink_slow ? 70U : 0U, blink_slow ? 25U : 0U, 0U);
    } else if (g_zero_captured) {
        /* White steady: ZERO accepted, safe and ready for ARM. */
        WS2812_Ctrl(45U, 45U, 45U);
    } else if (g_motor_power_on) {
        /* Yellow steady: CAN healthy; move to upright pose and issue ZERO. */
        WS2812_Ctrl(65U, 50U, 0U);
    } else {
        /* Blue steady: G030 valid but controlled motor power is off. */
        WS2812_Ctrl(0U, 0U, 35U);
    }
}

/* Never block the 1 kHz foreground loop waiting for a CAN FIFO slot. */
static bool can1_send(uint8_t node_id, uint8_t cmd_id,
                      const uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};
    uint8_t payload[8] = {0};

    if (length > 8U || HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        ++g_can_tx_drop;
        return false;
    }

    if (data != NULL && length != 0U) {
        memcpy(payload, data, length);
    }

    header.Identifier = odrive_id(node_id, cmd_id);
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    switch (length) {
    case 0U: header.DataLength = FDCAN_DLC_BYTES_0; break;
    case 1U: header.DataLength = FDCAN_DLC_BYTES_1; break;
    case 2U: header.DataLength = FDCAN_DLC_BYTES_2; break;
    case 3U: header.DataLength = FDCAN_DLC_BYTES_3; break;
    case 4U: header.DataLength = FDCAN_DLC_BYTES_4; break;
    case 5U: header.DataLength = FDCAN_DLC_BYTES_5; break;
    case 6U: header.DataLength = FDCAN_DLC_BYTES_6; break;
    case 7U: header.DataLength = FDCAN_DLC_BYTES_7; break;
    default: header.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, payload) != HAL_OK) {
        ++g_can_tx_drop;
        return false;
    }

    return true;
}

static void motor_send_torque(uint8_t joint, float motor_torque_nm)
{
    uint8_t data[4];
    put_float_le(data, motor_torque_nm);
    (void)can1_send(k_node_id[joint], CAN_CMD_SET_INPUT_TORQUE, data, 4U);
}

static void motor_send_axis_state(uint8_t joint, uint32_t axis_state)
{
    uint8_t data[4];
    put_u32_le(data, axis_state);
    (void)can1_send(k_node_id[joint], CAN_CMD_SET_AXIS_STATE, data, 4U);
}

/*
 * UART/CAN receive interrupts may publish a timestamp immediately after a
 * foreground function snapshots HAL_GetTick().  In that harmless one-tick
 * ordering case, an unsigned subtraction would look like 0xFFFFFFFF ms.
 * Treat a timestamp up to the present instant as age zero instead.
 */
static uint32_t sample_age_ms(uint32_t now_ms, uint32_t sample_ms)
{
    const int32_t elapsed_ms = (int32_t)(now_ms - sample_ms);
    return (elapsed_ms > 0) ? (uint32_t)elapsed_ms : 0U;
}

static bool g030_is_healthy(uint32_t now_ms)
{
    const uint8_t flags = g_g030_flags;

    return (sample_age_ms(now_ms, g_g030_last_rx_ms) <= G030_FRAME_MAX_AGE_MS)
        && ((flags & G030_FLAGS_ALL_ENCODERS_VALID) == G030_FLAGS_ALL_ENCODERS_VALID)
        && ((flags & G030_FLAG_NODE_FAULT) == 0U)
        && g_encoder[0].initialised
        && g_encoder[1].initialised
        && g_encoder[2].initialised;
}

static bool can_is_healthy(uint32_t now_ms)
{
    for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
        if (g_motor[joint].heartbeat_count == 0U
            || sample_age_ms(now_ms, g_motor[joint].last_heartbeat_ms) > CAN_HEARTBEAT_MAX_AGE_MS
            || g_motor[joint].axis_error != 0U) {
            return false;
        }
    }
    return true;
}

static void set_fault(const char *reason)
{
    if (g_mode == POC_FAULT) return;

    g_arm_sequence = ARM_SEQUENCE_IDLE;
    strncpy(g_fault_reason, reason, sizeof(g_fault_reason) - 1U);
    g_fault_reason[sizeof(g_fault_reason) - 1U] = '\0';
    g_mode = POC_FAULT;
}

static void send_zero_and_idle_once(void)
{
    if (g_idle_sent) return;

    for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
        motor_send_torque(joint, 0.0f);
        motor_send_axis_state(joint, AXIS_STATE_IDLE);
        g_last_tau_motor_nm[joint] = 0.0f;
    }
    g_idle_sent = true;
}

static void disarm(const char *reason)
{
    if (reason != NULL) {
        strncpy(g_fault_reason, reason, sizeof(g_fault_reason) - 1U);
        g_fault_reason[sizeof(g_fault_reason) - 1U] = '\0';
    }

    g_mode = POC_SAFE;
    g_arm_sequence = ARM_SEQUENCE_IDLE;
    g_pulse_joint = -1;
    g_pulse_until_ms = 0U;
    send_zero_and_idle_once();
}

/* Give the zero-torque/IDLE CAN frames time to leave the FIFO before cutting
 * the optional PMOS output.  This is not a replacement for a physical e-stop.
 */
static void request_board_power_off(uint32_t now_ms)
{
    disarm("power_off");
    g_power_off_pending = true;
    g_power_off_at_ms = now_ms + 30U;
}

static void update_encoder_from_g030(uint8_t joint, uint16_t raw,
                                     uint32_t sensor_timestamp_ms)
{
    OutputEncoderState *encoder = &g_encoder[joint];

    if (!encoder->initialised) {
        encoder->last_raw = raw;
        encoder->unwrapped_turns = (float)raw / 16384.0f;
        encoder->zero_turns = encoder->unwrapped_turns;
        encoder->last_sensor_timestamp_ms = sensor_timestamp_ms;
        encoder->q_rad = 0.0f;
        encoder->dq_rad_s = 0.0f;
        encoder->initialised = true;
        return;
    }

    int32_t delta = (int32_t)raw - (int32_t)encoder->last_raw;
    if (delta > 8192) delta -= 16384;
    if (delta < -8192) delta += 16384;

    uint32_t dt_ms = sensor_timestamp_ms - encoder->last_sensor_timestamp_ms;
    if (dt_ms >= 1U && dt_ms <= 10U) {
        const float raw_velocity_rad_s =
            ((float)delta / 16384.0f) * TWO_PI_F * (1000.0f / (float)dt_ms);
        encoder->dq_rad_s += VELOCITY_FILTER_ALPHA
                           * (raw_velocity_rad_s - encoder->dq_rad_s);
    }

    encoder->unwrapped_turns += (float)delta / 16384.0f;
    encoder->last_raw = raw;
    encoder->last_sensor_timestamp_ms = sensor_timestamp_ms;
    encoder->q_rad = (encoder->unwrapped_turns - encoder->zero_turns) * TWO_PI_F;
}

static void process_g030_byte(uint8_t byte)
{
    if (g_g030_parser_length == 0U) {
        if (byte == G030_SOF0) {
            g_g030_parser[g_g030_parser_length++] = byte;
        }
        return;
    }

    if (g_g030_parser_length == 1U) {
        if (byte == G030_SOF1) {
            g_g030_parser[g_g030_parser_length++] = byte;
        } else {
            g_g030_parser_length = (byte == G030_SOF0) ? 1U : 0U;
            if (g_g030_parser_length != 0U) g_g030_parser[0] = byte;
        }
        return;
    }

    g_g030_parser[g_g030_parser_length++] = byte;
    if (g_g030_parser_length != G030_FRAME_BYTES) return;

    const uint16_t received_crc = get_u16_le(&g_g030_parser[17]);
    const uint16_t calculated_crc = crc16_ccitt_false(&g_g030_parser[2], 15U);

    if (g_g030_parser[2] == G030_VERSION && received_crc == calculated_crc) {
        const uint8_t flags = g_g030_parser[10];
        const uint32_t timestamp_ms = get_u32_le(&g_g030_parser[6]);

        g_g030_raw[0] = get_u16_le(&g_g030_parser[11]);
        g_g030_raw[1] = get_u16_le(&g_g030_parser[13]);
        g_g030_raw[2] = get_u16_le(&g_g030_parser[15]);
        g_g030_flags = flags;
        g_g030_timestamp_ms = timestamp_ms;

        if ((flags & G030_FLAGS_ALL_ENCODERS_VALID) == G030_FLAGS_ALL_ENCODERS_VALID) {
            update_encoder_from_g030(0U, g_g030_raw[0], timestamp_ms);
            update_encoder_from_g030(1U, g_g030_raw[1], timestamp_ms);
            update_encoder_from_g030(2U, g_g030_raw[2], timestamp_ms);
        }

        g_g030_last_rx_ms = HAL_GetTick();
        ++g_g030_good_frames;
    } else {
        ++g_g030_bad_frames;
    }

    g_g030_parser_length = 0U;
}

/* --------------------------- UART callbacks ------------------------------ */

static void pc_append_byte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (g_pc_line_length != 0U && !g_pc_line_ready) {
            g_pc_line[g_pc_line_length] = '\0';
            g_pc_line_ready = true;
        }
        g_pc_line_length = 0U;
        return;
    }

    if (g_pc_line_length < (sizeof(g_pc_line) - 1U) && !g_pc_line_ready) {
        g_pc_line[g_pc_line_length++] = (char)byte;
    } else if (!g_pc_line_ready) {
        g_pc_line_length = 0U; /* reject an overlength line */
    }
}

static void uart_start_receivers(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_uart1_rx_dma, sizeof(g_uart1_rx_dma));
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart10, g_uart10_rx_dma, sizeof(g_uart10_rx_dma));

    /* We use only IDLE and full-buffer events, never half-transfer events. */
    if (huart1.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
    if (huart10.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart == &huart10) {
        for (uint16_t index = 0U; index < size; ++index) {
            process_g030_byte(g_uart10_rx_dma[index]);
        }
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart10, g_uart10_rx_dma, sizeof(g_uart10_rx_dma));
        if (huart10.hdmarx != NULL) __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
    } else if (huart == &huart1) {
        for (uint16_t index = 0U; index < size; ++index) {
            pc_append_byte(g_uart1_rx_dma[index]);
        }
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_uart1_rx_dma, sizeof(g_uart1_rx_dma));
        if (huart1.hdmarx != NULL) __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart10) {
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart10, g_uart10_rx_dma, sizeof(g_uart10_rx_dma));
    } else if (huart == &huart1) {
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_uart1_rx_dma, sizeof(g_uart1_rx_dma));
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        g_uart1_tx_busy = false;
    }
}

/* ----------------------------- CAN receive -------------------------------- */

void can_parse_feedback(uint16_t rx_id, uint8_t *data, uint8_t len)
{
    const uint8_t node_id = (uint8_t)(rx_id >> 5);
    const uint8_t command = (uint8_t)(rx_id & 0x1FU);

    if (command != CAN_CMD_HEARTBEAT || len < 8U) return;

    for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
        if (node_id == k_node_id[joint]) {
            g_motor[joint].axis_error = get_u32_le(&data[0]);
            g_motor[joint].axis_state = data[4];
            g_motor[joint].last_heartbeat_ms = HAL_GetTick();
            ++g_motor[joint].heartbeat_count;
            return;
        }
    }
}

/* -------------------------- PC command parsing ---------------------------- */

static void capture_zero(void)
{
    for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
        g_encoder[joint].zero_turns = g_encoder[joint].unwrapped_turns;
        g_encoder[joint].q_rad = 0.0f;
        g_encoder[joint].dq_rad_s = 0.0f;
        g_target_requested_rad[joint] = 0.0f;
        g_target_applied_rad[joint] = 0.0f;
    }

    g_zero_captured = true;
    strncpy(g_fault_reason, "zero_captured", sizeof(g_fault_reason) - 1U);
    g_fault_reason[sizeof(g_fault_reason) - 1U] = '\0';
}

static bool arm_if_safe(uint32_t now_ms)
{
#if POC_USE_BOARD_MOTOR_POWER
    if (!g_motor_power_on) {
        set_fault("power_off");
        return false;
    }
#endif
    if (!g_zero_captured) {
        set_fault("zero_required");
        return false;
    }
    if (!g030_is_healthy(now_ms)) {
        set_fault("g030_invalid");
        return false;
    }
    if (!can_is_healthy(now_ms)) {
        set_fault("can_not_ready");
        return false;
    }

    /* One ARM used to enqueue all 12 frames (four for each node) at once.
     * FDCAN has only eight TX FIFO slots, so node 3 could lose every command.
     * The 1 kHz sequencer sends exactly one frame per tick instead. */
    g_idle_sent = false;
    g_pulse_joint = -1;
    g_arm_sequence_frame = 0U;
    g_arm_sequence_deadline_ms = now_ms + 600U;
    g_arm_sequence = ARM_SEQUENCE_SEND_COMMANDS;
    strncpy(g_fault_reason, "arming", sizeof(g_fault_reason) - 1U);
    g_fault_reason[sizeof(g_fault_reason) - 1U] = '\0';
    return true;
}

static bool all_axes_closed_loop(void)
{
    for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
        if (g_motor[joint].axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL
            || g_motor[joint].axis_error != 0U) {
            return false;
        }
    }
    return true;
}

static void service_arm_sequence(uint32_t now_ms)
{
    if (g_arm_sequence == ARM_SEQUENCE_SEND_COMMANDS) {
        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) return;

        const uint8_t joint = g_arm_sequence_frame / 4U;
        const uint8_t step = g_arm_sequence_frame % 4U;
        bool sent = false;

        if (step == 0U) {
            sent = can1_send(k_node_id[joint], CAN_CMD_CLEAR_ERRORS, NULL, 0U);
        } else if (step == 1U) {
            uint8_t data[8];
            put_u32_le(&data[0], CONTROL_MODE_TORQUE);
            put_u32_le(&data[4], INPUT_MODE_PASSTHROUGH);
            sent = can1_send(k_node_id[joint], CAN_CMD_SET_CONTROLLER_MODE, data, 8U);
        } else if (step == 2U) {
            /* IEEE-754 +0.0f is all-zero bytes on the STM32 target. */
            const uint8_t data[4] = {0U, 0U, 0U, 0U};
            sent = can1_send(k_node_id[joint], CAN_CMD_SET_INPUT_TORQUE, data, 4U);
        } else {
            uint8_t data[4];
            put_u32_le(data, AXIS_STATE_CLOSED_LOOP_CONTROL);
            sent = can1_send(k_node_id[joint], CAN_CMD_SET_AXIS_STATE, data, 4U);
        }

        if (!sent) return;
        ++g_arm_sequence_frame;
        if (g_arm_sequence_frame >= (LEG_JOINTS * 4U)) {
            g_arm_sequence = ARM_SEQUENCE_WAIT_CLOSED_LOOP;
        }
        return;
    }

    if (g_arm_sequence == ARM_SEQUENCE_WAIT_CLOSED_LOOP) {
        if (all_axes_closed_loop()) {
            for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
                /* No position step at arm: start at the current measured pose. */
                g_target_requested_rad[joint] = g_encoder[joint].q_rad;
                g_target_applied_rad[joint] = g_encoder[joint].q_rad;
            }
            g_arm_sequence = ARM_SEQUENCE_IDLE;
            g_mode = POC_ARMED;
            strncpy(g_fault_reason, "armed", sizeof(g_fault_reason) - 1U);
            g_fault_reason[sizeof(g_fault_reason) - 1U] = '\0';
        } else if ((int32_t)(now_ms - g_arm_sequence_deadline_ms) >= 0) {
            set_fault("axis_not_closed_loop");
            send_zero_and_idle_once();
        }
    }
}

static void handle_pc_line(uint32_t now_ms)
{
    float a, b, c;
    unsigned joint;

    g_pc_last_command_ms = now_ms;

    if (strcmp(g_pc_line, "PING") == 0 || strcmp(g_pc_line, "KEEP") == 0) {
        return;
    }

    if (strcmp(g_pc_line, "DISARM") == 0) {
        disarm("pc_disarm");
        return;
    }

    if (strcmp(g_pc_line, "CLEAR") == 0) {
        if (g_mode != POC_ARMED) {
            g_mode = POC_SAFE;
            strncpy(g_fault_reason, "cleared", sizeof(g_fault_reason) - 1U);
            g_fault_reason[sizeof(g_fault_reason) - 1U] = '\0';
        }
        return;
    }

    if (strcmp(g_pc_line, "ZERO") == 0) {
        if (g_mode == POC_SAFE && g030_is_healthy(now_ms)) {
            capture_zero();
        } else if (g_mode != POC_SAFE) {
            set_fault("zero_while_armed");
        } else {
            set_fault("zero_g030_bad");
        }
        return;
    }

    if (strcmp(g_pc_line, "ARM") == 0) {
        if (g_mode == POC_SAFE) (void)arm_if_safe(now_ms);
        return;
    }

    if (strcmp(g_pc_line, "POWER ON") == 0) {
        if (g_mode == POC_SAFE) {
            g_power_off_pending = false;
            board_motor_power_set(true);
            strncpy(g_fault_reason, "power_on_wait_can", sizeof(g_fault_reason) - 1U);
            g_fault_reason[sizeof(g_fault_reason) - 1U] = '\0';
        }
        return;
    }

    if (strcmp(g_pc_line, "POWER OFF") == 0) {
        request_board_power_off(now_ms);
        return;
    }

    if (sscanf(g_pc_line, "SET %f %f %f", &a, &b, &c) == 3) {
        g_target_requested_rad[0] = clampf(a * DEG_TO_RAD_F, k_soft_min_rad[0], k_soft_max_rad[0]);
        g_target_requested_rad[1] = clampf(b * DEG_TO_RAD_F, k_soft_min_rad[1], k_soft_max_rad[1]);
        g_target_requested_rad[2] = clampf(c * DEG_TO_RAD_F, k_soft_min_rad[2], k_soft_max_rad[2]);
        return;
    }

    if (sscanf(g_pc_line, "GAINS %f %f %f", &a, &b, &c) == 3) {
        const float kp = clampf(a, 0.0f, 1.500f);
        const float kd = clampf(b, 0.0f, 0.030f);
        g_user_torque_cap_nm = clampf(c, 0.0f, USER_TORQUE_CAP_MAX_NM);
        for (uint8_t index = 0U; index < LEG_JOINTS; ++index) {
            g_kp[index] = kp;
            g_kd[index] = kd;
        }
        return;
    }

    /* Direction test only: a bounded motor-torque pulse for 200 ms. */
    if (sscanf(g_pc_line, "PULSE %u %f", &joint, &a) == 2) {
        if (g_mode == POC_ARMED && joint < LEG_JOINTS) {
            g_pulse_joint = (int8_t)joint;
            g_pulse_joint_torque_nm = clampf(a, -PULSE_MAX_TORQUE_NM, PULSE_MAX_TORQUE_NM);
            g_pulse_until_ms = now_ms + PULSE_DURATION_MS;
        }
        return;
    }

    set_fault("bad_pc_command");
}

/* -------------------------- Public controller API ------------------------- */

void LegPdPoc_Init(void)
{
    /* FDCAN1 is the sole motor bus. bsp_can_init currently also starts CAN2;
       that is harmless as long as CAN2 is physically unused. */
    board_motor_power_set(false);
    board_peripheral_5v_enable();
    HAL_Delay(5U);  /* allow the controlled UART/CAN 5 V rail to rise */
    bsp_can_init();
    uart_start_receivers();

    /* IMU is only brought up and polled for logging in this POC. */
    g_imu_error = BMI088_init();
    g_imu_ok = (g_imu_error == 0U);

    g_pc_last_command_ms = HAL_GetTick();
    strncpy(g_fault_reason, g_imu_ok ? "boot_wait_zero" : "imu_init_fail",
            sizeof(g_fault_reason) - 1U);
    g_fault_reason[sizeof(g_fault_reason) - 1U] = '\0';
}

void LegPdPoc_Control1kHz(void)
{
    const uint32_t now_ms = HAL_GetTick();

    if (g_power_off_pending && (int32_t)(now_ms - g_power_off_at_ms) >= 0) {
        board_motor_power_set(false);
        g_power_off_pending = false;
    }

    if (g_pc_line_ready) {
        g_pc_line_ready = false;
        handle_pc_line(now_ms);
    }

    /* Keep the board IMU read at the same 1 kHz cadence as the control loop.
       Its data is logged only; it is intentionally outside the PD equation. */
    if (g_imu_ok) {
        BMI088_read(g_imu_gyro, g_imu_accel, &g_imu_temperature);
    }

    if (g_arm_sequence != ARM_SEQUENCE_IDLE) {
        service_arm_sequence(now_ms);
        return;
    }

    if (g_mode != POC_ARMED) {
        send_zero_and_idle_once();
        return;
    }

    if (!g030_is_healthy(now_ms)) {
        set_fault("g030_lost");
        send_zero_and_idle_once();
        return;
    }
    if (!can_is_healthy(now_ms)) {
        set_fault("can_fault");
        send_zero_and_idle_once();
        return;
    }
    if ((now_ms - g_pc_last_command_ms) > PC_COMMAND_TIMEOUT_MS) {
        disarm("pc_timeout");
        return;
    }

    for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
        if (g_encoder[joint].q_rad < k_hard_min_rad[joint] ||
            g_encoder[joint].q_rad > k_hard_max_rad[joint]) {
            set_fault("joint_limit");
            send_zero_and_idle_once();
            return;
        }
    }

    /* Wait until every axis confirms closed loop before applying nonzero torque. */
    for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
        if (g_motor[joint].axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
            motor_send_torque(joint, 0.0f);
            return;
        }
    }

    for (uint8_t joint = 0U; joint < LEG_JOINTS; ++joint) {
        const float max_target_step = TARGET_SLEW_RAD_S * 0.001f;
        const float target_error = g_target_requested_rad[joint] - g_target_applied_rad[joint];
        g_target_applied_rad[joint] += clampf(target_error, -max_target_step, max_target_step);

        float joint_torque_nm =
            g_kp[joint] * (g_target_applied_rad[joint] - g_encoder[joint].q_rad)
            - g_kd[joint] * g_encoder[joint].dq_rad_s;

        if (g_pulse_joint == (int8_t)joint && now_ms < g_pulse_until_ms) {
            joint_torque_nm += g_pulse_joint_torque_nm;
        }

        const float cap = clampf(g_user_torque_cap_nm, 0.0f, HARD_TORQUE_CAP_NM);
        joint_torque_nm = clampf(joint_torque_nm, -cap, cap);

        const float motor_torque_nm = k_motor_torque_sign[joint] * joint_torque_nm;
        g_last_tau_motor_nm[joint] = motor_torque_nm;
        motor_send_torque(joint, motor_torque_nm);
    }

    if (g_pulse_joint >= 0 && now_ms >= g_pulse_until_ms) {
        g_pulse_joint = -1;
    }
}

void LegPdPoc_Service(void)
{
    static uint32_t last_status_ms = 0U;
    const uint32_t now_ms = HAL_GetTick();

    update_status_led(now_ms);

    /* A 320-byte 115200-baud frame needs < 30 ms.  Do not let a missed DMA
     * completion interrupt permanently silence the PC telemetry. */
    if (g_uart1_tx_busy) {
        if ((now_ms - g_uart1_tx_started_ms) <= 50U) return;
        (void)HAL_UART_AbortTransmit(&huart1);
        g_uart1_tx_busy = false;
        g_uart1_tx_timeout_count++;
    }

    /* 10 Hz is ample for PC/RL diagnostics and keeps the interactive console
     * readable.  The G030/IMU/control paths remain at 1 kHz. */
    if ((now_ms - last_status_ms) < 100U) return;
    last_status_ms = now_ms;

    const char *mode = (g_mode == POC_ARMED) ? "ARMED" :
                       (g_mode == POC_FAULT) ? "FAULT" : "SAFE";
    const uint8_t foot_down = (g_g030_flags & G030_FLAG_FOOT_PRESSED) != 0U;
    const uint32_t g030_age = sample_age_ms(now_ms, g_g030_last_rx_ms);

    const int written = snprintf((char *)g_uart1_tx_dma, sizeof(g_uart1_tx_dma),
        "S t=%lu mode=%s foot=%u gage=%lu flags=%02X q=[%.2f,%.2f,%.2f] "
        "dq=[%.2f,%.2f,%.2f] des=[%.2f,%.2f,%.2f] tau=[%.4f,%.4f,%.4f] "
        "axis=[%u,%u,%u] e=[%08lX,%08lX,%08lX] can1rx=%lu last=0x%03X "
        "pwr=%u imu=%u grz=%.2f uartto=%lu fault=%s\r\n",
        (unsigned long)now_ms, mode, foot_down, (unsigned long)g030_age, g_g030_flags,
        g_encoder[0].q_rad * RAD_TO_DEG_F,
        g_encoder[1].q_rad * RAD_TO_DEG_F,
        g_encoder[2].q_rad * RAD_TO_DEG_F,
        g_encoder[0].dq_rad_s * RAD_TO_DEG_F,
        g_encoder[1].dq_rad_s * RAD_TO_DEG_F,
        g_encoder[2].dq_rad_s * RAD_TO_DEG_F,
        g_target_applied_rad[0] * RAD_TO_DEG_F,
        g_target_applied_rad[1] * RAD_TO_DEG_F,
        g_target_applied_rad[2] * RAD_TO_DEG_F,
        g_last_tau_motor_nm[0], g_last_tau_motor_nm[1], g_last_tau_motor_nm[2],
        g_motor[0].axis_state, g_motor[1].axis_state, g_motor[2].axis_state,
        (unsigned long)g_motor[0].axis_error,
        (unsigned long)g_motor[1].axis_error,
        (unsigned long)g_motor[2].axis_error,
        (unsigned long)can1_raw_rx_count, (unsigned int)can1_last_rx_id,
        g_motor_power_on ? 1U : 0U,
        g_imu_ok ? 1U : 0U, g_imu_gyro[2],
        (unsigned long)g_uart1_tx_timeout_count, g_fault_reason);

    if (written > 0 && written < (int)sizeof(g_uart1_tx_dma)) {
        g_uart1_tx_busy = true;
        g_uart1_tx_started_ms = now_ms;
        if (HAL_UART_Transmit_DMA(&huart1, g_uart1_tx_dma, (uint16_t)written) != HAL_OK) {
            g_uart1_tx_busy = false;
        }
    }
}
