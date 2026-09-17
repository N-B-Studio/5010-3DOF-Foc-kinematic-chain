/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 1 kHz three-AS5047P encoder / foot-switch UART node
  ******************************************************************************
  * Target: STM32G030F6Px, generated from Ben's current CubeMX pinout.
  *
  * Pin assignment:
  *   PA0  STATUS LED, active-low (board LED2)
  *   PA1  SPI1_SCK
  *   PA2  SPI1_MOSI / AS5047P DI
  *   PA3  AS5047P #1 CSn, active-low (knee)
  *   PA4  AS5047P #2 CSn, active-low (hip)
  *   PA5  AS5047P #3 CSn, active-low (ankle)
  *   PA6  SPI1_MISO / AS5047P DO
  *   PA8  FOOT_SW, active-low, internal pull-up
  *   PB3  USART1_TX, 1,000,000 baud, 8N1
  *   TIM3 update interrupt: 1 kHz
  *
  * The code transmits one fixed 19-byte CRC-protected packet every 1 ms.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN 0 */

/* ---------- Board and protocol constants ---------- */
#define NODE_ID                         1U
#define CONTROL_HZ                      1000U
#define SPI_TIMEOUT_MS                  1U
#define UART_TIMEOUT_MS                 2U
#define ENCODER_COUNT                   3U
#define ENCODER_FAULT_STREAK_LIMIT      5U
#define UART_FAULT_STREAK_LIMIT         5U
#define FOOT_DEBOUNCE_SAMPLES           5U

/*
 * Physical SPI channels, confirmed by Ben's leg test from foot to hip:
 *   Encoder3 = ankle, Encoder1 = knee, Encoder2 = hip.
 *
 * The UART protocol is deliberately logical rather than SPI-channel order:
 *   angle[0] = hip, angle[1] = knee, angle[2] = ankle.
 * This matches H723 q[0..2] and Mini ODrive CAN node IDs 1, 2, 3.
 */
#define PHYSICAL_ENCODER_1               0U
#define PHYSICAL_ENCODER_2               1U
#define PHYSICAL_ENCODER_3               2U
#define UART_ANGLE_HIP_SOURCE            PHYSICAL_ENCODER_2
#define UART_ANGLE_KNEE_SOURCE           PHYSICAL_ENCODER_1
#define UART_ANGLE_ANKLE_SOURCE          PHYSICAL_ENCODER_3

/* AS5047P SPI: read ANGLECOM (address 0x3FFF, even parity included). */
#define AS5047P_READ_ANGLE_COMMAND      0xFFFFU
#define AS5047P_READ_ERRFL_COMMAND      0x4001U
#define AS5047P_NOP_COMMAND             0x0000U
#define AS5047P_ANGLE_MASK              0x3FFFU
#define AS5047P_ERROR_FLAG              0x4000U

/* Final UART frame: 2 SOF + 15 payload + 2 CRC = 19 bytes. */
#define FRAME_SOF0                      0xA5U
#define FRAME_SOF1                      0x5AU
#define FRAME_VERSION                   0x03U
#define FRAME_SIZE                      19U

/* flags byte in every UART packet */
#define FLAG_FOOT_PRESSED               (1U << 0)
#define FLAG_ENCODER_1_VALID            (1U << 1)
#define FLAG_ENCODER_2_VALID            (1U << 2)
#define FLAG_ENCODER_3_VALID            (1U << 3)
#define FLAG_NODE_FAULT                 (1U << 4)

typedef struct {
  uint16_t angle_raw;                           /* 0 ... 16383 */
  uint8_t fault_streak;
} EncoderState;

static EncoderState g_encoder[ENCODER_COUNT];
static volatile uint32_t g_tick_due = 0U;
static uint32_t g_time_ms = 0U;
static uint16_t g_sequence = 0U;
static uint8_t g_uart_fault_streak = 0U;
static bool g_foot_pressed = false;
static uint8_t g_foot_debounce_count = 0U;

static GPIO_TypeDef * const g_cs_ports[ENCODER_COUNT] = {
  GPIOA, GPIOA, GPIOA
};

static const uint16_t g_cs_pins[ENCODER_COUNT] = {
  GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5
};

static void LED_Set(bool on)
{
  /* Board LED2 is connected from 3V3 to PA0, hence PA0 low = LED on. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool AS5047P_HasEvenParity(uint16_t word)
{
  uint8_t parity = 0U;
  uint8_t bit;

  for (bit = 0U; bit < 16U; ++bit) {
    parity ^= (uint8_t)((word >> bit) & 0x1U);
  }
  return (parity == 0U);
}

static void AS5047P_Delay350ns(void)
{
  volatile uint32_t i;

  /* AS5047P datasheet: tCSn high between transmissions and tL (CSn falling
   * edge to first CLK rising edge) are both >= 350 ns.
   * At 64 MHz, 32 NOPs are >= 500 ns, excluding loop overhead. */
  for (i = 0U; i < 32U; ++i) {
    __NOP();
  }
}

static bool AS5047P_ReadRegister(uint8_t index, uint16_t command,
                                 uint16_t *response)
{
  uint16_t tx_word;
  uint16_t rx_word;
  HAL_StatusTypeDef status;

  /* The AS5047P latches one command at CSn rising.  Thus command and NOP
   * must be two separate 16-bit frames with a CSn-high interval between. */
  HAL_GPIO_WritePin(g_cs_ports[index], g_cs_pins[index], GPIO_PIN_RESET);
  AS5047P_Delay350ns();

  tx_word = command;
  rx_word = 0U;
  status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx_word,
                                   (uint8_t *)&rx_word, 1U, SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(g_cs_ports[index], g_cs_pins[index], GPIO_PIN_SET);

  if (status == HAL_OK) {
    AS5047P_Delay350ns();

    HAL_GPIO_WritePin(g_cs_ports[index], g_cs_pins[index], GPIO_PIN_RESET);
    AS5047P_Delay350ns();
    tx_word = AS5047P_NOP_COMMAND;
    rx_word = 0U;
    status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx_word,
                                     (uint8_t *)&rx_word, 1U, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(g_cs_ports[index], g_cs_pins[index], GPIO_PIN_SET);
  }

  if (status != HAL_OK) {
    return false;
  }

  *response = rx_word;
  return true;
}

static void AS5047P_ClearErrorRegister(uint8_t index)
{
  uint16_t ignored_response;

  /* ERRFL = 0x0001. The AS5047P clears it automatically after a read. */
  (void)AS5047P_ReadRegister(index, AS5047P_READ_ERRFL_COMMAND,
                              &ignored_response);
}

static bool AS5047P_ReadAngle(uint8_t index, uint16_t *angle_raw)
{
  uint16_t rx_word;

  if (!AS5047P_ReadRegister(index, AS5047P_READ_ANGLE_COMMAND, &rx_word)) {
    return false;
  }

  if ((!AS5047P_HasEvenParity(rx_word)) || ((rx_word & AS5047P_ERROR_FLAG) != 0U)) {
    return false;
  }

  *angle_raw = (uint16_t)(rx_word & AS5047P_ANGLE_MASK);
  return true;
}

static void Encoder_StoreValidSample(EncoderState *encoder, uint16_t raw)
{
  encoder->angle_raw = raw;
  encoder->fault_streak = 0U;
}

static void Encoder_StoreInvalidSample(EncoderState *encoder)
{
  if (encoder->fault_streak < 255U) {
    encoder->fault_streak++;
  }

}

static uint16_t CRC16_CCITT(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i;
  uint8_t bit;

  for (i = 0U; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (bit = 0U; bit < 8U; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static void PutU16LE(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)(value >> 8);
}

static void PutU32LE(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value & 0xFFU);
  dst[1] = (uint8_t)((value >> 8) & 0xFFU);
  dst[2] = (uint8_t)((value >> 16) & 0xFFU);
  dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static bool UART_SendFrame(uint8_t valid_mask, bool foot_pressed, bool node_fault)
{
  uint8_t frame[FRAME_SIZE];
  uint8_t flags = 0U;
  uint8_t logical_valid_mask = 0U;
  uint16_t crc;
  HAL_StatusTypeDef status;

  if (foot_pressed) {
    flags |= FLAG_FOOT_PRESSED;
  }
  if ((valid_mask & (1U << UART_ANGLE_HIP_SOURCE)) != 0U) {
    logical_valid_mask |= (1U << 0);
  }
  if ((valid_mask & (1U << UART_ANGLE_KNEE_SOURCE)) != 0U) {
    logical_valid_mask |= (1U << 1);
  }
  if ((valid_mask & (1U << UART_ANGLE_ANKLE_SOURCE)) != 0U) {
    logical_valid_mask |= (1U << 2);
  }
  flags |= (uint8_t)(logical_valid_mask << 1);
  if (node_fault) {
    flags |= FLAG_NODE_FAULT;
  }

  frame[0] = FRAME_SOF0;
  frame[1] = FRAME_SOF1;
  frame[2] = FRAME_VERSION;
  frame[3] = NODE_ID;
  PutU16LE(&frame[4], g_sequence++);
  PutU32LE(&frame[6], g_time_ms);
  frame[10] = flags;
  PutU16LE(&frame[11], g_encoder[UART_ANGLE_HIP_SOURCE].angle_raw);
  PutU16LE(&frame[13], g_encoder[UART_ANGLE_KNEE_SOURCE].angle_raw);
  PutU16LE(&frame[15], g_encoder[UART_ANGLE_ANKLE_SOURCE].angle_raw);

  crc = CRC16_CCITT(&frame[2], 15U); /* version through angle_3 */
  PutU16LE(&frame[17], crc);

  status = HAL_UART_Transmit(&huart1, frame, FRAME_SIZE, UART_TIMEOUT_MS);
  return (status == HAL_OK);
}

static bool Node_HasFault(void)
{
  uint8_t i;

  if (g_uart_fault_streak >= UART_FAULT_STREAK_LIMIT) {
    return true;
  }

  for (i = 0U; i < ENCODER_COUNT; ++i) {
    if (g_encoder[i].fault_streak >= ENCODER_FAULT_STREAK_LIMIT) {
      return true;
    }
  }
  return false;
}

static bool Foot_Update1kHz(void)
{
  const bool raw_pressed =
      (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET);

  if (raw_pressed == g_foot_pressed) {
    g_foot_debounce_count = 0U;
  } else {
    g_foot_debounce_count++;
    if (g_foot_debounce_count >= FOOT_DEBOUNCE_SAMPLES) {
      g_foot_pressed = raw_pressed;
      g_foot_debounce_count = 0U;
    }
  }

  return g_foot_pressed;
}

static void Node_UpdateLED(bool node_fault)
{
  if (!node_fault) {
    LED_Set(true);                        /* healthy: steady on */
  } else {
    LED_Set((g_time_ms % 200U) < 100U);  /* fault: 5 Hz blink */
  }
}

static void Node_Tick1kHz(void)
{
  uint8_t i;
  uint8_t valid_mask = 0U;
  uint16_t raw;
  bool foot_pressed;
  bool node_fault;
  bool uart_ok;

  g_time_ms++;

  for (i = 0U; i < ENCODER_COUNT; ++i) {
    if (AS5047P_ReadAngle(i, &raw)) {
      Encoder_StoreValidSample(&g_encoder[i], raw);
      valid_mask |= (uint8_t)(1U << i);
    } else {
      Encoder_StoreInvalidSample(&g_encoder[i]);
    }
  }

  /* PA8 is active-low and debounced for 5 consecutive 1 kHz samples. */
  foot_pressed = Foot_Update1kHz();

  node_fault = Node_HasFault();
  uart_ok = UART_SendFrame(valid_mask, foot_pressed, node_fault);
  if (uart_ok) {
    g_uart_fault_streak = 0U;
  } else if (g_uart_fault_streak < 255U) {
    g_uart_fault_streak++;
  }

  /* Include a just-detected UART failure in the LED health decision. */
  Node_UpdateLED(Node_HasFault());
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    /* The interrupt never talks to SPI or UART; it only schedules one task. */
    if (g_tick_due < 0xFFFFFFFFU) {
      g_tick_due++;
    }
  }
}

/* USER CODE END 0 */

int main(void)
{
  uint8_t i;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  LED_Set(true);
  /* Sensor requires up to 10 ms after power-up before its first valid angle.
   * Also clear any sticky ERRFL bits left by a previous firmware revision. */
  HAL_Delay(15U);
  g_foot_pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET);
  for (i = 0U; i < ENCODER_COUNT; ++i) {
    AS5047P_ClearErrorRegister(i);
  }
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END 2 */

  while (1) {
    bool run_tick = false;

    __disable_irq();
    if (g_tick_due > 0U) {
      g_tick_due--;
      run_tick = true;
    }
    __enable_irq();

    if (run_tick) {
      Node_Tick1kHz();
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA0 high means LED off; CSn high means all encoders deselected. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
                    GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __disable_irq();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* A fatal initialization error is also visible as a fast active-low blink. */
  while (1) {
    volatile uint32_t delay;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    for (delay = 0U; delay < 150000U; ++delay) {
      __NOP();
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    for (delay = 0U; delay < 150000U; ++delay) {
      __NOP();
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
