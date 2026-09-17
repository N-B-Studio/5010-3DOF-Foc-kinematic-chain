python pc_pd_poc.py --port COM4

python3 pc_pd_poc.py --port /dev/ttyACM0

CLEAR
ZERO
POWER ON
ARM

GAINS 0.30 0.004 0.05
PULSE 2 0.02

然后每一级保持 5–10 秒，输入 s 观察姿态与 tau，再进入下一档：

# 0.5 A
GAINS 0.15 0.003 0.080

# 2.0 A
GAINS 0.80 0.010 0.12

SET 0 0 0

最终视频结构，2–3 分钟
10 秒：硬件、双编码器、H723、CAN、三关节标注。
20 秒：命令和状态屏幕，显示 heartbeat、encoder、foot、fault、torque cap。
25 秒：RViz 数字腿与实机同步，单关节方向验证。
30 秒：2 → 5 A 静态支撑曲线和电子秤结果。
35 秒：脚端直线/椭圆轨迹跟踪。
25 秒：下蹲—站起、保持姿势。
15 秒：导轨辅助 hop。
10 秒：安全演示：DISARM 或通信断开，扭矩归零。



# H723 three-joint outer-PD POC

This is the first active controller for the leg, not the final RL interface.
The H723 owns all real-time safety and torque output.  The PC only sends
bounded output-joint targets over USART1.

```text
PC -- USART1, 115200 --> H723 -- FDCAN1, 1 Mbps --> nodes 1, 2, 3
                              ^
                              | USART10, 1 Mbps
                      G030 output encoders + foot switch
```

The provided starting project also maps `PC13` and `PC14` to the Board02's two
PMOS-controlled outputs.  This POC starts with both outputs **off** and will
not turn them on merely because the MCU boots or receives `ARM`.

## What was verified from your supplied files

- The board manual maps PC UART1 to `PA9 TX`, `PA10 RX`.
- It maps the G030 link to `USART10_RX=PE2`, `USART10_TX=PE3`.
- The board CAN1 is `FDCAN1_RX=PD0`, `FDCAN1_TX=PD1`.
- The supplied Mini ODrive CAN document uses `node_id << 5 | command_id`,
  torque command `0x0E`, torque controller mode `1`, and closed-loop state
  `8`.  The POC uses this exact protocol.
- The supplied G030 node has a fixed 19-byte, 1 Mbps UART packet: `A5 5A`,
  version 3, three raw AS5047P angles, foot flag, CRC-16/CCITT-FALSE.

## Required CubeMX changes

### USART1 - PC bridge

- Asynchronous, `115200`, 8-N-1, TX and RX.
- Pins: `PA9=TX`, `PA10=RX`.
- Enable **USART1 RX DMA** and **USART1 TX DMA**.
- Enable USART1 global interrupt.
- Use a USB-to-TTL adapter with **3.3 V TTL**, not an RS-232 adapter.
- Wiring: H723 `TX -> adapter RX`, H723 `RX -> adapter TX`, common GND.

### USART10 - G030 telemetry

- Asynchronous, **1,000,000 baud**, 8-N-1.  The old copied project is
  incorrectly set to 115200 here.
- Pins: `PE2=USART10_RX`, `PE3=USART10_TX`.
- Enable **USART10 RX DMA** and USART10 global interrupt.
- Wiring: G030 `PB3 TX -> H723 PE2 RX`, plus common GND.  H723 PE3 TX is not
  used by the frozen G030 firmware.

### FDCAN1 - all three motors

- Classic CAN, normal mode, 1 Mbps.  Preserve the current 96 MHz FDCAN clock
  and timing: prescaler 6, SyncJumpWidth 2, TimeSeg1 13, TimeSeg2 2.
- `PD0=FDCAN1_RX`, `PD1=FDCAN1_TX`.
- Rx FIFO0 new-message interrupt enabled.
- The three Mini ODrives must have distinct node IDs.  This source assumes
  **1, 2, 3**; change `k_node_id[]` if your actual IDs differ.
- CAN bus needs a 120-ohm terminator at each physical end, not at every motor.

### TIM2 - real 1 kHz

Your supplied H723 clock configuration is 192 MHz SYSCLK.  APB1 is divided by
2, so TIM2 receives 192 MHz.  The copied values `PSC=239, ARR=999` make
**800 Hz**, not 1 kHz.  Set:

```text
TIM2 Prescaler = 191
TIM2 Counter Period = 999
```

Enable TIM2 global interrupt.  Suggested NVIC preemption priorities:

```text
FDCAN1 RX = 0
USART10 = 1
TIM2 = 2
USART1 = 3
```

### BMI088

Keep your existing `SPI2`, `ACC_CS=PC0`, `GYRO_CS=PC3`.  The POC calls
`BMI088_init()` once and reads it at 200 Hz for diagnostics only.  It does not
enter the PD equation yet.

In `BMI088Middleware.c`, replace the fixed 480 MHz delay scale:

```c
ticks = us * 480;
```

with:

```c
ticks = us * (SystemCoreClock / 1000000U);
```

Your provided H723 project runs at 192 MHz, so the old delay is 2.5 times too
long.

## Files to add

Add `leg_pd_poc.c` and `leg_pd_poc.h` to the CubeIDE project.  Apply the three
small changes in `h723_main_changes.md`.  Keep `bsp_fdcan.c` because it already
forwards CAN RX to `can_parse_feedback()`.

Do not reuse the old virtual-spring control code: it uses two CAN buses and
automatically arms motors during boot.

## PC commands

Commands are ASCII, newline terminated.

```text
ZERO                         capture the current all-vertical pose; SAFE only
POWER ON                     explicitly energise Board02 controlled motor outputs
POWER OFF                    zero/IDLE then cut the Board02 controlled outputs
ARM                          enter torque PD; only after ZERO and healthy data
DISARM                       immediate zero torque + ODrive IDLE request
CLEAR                        leave a latched fault; no motor motion
KEEP                         refresh the 250 ms PC watchdog
SET hip knee ankle            desired output angles in degrees, each +/-10 deg
GAINS kp kd torque_cap        shared outer gains; cap cannot exceed 0.010 Nm
PULSE joint torque            +/-0.002 Nm max for 200 ms; direction calibration
```

`POWER ON` / `POWER OFF` apply when your motor 24 V goes through the two
controlled XT30 outputs.  They use `PC13` and `PC14`, matching the supplied
starting project.  If the Mini ODrives have their own permanent 24 V source,
set `POC_USE_BOARD_MOTOR_POWER` to `0` in `leg_pd_poc.c`; then `ARM` does not
require the `POWER ON` command.

The PC program maintains `KEEP` at 50 Hz after `ARM`.  Closing it or losing
the USB serial link disarms the H723 within 250 ms.

## Safe bring-up order

1. **No mechanics moving:** connect H723 to the PC and G030.  Do not connect
   motor power.  Confirm status lines show `mode=SAFE`, `flags=0E/0F`, three
   changing output angles, and `gage` below 10 ms. `pwr=0` is expected.
2. **CAN telemetry only:** with the leg mechanically supported, issue
   `POWER ON` if the motors use the Board02 controlled outputs; otherwise
   power their external supply. Do not issue ARM. Confirm `pwr=1`,
   `axis=[1,1,1]` or your known idle states, and all error words are zero.
   If nodes are not 1,2,3, change `k_node_id[]` before proceeding.
3. Put the three output links in the upright mechanical pose and run `ZERO`.
   Confirm `q=[0,0,0]` while still SAFE.
4. Mechanically support the leg, set `GAINS 0 0 0.002`, then `ARM`.  This only
   enters torque mode with zero commanded torque.
5. Run `PULSE 0 0.002`.  A positive pulse must make reported `q[0]` increase.
   If it decreases, disarm, change that element of `k_motor_torque_sign[]` to
   `-1.0f`, rebuild, and repeat.  Perform joints 1 and 2 one at a time.
6. With all signs correct: `GAINS 0.020 0.0008 0.006`, `ARM`, then command
   only `SET 1 0 0`, then return to `SET 0 0 0`.  Do one joint at a time.
7. End every bench session with `DISARM`, then `POWER OFF` if using the
   Board02 controlled outputs. Only after all three one-degree tests are
   correct should you use up to
   `SET 5 5 5` or increase torque cap toward 0.010 Nm.

Never test a new motor direction with a 5-degree command.  Keep the leg on
the vertical guide, keep a hand on the hardware emergency stop, and do not
save any ODrive configuration from this POC.
