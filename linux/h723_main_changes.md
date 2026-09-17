# H723 `main.c` integration

Do **not** retain the old two-motor virtual-spring loop.  It arms motors at
boot and sends torque automatically, so it is not safe for this three-joint
leg.

Keep Cube-generated peripheral initialisation.  Then make the following
minimal application changes.

## 1. Include the controller

In `/* USER CODE BEGIN Includes */` add:

```c
#include "leg_pd_poc.h"
```

## 2. Initialise safely

After all `MX_*_Init()` calls, replace the old `arm_enable(...)`, ODrive mode,
and closed-loop setup with:

```c
/* USER CODE BEGIN 2 */
LegPdPoc_Init();                 /* starts CAN and both UART DMA receivers */
HAL_TIM_Base_Start_IT(&htim2);   /* TIM2 must be exactly 1 kHz */
/* USER CODE END 2 */
```

There must be **no** automatic `Set_Axis_State(CLOSED_LOOP)` and no automatic
motor-power enable at boot. `LegPdPoc_Init()` explicitly sets the Board02
controlled-output pins `PC13` and `PC14` low.  Use the PC command `POWER ON`
only after the hardware is safe and you are ready to receive CAN heartbeats.

## 3. Replace the foreground loop

```c
while (1)
{
    LegPdPoc_Service();          /* non-blocking USART1 telemetry at 50 Hz */

    if (!loop_1khz_flag) {
        continue;
    }
    loop_1khz_flag = 0U;
    LegPdPoc_Control1kHz();
}
```

Keep this timer callback:

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        loop_1khz_flag = 1U;
    }
}
```

`bsp_fdcan.c` already calls `can_parse_feedback()` from its RX FIFO callback;
leave that call in place.  This POC implements that function for nodes 1, 2,
and 3 on **FDCAN1**.
