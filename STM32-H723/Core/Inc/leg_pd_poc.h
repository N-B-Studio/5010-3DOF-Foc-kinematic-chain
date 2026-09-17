#ifndef LEG_PD_POC_H
#define LEG_PD_POC_H

#include <stdint.h>

/*
 * First safe H723 three-joint outer-PD controller.
 *
 * The generated project keeps ownership of peripheral setup.  main.c calls
 * LegPdPoc_Init() once after MX_*_Init(), then calls LegPdPoc_Control1kHz()
 * from its 1 kHz foreground loop and LegPdPoc_Service() continuously.
 *
 * bsp_fdcan.c already forwards each FDCAN frame to can_parse_feedback().
 * That symbol is implemented in leg_pd_poc.c, so no CAN RX callback change
 * is needed.
 */

void LegPdPoc_Init(void);
void LegPdPoc_Control1kHz(void);
void LegPdPoc_Service(void);

/* Called by the existing bsp_fdcan.c CAN FIFO callback. */
void can_parse_feedback(uint16_t rx_id, uint8_t *data, uint8_t len);

#endif /* LEG_PD_POC_H */
