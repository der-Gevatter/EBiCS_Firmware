/*
 * systick.h
 *
 *  Created on: 03.04.2026
 *      Author: stephan
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#include <stdint.h>

/* initialize SysTick to 1ms tick; call early after SystemCoreClock is valid */
void systick_init(void);

/* returns milliseconds since systick_init (wraps at 2^32 ms ~ 49.7 days) */
uint32_t systick_ms(void);

/* simple blocking delay in ms */
void delay_ms(uint32_t ms);

#endif /* SYSTICK_H_ */
