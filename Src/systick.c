/*
 * systick.c
 *
 *  Created on: 03.04.2026
 *      Author: stephan
 */

#include "systick.h"
#include "stm32f103x6.h"
#include "stm32f1xx_hal.h"

/* millisecond tick counter incremented by SysTick IRQ */
static volatile uint32_t g_systick_ms = 0U;

void systick_init(void)
{
    /* configure SysTick for 1ms interrupts using SystemCoreClock */
    /* SysTick_Config returns 0 on success, non-zero on failure (if reload > SysTick_LOAD_RELOAD_Msk) */
    if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
        /* configuration failed — spin here or handle error as you see fit */
        while (1) { }
    }
    /* set priority if desired (optional) */
       NVIC_SetPriority(SysTick_IRQn, 3);
}

/* SysTick IRQ handler: increments ms counter */
void SysTick_Handler(void)
{
    g_systick_ms++;
}

/* return current ms tick */
uint32_t systick_ms(void)
{
    return g_systick_ms; /* atomic read of volatile 32-bit on 32-bit cortex-m is safe */
}

/* blocking delay using systick_ms */
void delay_ms(uint32_t ms)
{
    uint32_t start = systick_ms();
    while ((systick_ms() - start) < ms) {
       // __WFI(); /* optional: wait for interrupt to reduce power while waiting */
    }
}
