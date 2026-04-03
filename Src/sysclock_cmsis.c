/*
 * sysclock_cmsis.c
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#include "sysclock_cmsis.h"
#include "stm32f103x6.h"

/* Configure system clock: HSI/2 *16 = 64 MHz */
void SystemClock_Config_CMSIS(void)
{
    /* 0) Ensure PLL is off before changing PLL settings */
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) {}

    /* 1) Enable HSI and wait ready */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    /* 2) Set prescalers and Flash latency/prefetch BEFORE switching clocks */
    /* AHB = SYSCLK/1, APB2 = HCLK/1, APB1 = HCLK/2 */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; /* APB1 = HCLK/2 */

    /* Flash: enable prefetch and set 2 wait states for ~64MHz */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    /* 3) Configure PLL: source = HSI/2, PLLMUL = x16 */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLMULL16; /* check macro name in header */

    /* 4) Enable PLL and wait ready */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* 5) Select PLL as system clock */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

    /* 6) ADC prescaler */
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    /* 7) Update SystemCoreClock and configure SysTick */
    SystemCoreClockUpdate();
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, 0);
}
