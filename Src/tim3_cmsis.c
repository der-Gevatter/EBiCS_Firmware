/*
 * tim3_cmsis.c IM3 CMSIS-based init for TIM3
 *
 *  Created on: 26.04.2026
 *      Author: stephan
 */

#include "tim_cmsis.h"
#include "stm32f103x6.h"
#include "main.h"
#include "config.h"
#include "gpio_cmsis.h"

void TIM3_CMSIS_Init(void)
{
    /* Enable TIM3 clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3 peripheral

    /* Reset TIM3 to known state */
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST;  // assert reset
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST; // release reset

    /* Base configuration: prescaler and auto-reload */
    TIM3->PSC = 0U;                 // prescaler = 0 -> PSC register stores PSC (value)
    TIM3->ARR = 7813U;		        // auto-reload = 7813

    /* CR1: upcounter */
    TIM3->CR1 = 0U; // upcounting, ARPE = 0

    /* Configure Channel 1 as Output Compare, Toggle mode */
    /* OC1M = 011 (toggle on match), OC1PE = preload disabled */
    TIM3->CCMR1 = 0U;
    /* OC1M[2:0] = 011 (toggle), OC1PE = 0 */
    TIM3->CCMR1 |= (3U << 4); // OC1M bits are at 6:4 for CCMR1; set to 011 -> (3<<4)

    /* Set CCR1 to half period so we get OC1 match events at regular intervals. */
    /* Choose CCR1 = ARR/2 to center the toggle (produces a match twice per full period). */
    TIM3->CCR1 = (7813 / 2U);

    /* Enable CC1 output (CC1E = 1) in CCER, active on rising by default (polarity 0) */
    TIM3->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP); // ensure polarity = rising (default)
    TIM3->CCER |= TIM_CCER_CC1E; // enable output for channel 1

    /* Master mode selection: TRGO = OC1 (MMS = 011) -> set CR2 MMS bits accordingly */
    /* MMS bits are CR2[6:4]; value for OC1 is (MMS_1 | MMS_0) -> binary 011 -> numeric 3. */
    TIM3->CR2 &= ~(0x7U << 4);            // clear MMS
    TIM3->CR2 |= (3U << 4);               // MMS = 011 -> TRGO = OC1 (capture/compare match 1)

    /* Generate update to load prescaler */
    TIM3->EGR = TIM_EGR_UG;

    /* Clear pending flags */
    TIM3->SR = 0U;

    /* Enable update interrupt */
    TIM3->DIER |= TIM_DIER_UIE;

    /* NVIC: mirror HAL MSP that set priority 3 and enabled IRQ; keep same priority */
    NVIC_SetPriority(TIM3_IRQn, 3);
    NVIC_EnableIRQ(TIM3_IRQn); // enable TIM3 interrupt
}

/* Start TIM3 counting */
void TIM3_CMSIS_Start(void)
{
    TIM3->CR1 |= TIM_CR1_CEN; // start counter
}

/* Stop TIM3 */
void TIM3_CMSIS_Stop(void)
{
    TIM3->CR1 &= ~TIM_CR1_CEN; // stop counter
}

/* IRQ handler implementation stub (weak callback can be overridden in application) */
void TIM3_Handler_Impl(void)
{
    uint32_t sr = TIM3->SR;
    TIM3->SR = 0U; // clear all flags

    /* If you need update/cc interrupts, check flags here and call callbacks. */
    /* Currently left minimal; override TIM3_Callback_* in application as needed. */
    if (sr & TIM_SR_UIF)
    {
        /* weak callback for update event */
        TIM3_UpdateCallback();
    }
    if (sr & TIM_SR_CC1IF)
    {
        /* weak callback for CC1 match */
        TIM3_CC1Callback();
    }
}

/* Weak callbacks - application may override */
__attribute__((weak))
void TIM3_UpdateCallback(void)
{
    /* default: do nothing */
}

__attribute__((weak))
void TIM3_CC1Callback(void)
{
    /* default: do nothing */
}
