/*
 * tim2_cmsis.c TIM2 CMSIS-based init and IRQ handling for TIM2 on STM32F103 (STM32FEBKC6T6)
 *
 *  Created on: 21.04.2026
 *      Author: stephan
 */

#include "tim_cmsis.h"
#include "stm32f103x6.h"
#include "main.h"
#include "config.h"
#include "gpio_cmsis.h"

/* Local storage for last CCR1 captured value */
static volatile uint32_t tim2_last_ccr1 = 0U;

/* Forward: configure GPIO pins and AF remap for TIM2 CH1..CH3 */
static void tim2_gpio_init(void)
{
    /* Enable AFIO & GPIOA/B clocks */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN; // enable AFIO, PA, PB

    /* Remap TIM2 pins (full remap: CH1..CH3 -> PA15,PB3,PB10) */
    AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP; // Clear
    AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_FULLREMAP ; // enable TIM2 full remap
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

    /* PA0/PA1/PA2 as input floating (for hall / capture) */
    /* Configure PA0..PA2 CNF=01 (floating input), MODE=00 */
//    GPIOA->CRL &= ~((0xF << (0*4)) | (0xF << (1*4)) | (0xF << (2*4))); // clear PA0..PA2
//    GPIOA->CRL |=  (0x4 << (0*4)) |  (0x4 << (1*4)) |  (0x4 << (2*4)); // CNF=01, MODE=00 -> floating input
    gpio_config_input_pu(Hall_1_GPIO_Port, Hall_1_Pin);
    gpio_config_input_pu(Hall_2_GPIO_Port, Hall_2_Pin);
    gpio_config_input_pu(Hall_3_GPIO_Port, Hall_3_Pin);
}

/* Initialize TIM2 peripheral */
void TIM2_CMSIS_Init(void)
{
    /* enable TIM2 clock on APB1 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable TIM2 peripheral

    /* reset TIM2 to known state */
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST; // assert reset
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST; // release reset

    tim2_gpio_init(); // init pins and AF remap

    /* Prescaler and auto-reload */
    TIM2->PSC = 128U;   	 // prescaler = 128 -> PSC register stores (PSC-1)
    TIM2->ARR = 0xFFFFU;     // period = 65535

    /* CR1: upcounter, ARPE disabled (match HAL setting) */
    TIM2->CR1 = 0U; // ensure default: upcounting, no ARPE

    /* TI1 Selection: 1 -> CH1,2,3 are XORed to TI1 */
    TIM2->CR2 |= TIM_CR2_TI1S;

    /* Configure channel1 as input, TI1FP1 with filter and capture prescaler = /1 */
    TIM2->CCMR1 = 0U;
    /* CC1S = 01 -> TI1 mapped to CC1; IC1PSC = 00; IC1F = 1111 (filter=15) */
    TIM2->CCMR1 |= (3U << 0) | (15U << 4); // CC1S=01, IC1F=1111

    /* Channel2, Channel3 as direct TI (for XOR combined use CC2S/CC3S) */
    /* CC2S = 10 (direct TI2) with same filter; CC3S = 01 (TI3) with same filter */
    TIM2->CCMR1 |= (1U << 8) | (15U << 12); // CC2S=10, IC2F=1111

    TIM2->CCMR2 = 0U;
    TIM2->CCMR2 |= (1U << 0) | (15U << 4); // CC3S=01, IC3F=1111

    /* CCER: enable capture on CH1/CH2/CH3, rising edge (polarity=0) */
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P); // ensure rising
    TIM2->CCER = TIM_CCER_CC1E; // enable captures

    /* Slave mode: Reset mode on TI1F_ED (both edges of TI1) */
    TIM2->SMCR = 0U;
    /* TS = 100 TI1F_ED edge detector */
    TIM2->SMCR |= (4U << 4);   // set TS = TI1F_ED
    /* SMS = 100 -> Reset mode; select Trigger filter = 8 (filter bits 4..7 in SMCR) */
    TIM2->SMCR &= ~(0x7 << 0);
    TIM2->SMCR |=  (4U << 0);  // SMS = 100 Reset mode

    /* Master mode: TRGO reset (default 0) */
    TIM2->CR2 &= ~(0x7 << 4);

    /* Generate update to load prescaler */
    TIM2->EGR |= TIM_EGR_UG;

    /* Clear pending flags */
    TIM2->SR = 0U;

    /* Enable CC1 interrupt */
    //TIM2->DIER |= TIM_DIER_CC1IE | TIM_DIER_UIE;;
    TIM2->DIER |= TIM_DIER_CC1IE;

    /* Configure NVIC: priority 0 and enable IRQ */
    NVIC_SetPriority(TIM2_IRQn, 0);
    NVIC_EnableIRQ(TIM2_IRQn);       // enable TIM2 interrupt
}

/* Start TIM2 counting (counter enabled) */
void TIM2_CMSIS_Start(void)
{
    TIM2->CR1 |= TIM_CR1_CEN; // start counter
}

/* Stop TIM2 */
void TIM2_CMSIS_Stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN; // stop counter
}

/* Return last captured CCR1 value */
uint16_t TIM2_GetCCR1(void)
{
    return (uint16_t)(tim2_last_ccr1 & 0xFFFFU);
}

/* IRQ handler: called from vector table. Processes CC1 interrupt and calls user callback */
void TIM2_Handler_Impl(void)
{
    uint32_t sr = TIM2->SR;
    TIM2->SR = 0; // clear flags

    if (sr & TIM_SR_CC1IF)
    {
        uint32_t capture = TIM2->CCR1;
        TIM2_CaptureCallback(1, capture);
     }
}

/* Weak callback - user can override in application */
__attribute__((weak))
void TIM2_CaptureCallback(uint8_t channel, uint32_t diff)
{
    (void)channel;
    (void)diff;
}
