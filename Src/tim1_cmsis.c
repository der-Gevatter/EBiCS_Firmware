/*
 * tim1_cmsis.c CMSIS register-based init for TIM1 (PWM, complementary outputs, BDTR, CC4 ADC trigger)
 * for STM32F103 (STM32FEBKC6T6)
 *
 *  Created on: 19.04.2026
 *      Author: stephan
 */

#include "tim_cmsis.h"
#include "stm32f103x6.h"
#include "config.h"
#include "gpio_cmsis.h"

/* Configure GPIOs used by TIM1: PA7 (CH1N), PA8 (CH1), PA9 (CH2), PA10 (CH3), PB0 (CH2N), PB1 (CH3N) */
static void tim1_gpio_init(void)
{
    /* enable GPIOA and GPIOB clocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN; // enable gpio clocks

    /* configure PA7 (CRL, pin7) as AF push-pull, 50MHz */
	gpio_config_output_pp_af(GPIOA, GPIO_PIN_7, 3, 1);

    /* configure PA8..PA10 (CRH) as AF push-pull, 50MHz */
	gpio_config_output_pp_af(GPIOA, GPIO_PIN_8, 3, 1);
	gpio_config_output_pp_af(GPIOA, GPIO_PIN_9, 3, 1);
	gpio_config_output_pp_af(GPIOA, GPIO_PIN_10, 3, 1);

    /* configure PB0/PB1 (CRL pins 0 and 1) as AF push-pull, 50MHz */
	gpio_config_output_pp_af(GPIOB, GPIO_PIN_0, 3, 1);
	gpio_config_output_pp_af(GPIOB, GPIO_PIN_1, 3, 1);

    /* enable AFIO clock */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // enable afio

    /* set TIM1 partial remap: MAPR bits [7:6] = 01b -> partial remap */
    AFIO->MAPR &= ~(0x3 << 6); // clear TIM1 remap
    AFIO->MAPR |=  (0x1 << 6); // set TIM1 partial remap
}

/* Configure TIM1 registers for PWM center-aligned1, prescaler=0, ARR=TIM1_ARR_VALUE, dead-time=32, complementary outputs */
void TIM1_CMSIS_Init(void)
{
    /* configure GPIO + AF remap */
    tim1_gpio_init(); // init pins and AF remap

    /* enable TIM1 clock */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // enable tim1 peripheral

    /* reset TIM1 to known state */
    RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST; // assert reset
    RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST; // release reset

    /* prescaler and auto-reload */
    TIM1->PSC = 0U;  // prescaler = 0
    TIM1->ARR = _T;	 // set ARR

    /* CR1: ARPE = 1, CMS = 01 -> center-aligned mode 1 */
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0; // ARR preload + center-aligned1

    /* CR2 cleared initially; MMS will be set for OC4REF below */
    TIM1->CR2 = 0U; // clear CR2

    /* CCMR1: OC1/OC2 PWM1 (OCxM = 110), enable OCxPE for preload */
    TIM1->CCMR1 = (6U << 4) | (1U << 3)     /* OC1M=110 PWM1, OC1PE=1 */
                | (6U << 12) | (1U << 11);  /* OC2M=110 PWM1, OC2PE=1 */ // set CCMR1

    TIM1->CCMR2 = (6U << 4)  | (1U << 3)   /* OC3 Modus & Preload */
                | (6U << 12) | (1U << 11); /* OC4 Modus & Preload */

    /* initial compare values */
    TIM1->CCR1 = 1U; // small non-zero to start
    TIM1->CCR2 = 1U; // small non-zero to start
    TIM1->CCR3 = 1U; // small non-zero to start
    TIM1->CCR4 = (uint32_t)(_T - 60U); // CCR4 used as trigger for ADC injected (slightly off center to reduce noise/ringing)


    /* CCER: enable main and complementary outputs, set complementary polarity low */
    TIM1->CCER = 0U; // clear CCER
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE; // enable CH1 and CH1N
    TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE; // enable CH2 and CH2N
    TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE; // enable CH3 and CH3N
    TIM1->CCER |= TIM_CCER_CC4E;				  // enable CH4

    /* set complementary polarity bits NP = 1 to make complementary outputs active low */
    TIM1->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP; // comp outputs active low

    /* Master mode: MMS = 011 (OC4REF) -> set bits [6:4] = 0b011 */
    TIM1->CR2 &= ~(0x7 << 4); // clear MMS bits
    TIM1->CR2 |=  (0x7 << 4); // set MMS = OC4REF (TRGO)

    /* BDTR: dead-time, OSSR/OSSI enabled, LOCK=0, Break disabled, MOE enable */
    TIM1->BDTR = 0U; // clear BDTR
    TIM1->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR;
    TIM1->BDTR |= (32U & 0xFF); // Dead-time: 32 Ticks @ 64MHz = 500ns
    TIM1->BDTR |= TIM_BDTR_MOE; // enable main output

    /* clear pending status flags */
    TIM1->SR = 0U; // clear status register

    /* enable CC4 interrupt (used for ADC injected triggering/handling) */
    TIM1->DIER |= TIM_DIER_CC4IE; // enable CC4 interrupt

    /* generate update event to load shadow registers */
    TIM1->EGR = TIM_EGR_UG; // reload shadow registers

    /* don't start timer here; call tim1_start_pwm() explicitly to start */
}

/* start PWM by enabling counter */
void tim1_start_pwm(void)
{
    TIM1->CR1 |= TIM_CR1_CEN; // enable counter (start)
}

/* stop PWM by disabling counter */
void tim1_stop_pwm(void)
{
    TIM1->CR1 &= ~TIM_CR1_CEN; // disable counter (stop)
}

/* set CCR value for channel 1..4 */
void tim1_set_ccr(uint8_t channel, uint32_t value)
{
    switch (channel)
    {
        case 1: TIM1->CCR1 = value; break; // write CCR1
        case 2: TIM1->CCR2 = value; break; // write CCR2
        case 3: TIM1->CCR3 = value; break; // write CCR3
        case 4: TIM1->CCR4 = value; break; // write CCR4
        default: break;
    }
}
