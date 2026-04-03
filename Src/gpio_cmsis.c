/*
 * gpio_cmsis.c
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#include "gpio_cmsis.h"
#include "stm32f103x6.h"
#include "main.h"

/* Helper: configure single pin as input pull-up (pin is index 0..15) */
static void gpio_config_input_pu(GPIO_TypeDef *GPIOx, uint32_t pin_num)
{
	if (pin_num > 15) return;

    uint32_t shift = (pin_num & 7) * 4;

    /* Set ODR bit first to select pull-up (ODR=1 => pull-up, ODR=0 => pull-down) */
    GPIOx->ODR |= (1U << pin_num); /* pull-up */

    if (pin_num < 8) {
        GPIOx->CRL &= ~(0xFUL << shift);
        GPIOx->CRL |= (0x8UL << shift); /* MODE=00, CNF=10 -> input pull-up/pull-down */
    } else {
        GPIOx->CRH &= ~(0xFUL << shift);
        GPIOx->CRH |= (0x8UL << shift);
    }
}

/* Helper: configure single pin as output push-pull, 2MHz (low speed) (pin is index 0..15) */
static void gpio_config_output_pp(GPIO_TypeDef *GPIOx, uint32_t pin_num)
{
	if (pin_num > 15) return;

    uint32_t shift = (pin_num & 7) * 4;
    if (pin_num < 8) {
        GPIOx->CRL &= ~(0xFUL << shift);
        GPIOx->CRL |= (0x2UL << shift); /* MODE=10 (2MHz), CNF=00 (GP push-pull) */
    } else {
        GPIOx->CRH &= ~(0xFUL << shift);
        GPIOx->CRH |= (0x2UL << shift);
    }
}

/* Helper: enable correct EXTI IRQ for given pin index */
static void enable_exti_irq_for_pin(uint32_t pin_idx, uint32_t priority)
{
    if (pin_idx <= 4) {
        NVIC_SetPriority((IRQn_Type)(EXTI0_IRQn + pin_idx), priority);
        NVIC_EnableIRQ((IRQn_Type)(EXTI0_IRQn + pin_idx));
    } else if (pin_idx <= 9) {
        NVIC_SetPriority(EXTI9_5_IRQn, priority);
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    } else {
        NVIC_SetPriority(EXTI15_10_IRQn, priority);
        NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
}

/* GPIO init function */
void GPIO_Init_CMSIS(void)
{
    /* Enable clocks for GPIOA, GPIOB, GPIOC and AFIO */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;
    (void)RCC->APB2ENR;

    /* Hall_1, Hall_2 and Hall_3 input pull-up */
    gpio_config_input_pu(Hall_1_GPIO_Port, Hall_1_Pin);
    gpio_config_input_pu(Hall_2_GPIO_Port, Hall_2_Pin);
    gpio_config_input_pu(Hall_3_GPIO_Port, Hall_3_Pin);

    /* Brake pin input pull-up */
    gpio_config_input_pu(Brake_GPIO_Port, Brake_Pin);

    /* Speed_EXTI5 and PAS_EXTI8 as input pull-up */
    gpio_config_input_pu(Speed_EXTI5_GPIO_Port, Speed_EXTI5_Pin);
    gpio_config_input_pu(PAS_GPIO_Port, PAS_EXTI8_Pin);

    /* Set LED output low */
    LED_GPIO_Port->ODR &= ~(1U << LED_Pin);

    /* LED pin output push-pull, low speed */
    gpio_config_output_pp(LED_GPIO_Port, LED_Pin);

    /* LIGHT pin output push-pull */
    gpio_config_output_pp(LIGHT_GPIO_Port, LIGHT_Pin);

    /* BRAKE_LIGHT pin output push-pull */
    gpio_config_output_pp(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin);

    /* Map EXTI lines to port B for lines 5 and 8 via AFIO->EXTICR */
    /* EXTI5 is in EXTICR[1] (lines 4..7), position for line5: bits [7:4] */
    AFIO->EXTICR[1] &= ~(0xFUL << 4);
    AFIO->EXTICR[1] |= (0x1UL << 4); /* 0x1 = Port B */

    /* EXTI8 is in EXTICR[2] (lines 8..11), position for line8: bits [3:0] */
    AFIO->EXTICR[2] &= ~(0xFUL << 0);
    AFIO->EXTICR[2] |= (0x1UL << 0); /* Port B */

    /* Unmask EXTI lines and set falling trigger only */
    EXTI->IMR |= (1U << Speed_EXTI5_Pin) | (1U << PAS_EXTI8_Pin);
    EXTI->FTSR |= (1U << Speed_EXTI5_Pin) | (1U << PAS_EXTI8_Pin);
    EXTI->RTSR &= ~((1U << Speed_EXTI5_Pin) | (1U << PAS_EXTI8_Pin));

    /* Enable NVIC for the EXTI lines used, priority as small integer (0..max) */
    enable_exti_irq_for_pin(Speed_EXTI5_Pin, 2);
    enable_exti_irq_for_pin(PAS_EXTI8_Pin, 2);
}
