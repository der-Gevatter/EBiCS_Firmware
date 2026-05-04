/*
 * gpio_cmsis.c
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#include "gpio_cmsis.h"
#include "stm32f103x6.h"
#include "main.h"

/* Helper: enable clock for Port */
static void gpio_enable_clock_for_port(GPIO_TypeDef *port)
{
    if (port == GPIOA) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    else if (port == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    else if (port == GPIOC) RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    (void)RCC->APB2ENR;
}

/* Helper: configure single pin as input pull-up (pin is index 0..15) */
/* pin_mask must be exactly one bit (HAL format e.g. 0x0020). */
void gpio_config_input_pu(GPIO_TypeDef *GPIOx, uint16_t pin_mask)
{
    if (pin_mask == 0) return;
    /* ensure single-bit mask */
    if ((pin_mask & (pin_mask - 1)) != 0) return;

    gpio_enable_clock_for_port(GPIOx);

    /* compute pin index 0..15 */
    uint8_t pin = __builtin_ctz(pin_mask); // count trailing zeros

    /* set ODR bit = 1 to select pull-up */
    GPIOx->ODR |= (1U << pin);

    uint32_t shift = (pin & 7) * 4;
    if (pin < 8) {
        GPIOx->CRL &= ~(0xFUL << shift);
        GPIOx->CRL |=  (0x8UL << shift); // MODE=00, CNF=10 -> input pull-up/pull-down
    } else {
        GPIOx->CRH &= ~(0xFUL << shift);
        GPIOx->CRH |=  (0x8UL << shift);
    }
}

/* configure single pin as output push-pull, 2MHz (low speed) or AF push-pull if af==1 */
void gpio_config_output_pp_af(GPIO_TypeDef *GPIOx, uint16_t pin_mask, int spd, int af)
{
    if (pin_mask == 0) return;
    /* ensure single-bit mask */
    if ((pin_mask & (pin_mask - 1)) != 0) return;

    gpio_enable_clock_for_port(GPIOx); // enable port clock

    /* compute pin index 0..15 */
    uint8_t pin = __builtin_ctz(pin_mask); // count trailing zeros
    uint32_t shift = (pin & 7) * 4;
    uint32_t val = 0;

    /* setting mode based on speed (spd) */
    switch(spd){
        case 1:  val = 0x1UL; break; // 10MHz
        case 2:  val = 0x2UL; break; // 2MHz
        case 3:  val = 0x3UL; break; // 50MHz
        default: val = 0x2UL; break; // fallback to 2MHz
    }

    if (af) {
        val |= (0x2UL << 2); /* CNF = 10 -> AF push-pull */
    } else {
        /* CNF = 00 -> GP push-pull (val already MODE bits) */
    }

    if (pin < 8) {
        GPIOx->CRL &= ~(0xFUL << shift); // clear field
        GPIOx->CRL |=  (val << shift);   // set MODE/CNF
    } else {
        GPIOx->CRH &= ~(0xFUL << shift); // clear field
        GPIOx->CRH |=  (val << shift);   // set MODE/CNF
    }
}

/* Helper: enable correct EXTI IRQ for given pin mask */
static void enable_exti_irq_for_pin(uint16_t pin_mask, uint32_t priority)
{
    if (pin_mask == 0) return;
    /* ensure single-bit mask */
    if ((pin_mask & (pin_mask - 1)) != 0) return;

    /* compute pin index 0..15 */
    uint8_t pin = __builtin_ctz(pin_mask); // count trailing zeros

    if (pin <= 4) {
        NVIC_SetPriority((IRQn_Type)(EXTI0_IRQn + pin), priority);
        NVIC_EnableIRQ((IRQn_Type)(EXTI0_IRQn + pin));
    } else if (pin <= 9) {
        NVIC_SetPriority(EXTI9_5_IRQn, priority);
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    } else {
        NVIC_SetPriority(EXTI15_10_IRQn, priority);
        NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
}

/* Helper: configure one pin (mask) of a port as analog inputs */
void gpio_config_analog_pin(GPIO_TypeDef *GPIOx, uint16_t pin_mask)
{
    if (pin_mask == 0) return;
    /* ensure single-bit mask */
    if ((pin_mask & (pin_mask - 1)) != 0) return;

    /* compute pin index 0..15 */
    gpio_enable_clock_for_port(GPIOx);

    uint8_t pin = __builtin_ctz(pin_mask); // count trailing zeros

    uint32_t shift = (pin & 7) * 4;
    if (pin < 8) {
    	GPIOx->CRL &= ~(0xFUL << shift); /* Set MODE=00, CNF=00 => clear the 4-bit field */
    } else {
    	GPIOx->CRH &= ~(0xFUL << shift);
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
    gpio_config_output_pp_af(LED_GPIO_Port, LED_Pin, 2, 0);

    /* LIGHT pin output push-pull */
    gpio_config_output_pp_af(LIGHT_GPIO_Port, LIGHT_Pin, 2, 0);

    /* BRAKE_LIGHT pin output push-pull */
    gpio_config_output_pp_af(BRAKE_LIGHT_GPIO_Port, BRAKE_LIGHT_Pin, 2, 0);

    /* Map EXTI lines to port B for lines 5 and 8 via AFIO->EXTICR */
    /* EXTI5 is in EXTICR[1] (lines 4..7), position for line5: bits [7:4] */
    AFIO->EXTICR[1] &= ~(0xFUL << 4);
    AFIO->EXTICR[1] |= (0x1UL << 4); // 0x1 = Port B

    /* EXTI8 is in EXTICR[2] (lines 8..11), position for line8: bits [3:0] */
    AFIO->EXTICR[2] &= ~(0xFUL << 0);
    AFIO->EXTICR[2] |= (0x1UL << 0); // Port B

    /* Unmask EXTI lines and set falling trigger only */
    EXTI->IMR |= Speed_EXTI5_Pin | PAS_EXTI8_Pin;
    EXTI->FTSR |= Speed_EXTI5_Pin | PAS_EXTI8_Pin;
    EXTI->RTSR &= ~(Speed_EXTI5_Pin | PAS_EXTI8_Pin);

    /* Enable NVIC for the EXTI lines used, priority as small integer (0..max) */
    enable_exti_irq_for_pin(Speed_EXTI5_Pin, 2);
    enable_exti_irq_for_pin(PAS_EXTI8_Pin, 2);
}
