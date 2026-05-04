/*
 * gpio_cmsis.h
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#ifndef GPIO_CMSIS_H_
#define GPIO_CMSIS_H_

#include "stm32f103x6.h"

/* gpio masks in HAL-format */
#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */

/* state values */
#define GPIO_STATE_RESET 0U
#define GPIO_STATE_SET   1U
#define GPIO_STATE_TOGGLE 2U

/* perform atomic ops */
static inline void gpio_set(GPIO_TypeDef *GPIOx, uint16_t pin_mask) {
	GPIOx->BSRR = pin_mask;           // atomic set
}

static inline void gpio_reset(GPIO_TypeDef *GPIOx, uint16_t pin_mask) {
	GPIOx->BRR = pin_mask;            // atomic reset
}

static inline void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t pin_mask) {
	GPIOx->ODR ^= pin_mask;           // not atomic; protect if concurrent access possible
}

static inline void gpio_write(GPIO_TypeDef *GPIOx, uint16_t pin_mask, uint8_t state) {
    if (state == GPIO_STATE_SET) {
        gpio_set(GPIOx, pin_mask);
    } else if (state == GPIO_STATE_RESET) {
        gpio_reset(GPIOx, pin_mask);
    } else {
        gpio_toggle(GPIOx, pin_mask);
    }
}

/* read input data register and return normalized bit (0 or 1) */
static inline uint8_t gpio_read(GPIO_TypeDef *GPIOx, uint16_t pin_mask) {
    return ( (GPIOx->IDR & pin_mask) ? 1U : 0U );
}

void GPIO_Init_CMSIS(void);
void gpio_config_input_pu(GPIO_TypeDef *GPIOx, uint16_t pin_mask);
void gpio_config_output_pp_af(GPIO_TypeDef *GPIOx, uint16_t pin_mask, int spd, int af);
void gpio_config_analog_pin(GPIO_TypeDef *GPIOx, uint16_t pin_mask);

#endif /* GPIO_CMSIS_H_ */
