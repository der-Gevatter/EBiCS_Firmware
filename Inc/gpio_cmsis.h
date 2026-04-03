/*
 * gpio_cmsis.h
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#ifndef GPIO_CMSIS_H_
#define GPIO_CMSIS_H_

#include "stm32f103x6.h"

/* state values */
#define GPIO_STATE_RESET 0U
#define GPIO_STATE_SET   1U
#define GPIO_STATE_TOGGLE 2U

/* convert numeric pin to mask and perform atomic ops */
static inline void gpio_set(GPIO_TypeDef *port, uint8_t pin) {
    uint32_t mask = (1U << pin);
    port->BSRR = mask;           // atomic set
}

static inline void gpio_reset(GPIO_TypeDef *port, uint8_t pin) {
    uint32_t mask = (1U << pin);
    port->BRR = mask;            // atomic reset
    // alternative: port->BSRR = mask << 16;
}

static inline void gpio_toggle(GPIO_TypeDef *port, uint8_t pin) {
    uint32_t mask = (1U << pin);
    port->ODR ^= mask;           // not atomic; protect if concurrent access possible
}

static inline void gpio_write(GPIO_TypeDef *port, uint8_t pin, uint8_t state) {
    if (state == GPIO_STATE_SET) {
        gpio_set(port, pin);
    } else if (state == GPIO_STATE_RESET) {
        gpio_reset(port, pin);
    } else {
        gpio_toggle(port, pin);
    }
}

/* read input data register and return normalized bit (0 or 1) */
static inline uint8_t gpio_read(GPIO_TypeDef *port, uint8_t pin) {
    uint32_t mask = (1U << pin);
    return ( (port->IDR & mask) ? 1U : 0U );
}

void GPIO_Init_CMSIS(void);

#endif /* GPIO_CMSIS_H_ */
