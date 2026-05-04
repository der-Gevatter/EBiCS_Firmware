/*
 * tim1_irq_layer.h
 *
 *  Created on: 19.04.2026
 *      Author: stephan
 */

#ifndef TIM_IRQ_LAYER_H_
#define TIM_IRQ_LAYER_H_

#include <stm32f103x6.h>

/* Export IRQ handler prototypes used in vector table */
void TIM1_UP_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);

#endif /* TIM_IRQ_LAYER_H_ */
