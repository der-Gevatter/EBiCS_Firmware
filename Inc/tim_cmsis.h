/*
 * tim_cmsis.h
 *
 *  Created on: 19.04.2026
 *      Author: stephan
 */

#ifndef TIM_CMSIS_H_
#define TIM_CMSIS_H_

#include "stm32f103x6.h"

/* Public API */
/* TIM1 */
void TIM1_CMSIS_Init(void);                // initialize TIM1 (register-level)
void tim1_start_pwm(void);                 // enable TIM1 counter and start PWM
void tim1_stop_pwm(void);                  // stop TIM1 counter
void tim1_set_ccr(uint8_t channel, uint32_t value); // set CCR1..CCR4

/* TIM2 */
void TIM2_CMSIS_Init(void);
void TIM2_CMSIS_Start(void);
void TIM2_CMSIS_Stop(void);

void TIM2_Handler_Impl(void);

/* Weak callback to be implemented by application */
void TIM2_CaptureCallback(uint8_t channel, uint32_t ccr_value);

/* TIM3 */
void TIM3_CMSIS_Init(void);
void TIM3_CMSIS_Start(void);
void TIM3_CMSIS_Stop(void);

void TIM3_Handler_Impl(void);

/* Weak callback to be implemented by application */
void TIM3_UpdateCallback(void);
void TIM3_CC1Callback(void);

#endif /* TIM_CMSIS_H_ */
