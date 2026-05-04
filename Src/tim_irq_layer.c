/*
 * tim_irq_layer.c - Central IRQ layer that forwards to individual timer handlers
 *
 *  Created on: 19.04.2026
 *      Author: stephan
 */

#include "tim_irq_layer.h"
#include "tim_cmsis.h" /* declares TIMx handler prototype */
#include "main.h"
#include "stm32f1xx_hal.h"

/* Forwarders used in vector table */

/* TIM2 forwarder: direct call to implementation in tim2_cmsis.c */

void TIM2_IRQHandler(void)
{
#ifdef USE_CMSIS
	TIM2_Handler_Impl();
#else
    /* Minimal TIM handle populated so HAL can operate on it */
    TIM_HandleTypeDef htim_tmp;

    /* Set instance to hardware TIM2 */
    /* This is required by HAL_TIM_IRQHandler() */
    htim_tmp.Instance = TIM2;

    /* Optionally set a sensible state so HAL won't bail out (READY) */
    htim_tmp.State = HAL_TIM_STATE_READY;

	//TIM2_Handler_Impl();
	HAL_TIM_IRQHandler(&htim_tmp);
#endif
}

/* TIM3 placeholder forwarder */
void TIM3_IRQHandler(void)
{
	TIM3_Handler_Impl();
}
