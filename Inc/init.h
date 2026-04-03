/*
 * init.h
 *
 *  Created on: 26.03.2026
 *      Author: StK
 */

#ifndef INIT_H_
#define INIT_H_

#include <stdio.h>

typedef void (*uart_tx_cb_t)(void);

static volatile uart_tx_cb_t uart1_tx_done_cb = 0;
static volatile uart_tx_cb_t uart1_tx_error_cb = 0;

/* External variables ---------------------------------------------------------*/
//extern ADC_HandleTypeDef hadc1;
//extern ADC_HandleTypeDef hadc2;

//extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim3;

//extern UART_HandleTypeDef huart1;

//extern IWDG_HandleTypeDef hiwdg;

/* Function prototypes -----------------------------------------------*/

//void MX_TIM1_Init(void);
//void MX_TIM2_Init(void);
//void MX_TIM3_Init(void);

void MX_ADC1_Init_CMSIS(uint16_t offset);
void ADC1_DMA_TIM3_Init_Start(volatile uint16_t * adcData);
//void MX_ADC2_Init(void);

//void MX_USART1_UART_Init(void);


//void MX_DMA_Init(void);

//void MX_IWDG_Init(void);
//void init_watchdog(void);

#endif /* INIT_H_ */
