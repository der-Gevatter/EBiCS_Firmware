/*
 * uart_cmsis.h
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#ifndef UART_CMSIS_H_
#define UART_CMSIS_H_

#include <stdio.h>
typedef void (*uart_tx_cb_t)(void);

static volatile uart_tx_cb_t uart1_tx_done_cb = 0;
static volatile uart_tx_cb_t uart1_tx_error_cb = 0;

//void UART1_Init(uint32_t baud);
void USART1_Init_CMSIS(void);
void USART1_Config(uint32_t baudrate, uint32_t pclk2_hz);
void USART1_Start_RX_DMA_Circular(uint8_t *pData, uint16_t Size);
void USART1_Start_TX_DMA_NonBlocking(uint8_t *buf, uint16_t len, uart_tx_cb_t done_cb, uart_tx_cb_t err_cb);
void DMA1_Channel4_IRQHandler(void);

#endif /* UART_CMSIS_H_ */
