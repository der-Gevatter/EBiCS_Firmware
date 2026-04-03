/*
 * uart_irq_layer.h
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#ifndef UART_IRQ_LAYER_H_
#define UART_IRQ_LAYER_H_

#include <stdint.h>

/* callback type for TX done/error */
typedef void (*uart_tx_cb_t)(void);

/* Initialize IRQ layer (assumes MSP + USART config already done) */
void uart_irq_layer_init(void);

/* Start non-blocking TX via DMA (DMA1 Channel4 on USART1) */
void uart_tx_start_dma(uint8_t *buf, uint16_t len, uart_tx_cb_t done_cb, uart_tx_cb_t err_cb);

/* Stop TX DMA safely */
void uart_tx_stop_dma(void);

/* simple blocking UART TX using polling (TXE + TC) */
void uart_tx_blocking(const uint8_t *buf, uint16_t len, uint32_t timeout_ms);

/* Start circular RX DMA into provided buffer (DMA1 Channel5 on USART1).
   buffer_size must be >0. */
void uart_rx_start_circular(uint8_t *buf, uint16_t buffer_size);

/* Return current RX head index (number of bytes received) in circular buffer */
uint16_t uart_rx_get_head(uint16_t buffer_size);

#endif /* UART_IRQ_LAYER_H_ */
