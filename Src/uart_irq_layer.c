/*
 * uart_irq_layer.c
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#include "uart_irq_layer.h"
#include "uart_cmsis.h"
#include "stm32f103x6.h"
#include "systick.h"

/* user-provided callbacks */
static uart_tx_cb_t tx_done_handler = NULL;
static uart_tx_cb_t tx_err_handler  = NULL;

void uart_irq_layer_init(void)
{
    /* assume MX_USART1_MspInit_CMSIS() already did NVIC enable + priorities */
    /* clear any stray DMA flags */
    DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;
}

/* TX start/stop implementations; follow safe sequence from earlier discussion */
void uart_tx_start_dma(uint8_t *buf, uint16_t len, uart_tx_cb_t done_cb, uart_tx_cb_t err_cb)
{
    tx_done_handler = done_cb;
    tx_err_handler  = err_cb;

    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    (void)DMA1_Channel4->CCR;
    DMA1->IFCR = DMA_IFCR_CGIF4;

    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;
    DMA1_Channel4->CMAR = (uint32_t)buf;
    DMA1_Channel4->CNDTR = len;

    DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_TEIE;
    USART1->CR3 |= USART_CR3_DMAT;
    DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void uart_tx_stop_dma(void)
{
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    (void)DMA1_Channel4->CCR;
    DMA1->IFCR = DMA_IFCR_CGIF4;
    USART1->CR3 &= ~USART_CR3_DMAT;
}

/* simple blocking UART TX using polling (TXE + TC). */
/* Caller must ensure USART1 clock + pins configured. This is independent from DMA. */
void uart_tx_blocking(const uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint16_t i;
    uint32_t start = systick_ms(); // implement or replace with your ms tick source

    for (i = 0; i < len; ++i) {
        // wait for TXE (transmit data register empty)
        while (!(USART1->SR & USART_SR_TXE)) {
            if (timeout_ms && ((systick_ms() - start) > timeout_ms)) return; // timeout
        }
        USART1->DR = (uint8_t)buf[i]; // English comment: write byte to data register
    }

    // wait for TC (transmission complete) to ensure last byte sent out
    while (!(USART1->SR & USART_SR_TC)) {
        if (timeout_ms && ((systick_ms() - start) > timeout_ms)) return;
    }
    // clear TC by reading SR and writing DR (or by writing 0 to TC if needed)
    (void)USART1->SR;
    (void)USART1->DR;
}

/* start circular RX */
void uart_rx_start_circular(uint8_t *buf, uint16_t buffer_size)
{
    /* disable channel */
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    (void)DMA1_Channel5->CCR;
    DMA1->IFCR = DMA_IFCR_CGIF5;

    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;
    DMA1_Channel5->CMAR = (uint32_t)buf;
    DMA1_Channel5->CNDTR = buffer_size;

    /* peripheral to memory, circular, meminc, TC/HT/TE IRQs optional */
    DMA1_Channel5->CCR = /* DIR=0 */ DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TEIE | DMA_CCR_HTIE | DMA_CCR_TCIE;

    USART1->CR3 |= USART_CR3_DMAR;
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

uint16_t uart_rx_get_head(uint16_t buffer_size)
{
    uint16_t remaining = (uint16_t)DMA1_Channel5->CNDTR;
    return (uint16_t)((buffer_size - remaining) % buffer_size);
}

/* Channel4 IRQ handling - call user callbacks from here */
void DMA1_Channel4_IRQHandler(void)
{
    uint32_t isr = DMA1->ISR;

    if (isr & DMA_ISR_TCIF4) {
        DMA1->IFCR = DMA_IFCR_CTCIF4;
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;
        (void)DMA1_Channel4->CCR;
        USART1->CR3 &= ~USART_CR3_DMAT;
        if (tx_done_handler) tx_done_handler();
    }
    if (isr & DMA_ISR_TEIF4) {
        DMA1->IFCR = DMA_IFCR_CTEIF4;
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;
        (void)DMA1_Channel4->CCR;
        USART1->CR3 &= ~USART_CR3_DMAT;
        if (tx_err_handler) tx_err_handler();
    }
}

/* Channel5 IRQ handling (HT/TC/TE clears) */
void DMA1_Channel5_IRQHandler(void)
{
    uint32_t isr = DMA1->ISR;
    if (isr & DMA_ISR_TEIF5) { DMA1->IFCR = DMA_IFCR_CTEIF5; /* recovery */ }
    if (isr & DMA_ISR_HTIF5) { DMA1->IFCR = DMA_IFCR_CHTIF5; /* optional hook */ }
    if (isr & DMA_ISR_TCIF5) { DMA1->IFCR = DMA_IFCR_CTCIF5; /* circular: clear and continue */ }
}
