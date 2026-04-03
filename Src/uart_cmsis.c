/*
 * uart_cmsis.c
 *
 *  Created on: 02.04.2026
 *      Author: stephan
 */

#include "uart_cmsis.h"
#include "stm32f103x6.h"

void MX_USART1_MspInit_CMSIS(void)
//void UART1_Init(uint32_t baud)
{

    /* 1) Enable clocks: GPIOB, USART1, AFIO, DMA1 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    (void)RCC->APB2ENR;
    (void)RCC->AHBENR;

    /* 2) Remap USART1: set USART1_REMAP bit in AFIO->MAPR (same as __HAL_AFIO_REMAP_USART1_ENABLE) */
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

    /* 3) Configure PB6 = TX (AF push-pull), PB7 = RX (input floating) */
    GPIOB->CRL &= ~((0xF << (6*4)) | (0xF << (7*4))); /* clear pins 6/7 */
    GPIOB->CRL |=  (0xB << (6*4)); /* MODE6=11(50MHz), CNF6=10(AF PP) */
    GPIOB->CRL |=  (0x4 << (7*4)); /* MODE7=00, CNF7=01(Input floating) */

    /* 4) USART1 basic enable (caller must configure baud/CR registers elsewhere)
       Ensure USART1 not enabled here if already configured later.
       We only ensure peripheral clock above.
    */

    /* 5) DMA configuration (reset CCRs first) */
    /* Disable channels before config */
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    /* Clear channel configuration registers */
    DMA1_Channel4->CCR = 0;
    DMA1_Channel5->CCR = 0;

    /* TX: DMA1 Channel4
       - Memory to peripheral (DIR = 1)
       - Peripheral increment disabled (PINC=0)
       - Memory increment enabled (MINC=1)
       - Peripheral size = 8-bit (PSIZE=00)
       - Memory size = 8-bit (MSIZE=00)
       - Normal mode (CIRC=0)
       - Priority low (PL=00)
       - No circular, no mem-to-mem
    */
    DMA1_Channel4->CCR = (1U << 4)   /* DIR: 1 = memory to peripheral (bit4) */
                       | (0U << 6)   /* PINC: 0 */
                       | (1U << 7)   /* MINC: 1 */
                       | (0U << 8)   /* PSIZE: 00 = 8-bit */
                       | (0U << 10)  /* MSIZE: 00 = 8-bit */
                       | (0U << 5)   /* CIRC: 0 */
                       | (0U << 14); /* PL: 00 = low priority (bits 13:14) */

    /* Peripheral address for TX: USART1->DR */
    DMA1_Channel4->CPAR = (uint32_t)&(USART1->DR);
    /* CMAR and CNDTR set when starting transfer */

    /* RX: DMA1 Channel5
       - Peripheral to memory (DIR = 0)
       - PINC = 0
       - MINC = 1
       - PSIZE = 8-bit, MSIZE = 8-bit
       - Circular mode (CIRC = 1)
       - Priority low (PL = 00)
    */
    DMA1_Channel5->CCR = (0U << 4)   /* DIR: 0 = peripheral to memory */
                       | (0U << 6)   /* PINC: 0 */
                       | (1U << 7)   /* MINC: 1 */
                       | (0U << 8)   /* PSIZE: 00 */
                       | (0U << 10)  /* MSIZE: 00 */
                       | (1U << 5)   /* CIRC: 1 */
                       | (0U << 14); /* PL: 00 */

    DMA1_Channel5->CPAR = (uint32_t)&(USART1->DR);
    /* CMAR and CNDTR set when enabling RX DMA transfer */

    /* Clear any pending DMA interrupt flags for channel4/5 */
    DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;

    /* 6) NVIC: set priorities and enable IRQs
       CMSIS NVIC_SetPriority uses single numeric priority. Map HAL (pre/sub) to one value:
       USART1: (0,0) -> priority 0
       DMA1_Channel5: (1,0) -> priority 1
       DMA1_Channel4: (3,1) -> map to priority 3 (or 3)
    */
    NVIC_SetPriority(USART1_IRQn, 0);
    NVIC_EnableIRQ(USART1_IRQn);

    NVIC_SetPriority(DMA1_Channel5_IRQn, 1);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    NVIC_SetPriority(DMA1_Channel4_IRQn, 3);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

/* ----------------- USART configuration (baud, enable, DMA requests) ----------------- */
/* Call this after MX_USART1_DMA_Init_CMSIS() */
void USART1_Config(uint32_t baudrate, uint32_t pclk2_hz)
{
    /* Disable USART before config */
    USART1->CR1 &= ~USART_CR1_UE;

    /* Configure baudrate: BRR = pclk / baud for oversampling by 16 */
    uint32_t uartdiv = (pclk2_hz + (baudrate/2)) / baudrate;
    USART1->BRR = uartdiv;

    /* CR1: 8 data bits, no parity (M=0, PCE=0), RXNE interrupt/flags left to user.
       Enable TE and RE after CR1 set. */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE;

    /* CR2/CR3 defaults: ensure no LIN, STOP bits default (1 stop) */
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    /* Enable DMA requests for TX and RX if using DMA */
    USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;

    /* Enable USART */
    USART1->CR1 |= USART_CR1_UE;

    /* Small readback delay */
    (void)USART1->SR;
    (void)USART1->DR;
}
