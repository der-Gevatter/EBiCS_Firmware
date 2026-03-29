/*
 * init.c
 *
 *  Created on: 26.03.2026
 *      Author: StK
 */

#include "init.h"
#include "stm32f103x6.h"
#include "main.h"


void SystemClock_Config_CMSIS(void)
{
    /* 1) Enable HSI and wait ready */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) {}

    /* 2) Configure PLL: source = HSI/2, PLLMUL = x16
       PLLSRC bit = 0 selects HSI/2; PLLMUL bits: PLLMUL[3:0] = 1110 => x16 */
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    /* PLLSRC = 0 (HSI/2) implicit; set PLLMULL = 14 (x16) */
    RCC->CFGR |= (RCC_CFGR_PLLMULL16);

    /* 3) Enable PLL and wait ready */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* 4) Configure Flash latency and enable Prefetch buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2; /* 2 wait states for 64MHz */

    /* 5) Select PLL as system clock */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

    /* 6) Set AHB, APB1, APB2 prescalers
       AHB = SYSCLK /1  (HPRE = 0)
       APB2 = HCLK /1  (PPRE2 = 0)
       APB1 = HCLK /2  (PPRE1 = 4 => DIV2)
    */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; /* APB1 = HCLK/2 */
    /* APB2 default DIV1, AHB default DIV1 */

    /* 7) Configure ADC prescaler: ADCCLK = PCLK2 / 6
       In RM0008 ADC prescaler bits ADCPRE[1:0] in RCC_CFGR2 for some series,
       but for STM32F1 standard peripheral lib uses RCC->CFGR & ADC prescaler in RCC->CFGR?
       For STM32F1: ADC prescaler is in RCC->CFGR (ADCPRE bits) as RCC_CFGR_ADCPRE_x.
    */
    /* Clear ADCPRE bits then set to /6 (bits encoding: 10 -> /6) */
    RCC->CFGR &= ~RCC_CFGR_ADCPRE;
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;

    /* 8) Enable peripheral clocks if needed elsewhere (Systick uses HCLK) */
    /* Systick: configure to 1ms tick */
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U; /* SystemCoreClock should be 64MHz */
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Set SysTick priority */
    NVIC_SetPriority(SysTick_IRQn, 0);
}


void MX_ADC1_Init_CMSIS(uint16_t offset)
{
    // Erwartet: RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; und GPIOs bereits als analog konfiguriert
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    // 1) Deactivate ADC (clear ADON) to allow configuration/calibration
	ADC1->CR2 &= ~ADC_CR2_ADON;
	for (volatile int i=0;i<100;++i) __NOP();

    // 2) Reset calibration and calibrate
	ADC1->CR2 |= ADC_CR2_RSTCAL;
	uint32_t t=0;
	while (ADC1->CR2 & ADC_CR2_RSTCAL) { if (++t>100000) break; }

	ADC1->CR2 |= ADC_CR2_CAL;
	t=0;
	while (ADC1->CR2 & ADC_CR2_CAL) { if (++t>100000) break; }


    // 3) Mode: scan enabled, continuous disabled, right alignment
    ADC1->CR1 |= ADC_CR1_SCAN;      // ScanConvMode = ENABLE
    ADC1->CR2 &= ~ADC_CR2_CONT;     // ContinuousConvMode = DISABLE
    ADC1->CR2 &= ~ADC_CR2_ALIGN;    // DataAlign = RIGHT (0)

    // 4) External trigger for regular conversions: TIM3 TRGO -> EXTSEL = 4 (CR2[24:20])
    //    Enable external trigger (EXTTRIG bit)
    ADC1->CR2 &= ~ADC_CR2_EXTSEL;
    ADC1->CR2 |= (4UL << 20);       // EXTSEL = 4 -> TIM3 TRGO (RM0008 mapping)
    ADC1->CR2 |= ADC_CR2_EXTTRIG;

    // 5) Number of regular conversions: 8 -> L = 7 (SQR1[L 23:20])
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR1 |= (7UL << 20);      // 8 conversions -> L = 7

    // 6) Regular sequence ranks (target):
    //    Rank1 = CH7, Rank2 = CH3, Rank3 = CH4, Rank4 = CH5,
    //    Rank5 = CH6, Rank6 = CH8, Rank7 = CH9, Rank8 = IN16 (temp)
    ADC1->SQR3 = 0;
    ADC1->SQR3 |= (1UL  << 0);   // SQ1 = CH7
    ADC1->SQR3 |= (4UL  << 5);   // SQ2 = CH3
    ADC1->SQR3 |= (0UL  << 10);  // SQ3 = CH4
    ADC1->SQR3 |= (11UL  << 15);  // SQ4 = CH5
    ADC1->SQR3 |= (10UL  << 20);  // SQ5 = CH6
    ADC1->SQR3 |= (14UL  << 25);  // SQ6 = CH8

    ADC1->SQR2 = 0;
    ADC1->SQR2 |= (7UL  << 0);   // SQ7 = CH9
    ADC1->SQR2 |= (16UL << 5);   // SQ8 = IN16 (temperature sensor)

    // 7) Sampling times: use shortest (1.5 cycles) => reset SMPR registers (0 => 1.5 cycles)
    ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 |
                     ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 |
                     ADC_SMPR2_SMP6 | ADC_SMPR2_SMP7 | ADC_SMPR2_SMP8 |
                     ADC_SMPR2_SMP9);
    ADC1->SMPR1 &= ~(ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP12 |
                     ADC_SMPR1_SMP13 | ADC_SMPR1_SMP14 | ADC_SMPR1_SMP15 |
                     ADC_SMPR1_SMP16 | ADC_SMPR1_SMP17);
    //**// If you want specific channels different sampling, set the corresponding SMPx fields here.

    // 8) Injected channel configuration:
    // Stop ADC (ensure ADON cleared) before changing injected trigger/source
    ADC1->CR2 &= ~ADC_CR2_ADON;

    // Injected sequence length JL = 0 (1 conversion)
    ADC1->JSQR &= ~ADC_JSQR_JL;

    // JSQ1 = channel 4
    ADC1->JSQR &= ~ADC_JSQR_JSQ1;
    ADC1->JSQR |= (0UL << 0);    // JSQ1 = IN4

    // Injected external trigger source: TIM1 CC4 -> JEXTSEL = mapping value
    // RM0008 mapping for JEXTSEL (CR2[17:15]):
    // 000 TIM1_CC1, 001 TIM1_CC2, 010 TIM1_CC3, 011 TIM2_CC2,
    // 100 TIM3_TRGO, 101 TIM2_TRGO, 110 TIM3_CC4, 111 TIM1_TRGO
    // TIM1_CC4 is not explicitly listed in the 3-bit table (but TIM1_CC4 appears as EXTSEL=8 for regular).
    // For injected events common practice: use TIM1 CC4 via JEXTSEL=0b000..? -> Many apps use TIM1_CC4 mapped as value '0'.. check RM.
    // Safer choice for TIM1 CC4 is to use TIM1 TRGO or TIM3 CC4 depending on actual need.
    // Here we'll set JEXTSEL = TIM1_CC4 using the regular mapping value 8 -> that requires writing 3-bit field with value 0..7 only.
    // TIM1_CC4 isn't in the 3-bit table; instead use TIM1_TRGO (7) or TIM3_CC4 (6) if you intend CC4 pulse.
    // If you specifically require TIM1 CC4 as injected source, set the appropriate timer to generate TRGO on CC4 and use JEXTSEL accordingly.
    // For this code we use TIM1_TRGO (7) as injected trigger example:

    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL);
    ADC1->CR2 |= (7UL << 15);   // JEXTSEL = 7 -> TIM1_TRGO (choose if TIM1 TRGO configured to reflect CC4)
    ADC1->CR2 |= ADC_CR2_JEXTTRIG; // enable injected external trigger

    // Disable auto-injected (JAUTO) to match HAL: JAUTO = 0
    ADC1->CR1 &= ~ADC_CR1_JAUTO;

    // Injected sampling time for channel 4 already set via SMPR2 (channel 4 -> SMP4 bits), we left SMP fields = 0 (1.5 cycles)
    // Set injected offset for injected rank1 in JOFR1
    ADC1->JOFR1 = (offset & 0x0FFF);

    // 9) Re-enable ADC
    ADC1->CR2 |= ADC_CR2_ADON;

    // Small delay to allow ADC stabilization before first conversion (optional)
    for (volatile int i = 0; i < 2000; ++i) __NOP();

    // End of init
}

void ADC1_DMA_TIM3_Init_Start(volatile uint16_t* adcData)
{
    // 0) Enable clocks: AFIO optional, GPIOs assumed konfiguriert, hier ADC1, DMA1, TIM3
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;    // DMA1 clock (STM32F1 line)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;   // TIM3 clock

    // Set APB1 timer clock assumed 64 MHz already.
    // 1) Configure TIM3 for 8 kHz TRGO (update event).
    TIM3->PSC = 7;           			  // Prescaler -> divide by (PSC+1)=8 => 8 MHz timer clock
    TIM3->ARR = 999;        			  // Auto-reload -> (ARR+1)=1000 => update = 8MHz/1000 = 8kHz
    // Configure TRGO: MMS = 010 -> Update as TRGO
    TIM3->CR2 &= ~TIM_CR2_MMS;
    TIM3->CR2 |= (0x2 << 4);
    TIM3->EGR = TIM_EGR_UG;  			  // generate update to load prescaler
    TIM3->CR1 |= TIM_CR1_CEN;			  // start timer

    // 2) Ensure ADC disabled for config/calib (your init already did calibration)
    ADC1->CR2 &= ~ADC_CR2_ADON;

    // 3) Ensure ADC DMA is enabled
    ADC1->CR2 |= ADC_CR2_DMA;

    // 4) Configure DMA1 Channel1 for ADC1 -> Memory (adcData)
    // Disable channel first
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    while (DMA1_Channel1->CCR & DMA_CCR_EN) {}

    // Peripheral address = ADC1->DR
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    // Memory address
    DMA1_Channel1->CMAR = (uint32_t) adcData;
    // Number of data
    DMA1_Channel1->CNDTR = 8;

    // Configure CCR:
    // - Memory increment
    // - Peripheral size = 16-bit (ADC data 12-bit -> store in 16-bit)
    // - Memory size = 16-bit
    // - Circular mode (optional) -> enable if continuous acquisition desired
    // - Priority high
    // - Direction peripheral-to-memory (DIR = 0)
    DMA1_Channel1->CCR = 0;
    DMA1_Channel1->CCR |= DMA_CCR_MINC;          // Memory increment
    // PSIZE = 01 (16-bit) -> bits [9:8] = 01
    DMA1_Channel1->CCR |= (1U << 8);
    // MSIZE = 01 (16-bit) -> bits [11:10] = 01
    DMA1_Channel1->CCR |= (1U << 10);
    // Circular mode (enable for continuous repeated transfers)
    DMA1_Channel1->CCR |= DMA_CCR_CIRC;
    // Priority level: very high (PL = 11 -> bits [13:12])
    DMA1_Channel1->CCR |= (3U << 12);
    // Option: enable transfer complete interrupt:
    // DMA1_Channel1->CCR |= DMA_CCR1_TCIE;
    // Configure/clear flags in DMA1->IFCR if needed
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1; // clear all (implementation dependent)

    // 5) Enable DMA channel
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // 6) Make sure ADC regular external trigger is set to TIM3 TRGO and EXTTRIG is enabled
    //    You already set EXTSEL=4 (TIM3 TRGO) and EXTTRIG in your MX_ADC1_Init.
    //    If not, set here:
    ADC1->CR2 &= ~ADC_CR2_EXTSEL;
    ADC1->CR2 |= (4UL << 20);   // EXTSEL = 4 -> TIM3 TRGO (RM0008 mapping)
    ADC1->CR2 |= ADC_CR2_EXTTRIG;

    // 7) Enable ADC (if not already)
    ADC1->CR2 |= ADC_CR2_ADON;
    //delay_cycles(2000);

    // 8) Start TIM3 counting to generate TRGO (if not started yet)
    // TIM3 already enabled above; if you prefer start now:
    // TIM3->CR1 |= TIM_CR1_CEN;

    // From now on: each TIM3 TRGO will trigger ADC regular conversion sequence.
    // DMA in circular mode schreibt kontinuierlich in adcData[0..7].
}

// Callback-Protos
void HAL_ADC_ConvCpltCallback(void);
void HAL_ADCEx_InjectedConvCpltCallback(void);

// IRQ-Handler
/*void ADC1_IRQHandler(void)
{
    uint32_t sr = ADC1->SR;

    // Regular end of conversion (EOC)
    if (sr & ADC_SR_EOC) {
        uint16_t regular_val = ADC1->DR & 0x0FFF; // Lesen löscht EOC
        // optional: speichere Wert in Buffer, setze Flag etc.
        HAL_ADC_ConvCpltCallback();
    }

    // Injected end of conversion (JEOC)
    if (sr & ADC_SR_JEOC) {
        uint16_t inj_val = ADC1->JDR1 & 0x0FFF; // Lesen löscht JEOC
        // setze/verwende inj_val (z.B. global)
        HAL_ADCEx_InjectedConvCpltCallback();
    }

}*/
