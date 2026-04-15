/*
 * adc_cmsis.c
 *
 *  Created on: 10.04.2026
 *      Author: stephan
 */

#include "adc_cmsis.h"
#include "stm32f103x6.h"
#include "gpio_cmsis.h"
#include "main.h"

/* Helper: enable ADC clocks (APB2ENR ADC1/ADC2 share APB2ENR ADC1EN bit on F1) */
static void adc_enable_clocks(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // ADC1 + ADC2 share enable
    (void)RCC->APB2ENR;
    for (volatile int i = 0; i < 1000; ++i) __asm__("nop");
}

/* Helper: Calibrate ADCx (call after ADON wake sequence) */
static void adc_calibrate(ADC_TypeDef *ADCx)
{
    /* 1. Wake up ADC from power-down mode */
    ADCx->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 1000; i++); // small delay

    /* 2. Reset calibration registers */
    ADCx->CR2 |= ADC_CR2_RSTCAL;
    while (ADCx->CR2 & ADC_CR2_RSTCAL); // wait for reset complete

    /* 3. Start actual calibration */
    ADCx->CR2 |= ADC_CR2_CAL;
    /* Wait for CAL bit cleared by hardware */
    while (ADCx->CR2 & ADC_CR2_CAL);
}

/* Helper: Configure sample time for a specific channel (channel 0..17) */
static void adc_set_sample_time(ADC_TypeDef *ADCx, uint8_t channel, uint32_t sample_bits)
{
    /* sample_bits is the 3-bit encoding (e.g., 0=1.5, 7=239.5). We'll pass as literal macro values similar to CMSIS header. */
    if (channel <= 9) {
        uint32_t pos = (channel) * 3;
        ADCx->SMPR2 &= ~(0x7UL << pos);
        ADCx->SMPR2 |= (sample_bits & 0x7UL) << pos;
    } else {
        uint32_t ch = channel - 10;
        uint32_t pos = ch * 3;
        ADCx->SMPR1 &= ~(0x7UL << pos);
        ADCx->SMPR1 |= (sample_bits & 0x7UL) << pos;
    }
}

/* Configure regular SQRx and sample times */
static void ADC1_ConfigRegularSequence(void)
{
    /* Set number of regular conversions: N-1 in SQR1[23:20] */
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR1 |= ((8 - 1) << ADC_SQR1_L_Pos); // 8 conversions

    /* Map channels ranks 1..8: ch1, ch4, ch0, ch11, ch10, ch14, ch5, tempsensor */
    /* Sample times: SMPR1/SMPR2 */
    ADC1->SQR3 = 0;	// clear

    ADC1->SQR3 |= (1  << ADC_SQR3_SQ1_Pos);  // rank1 -> CH1	->	battery voltage
    adc_set_sample_time(ADC1, 1, ADC_SAMPLETIME_1_5);

    ADC1->SQR3 |= (4  << ADC_SQR3_SQ2_Pos);  // rank2 -> CH4	->	Connector SP: throttle input
    adc_set_sample_time(ADC1, 4, ADC_SAMPLETIME_1_5);

    ADC1->SQR3 |= (0  << ADC_SQR3_SQ3_Pos);  // rank3 -> CH0	->	Phase current 1
    adc_set_sample_time(ADC1, 0, ADC_SAMPLETIME_1_5);

    ADC1->SQR3 |= (11 << ADC_SQR3_SQ4_Pos);  // rank4 -> CH11	->	Phase current 2
    adc_set_sample_time(ADC1, 11, ADC_SAMPLETIME_1_5);

    ADC1->SQR3 |= (10 << ADC_SQR3_SQ5_Pos);  // rank5 -> CH10	->	Phase current 3
    adc_set_sample_time(ADC1, 10, ADC_SAMPLETIME_1_5);

    ADC1->SQR3 |= (14 << ADC_SQR3_SQ6_Pos);  // rank6 -> CH14	->	connector AD2 (torque)
    adc_set_sample_time(ADC1, 14, ADC_SAMPLETIME_1_5);

    ADC1->SQR2 = 0;	// clear

    ADC1->SQR2 |= (5  << ADC_SQR2_SQ7_Pos);  // rank7 -> CH5	->	connector AD1, temperature or torque input for Controller from PhoebeLiu @ aliexpress
    adc_set_sample_time(ADC1, 5, ADC_SAMPLETIME_239_5);

    ADC1->SQR2 |= (16 << ADC_SQR2_SQ8_Pos); // rank8 -> CH16	->	temp sensor (mapped on CH16)
    adc_set_sample_time(ADC1, 16, ADC_SAMPLETIME_239_5);
}

/* Configure injected channel (Injected channel 0 -> injected rank1) */
static void ADC1_ConfigInjected(void)
{
    /* Disable injected conversions while configuring */
    ADC1->CR1 &= ~ADC_CR1_JAUTO;
    ADC1->JSQR = 0;
    /* Injected sequence length: JL bits in JSQR (JL = injected conversions -1) */
    ADC1->JSQR &= ~ADC_JSQR_JL;
    ADC1->JSQR |= ( (1 - 1) << ADC_JSQR_JL_Pos ); // 1 injected conversion

    /* Set injected channel in JSQR: JEXTSEL/JEXTTRIG config for injected trigger */
    ADC1->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTTRIG);
    ADC1->CR2 |= (0b001 << ADC_CR2_JEXTSEL_Pos); // TIM1_CC4 trigger
    ADC1->CR2 |= ADC_CR2_JEXTTRIG; // enable trigger

    /* JSQR: injected sequence channels are set in JSQR bits */
    ADC1->JSQR &= ~ADC_JSQR_JSQ4; // clear
    ADC1->JSQR |= (0 << ADC_JSQR_JSQ4_Pos); // Injected channel 0

    /* Sampling time for injected channel: set very small or larger */
    /* set SMPR2 for channel0 to 1.5 cycles (0) */
    adc_set_sample_time(ADC1, 0, ADC_SAMPLETIME_1_5);

    /* set offset to 0 - will be dynamically changed later */
    ADC1->JOFR1 = 0;
}

/* Initialize ADC1 (regular + injected config) */
void ADC1_CMSIS_Init(void)
{
    /* Turn off ADC and reset config */
    ADC1->CR2 = 0;
    ADC1->CR1 = 0;

    /* 1) Enable clocks: ADC1, ADC2 on APB2; DMA1 on AHB */
    adc_enable_clocks();
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* 2) Configure ADC common register set Dual Injected Simultaneous */
    /* ADC1 = Master */
    ADC1->CR1 &= ~ADC_CR1_DUALMOD;
    /* INJECSIMULT */
    ADC1->CR1 |= (0b101 << ADC_CR1_DUALMOD_Pos);

    /* 3) configure gpios analog */
    gpio_config_analog_pin(GPIOA, Throttle_Pin);			// throttle
    gpio_config_analog_pin(GPIOA, Phase_Current1_Pin);		// phase_current_1
    gpio_config_analog_pin(GPIOA, GPIO_PIN_1);				// battery voltage
    gpio_config_analog_pin(GPIOA, GPIO_PIN_5);				// AD1 - MotorTemp
    gpio_config_analog_pin(GPIOC, Temperature_Pin);			// temperature
    gpio_config_analog_pin(GPIOC, Phase_Current_2_Pin);		// phase_current_2
    gpio_config_analog_pin(GPIOC, Phase_Current_3_Pin);		// phase_current_3
    gpio_config_analog_pin(GPIOC, GPIO_PIN_4);				// AD2 - torque

    /* 4) ADC1 basic config: disable scan in CR1/CR2 then set right values */
    ADC1->CR1 |= ADC_CR1_SCAN;	  // we will set sequence length in SQR1
    ADC1->CR2 = 0;                // reset CR2
    ADC1->CR2 |= ADC_CR2_EXTTRIG; // enable external trigger for regular conversions
    ADC1->CR2 &= ~ADC_CR2_EXTSEL; // we'll use TIM3 TRGO: set bits below
    /* Set EXTSEL for TIM3 TRGO (EXTSEL = 0b0100 for TIM3 TRGO on F1) */
    ADC1->CR2 |= (4 << ADC_CR2_EXTSEL_Pos);
    ADC1->CR2 &= ~ADC_CR2_ALIGN;  // right align

    /* enable TempSensor */
    ADC1->CR2 |= ADC_CR2_TSVREFE;

    /* 5) Configure regular channel sequence and sample times */
    ADC1_ConfigRegularSequence();

    /* 6) Configure injected channel */
    ADC1_ConfigInjected();

    /* 7) Wake ADON sequence + calibrate */
    adc_calibrate(ADC1);

    /* Enable JEOC interrupt (we handle IRQ in irq layer) */
    ADC1->CR1 |= ADC_CR1_JEOCIE; // enable interrupt at end of injected conversion

    /* NVIC for ADC1_2 IRQ */
    NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
    NVIC_EnableIRQ(ADC1_2_IRQn);

    /* Ensure ADC is enabled for conversions */
    ADC1->CR2 |= ADC_CR2_ADON;
}

void ADC1_DMA_Init_Circular(volatile uint32_t* adcData, uint16_t len)
{
    /* enable Clocks */
	adc_enable_clocks();
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* Use DMA1 Channel1 registers */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN; // disable before config

    /* Peripheral address: ADC1 data register */
    DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
    /* Memory address: pointer to adcData buffer */
    DMA1_Channel1->CMAR = (uint32_t)adcData;
    /* Number of data items (halfwords) */
    DMA1_Channel1->CNDTR = len;

    /* CCR: configure
       - memory increment
       - peripheral size = halfword (01)
       - memory size = halfword (01)
       - circular mode
       - peripheral-to-memory (DIR = 0)
       - priority medium (1)
       - transfer complete interrupts
    */
    DMA1_Channel1->CCR = 0;
    DMA1_Channel1->CCR |= DMA_CCR_MINC;                        // memory increment
    /* set peripheral & memory data size to halfword (01) */
    DMA1_Channel1->CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE);
    DMA1_Channel1->CCR |= (1 << DMA_CCR_MSIZE_Pos) | (1 << DMA_CCR_PSIZE_Pos);
    DMA1_Channel1->CCR |= DMA_CCR_CIRC;                        // circular
    DMA1_Channel1->CCR |= (1 << DMA_CCR_PL_Pos);               // priority medium
    /* Enable DMA interrupt */
    DMA1_Channel1->CCR |= DMA_CCR_TCIE; 					   // transfer complete interrupt

    /* Enable DMA channel */
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    /* NVIC for DMA1 Channel1 IRQ */
    NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 1));
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* Linkage: start ADC multi-mode DMA: in CMSIS we enable DMA request from ADC1 CR2 */
    ADC1->CR2 |= ADC_CR2_DMA;   // enable DMA for regular conversions
    ADC1->CR2 |= ADC_CR2_ADON;  // ensure ADC on (again)
}

/* Initialize ADC2 (injected config only) */
/* ADC2 injected init as slave (software JSWSTART) */
void ADC2_CMSIS_Init(void)
{
    /* Basic reset */
    ADC2->CR1 = 0;
    ADC2->CR2 = 0;

    /* 1) enable ADC clocks (ADC1/ADC2 share APB2ENR ADC1EN on F1) */
    adc_enable_clocks();

    /* 2) ADC2: sampling time for channel 11 (CH11 in SMPR1) */
    adc_set_sample_time(ADC2, 11, ADC_SAMPLETIME_1_5);

    /* 3) Configure injected sequence: 1 conversion, rank1 = ch11 */
    ADC2->JSQR = 0;
    /* JL (injected length) = (n-1) -> for 1 conv JL=0 */
    ADC2->JSQR &= ~ADC_JSQR_JL;
    ADC2->JSQR |= ( (1 - 1) << ADC_JSQR_JL_Pos );

    /* Place channel 11 into JSQ1 (JSQ1 bits) */
    /* Clear jsq1..jsq4 fields then set JSQ1 = 11 */
    ADC2->JSQR &= ~ADC_JSQR_JSQ4;            // clear JSQ fields area
    ADC2->JSQR |= (11 << ADC_JSQR_JSQ4_Pos); // JSQ1 position macro name used as in CMSIS for F1 headers

    /* 4) Trigger: use software start for injected (JSWSTART) -> disable external injected trigger */
    ADC2->CR2 &= ~(ADC_CR2_JEXTSEL | ADC_CR2_JEXTTRIG); /* ensure external injected trigger disabled */

    /* 5) Offset for injected rank1 */
    ADC2->JOFR1 = 0;

    /* 6) Data alignment & scan/continuous: right aligned, no continuous */
    ADC2->CR2 &= ~ADC_CR2_ALIGN;
    ADC2->CR2 &= ~ADC_CR2_CONT;
    ADC2->CR1 &= ~ADC_CR1_SCAN; /* injected only, clear scan for ADC2 unless you need injected scan */

    /* 7) Wake + calibrate ADC2 */
    adc_calibrate(ADC2);

    /* 8) Ensure ADC2 is powered on (ADON set by calibrate helper) */
    ADC2->CR2 |= ADC_CR2_ADON;  // ensure ADC on (again)
}
