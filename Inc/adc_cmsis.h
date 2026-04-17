/*
 * adc_cmsis.h
 *
 *  Created on: 10.04.2026
 *      Author: stephan
 */

#ifndef ADC_CMSIS_H_
#define ADC_CMSIS_H_

#include <stdint.h>

#define ADC_SAMPLETIME_1_5   0
#define ADC_SAMPLETIME_7_5   1
#define ADC_SAMPLETIME_13_5  2
#define ADC_SAMPLETIME_28_5  3
#define ADC_SAMPLETIME_41_5  4
#define ADC_SAMPLETIME_55_5  5
#define ADC_SAMPLETIME_71_5  6
#define ADC_SAMPLETIME_239_5 7

void ADC1_CMSIS_Init(void);
void ADC2_CMSIS_Init(void);
void ADC1_DMA_Init_Circular(volatile uint32_t* adcData, uint16_t len);

#endif /* ADC_CMSIS_H_ */
