/*
 * adc_irg_layer.c
 *
 *  Created on: 10.04.2026
 *      Author: stephan
 */

#include "adc_irq_layer.h"
#include "stm32f103x6.h"
#include "stm32f1xx.h"
#include "main.h"
#include "config.h"
#include "FOC.h"
#include <arm_math.h>

/* ADC1_2 IRQ handler (handles both ADC1 and ADC2 JEOC/EOC) */
void ADC1_2_IRQHandler(void)
{
	uint32_t sr1 = ADC1->SR;

    if (sr1 & ADC_SR_JEOC) {
        /* clear JEOC by reading injected data registers or by clearing flags */
    	ADC1->SR = ~(ADC_SR_JEOC);
        /* injected end of conversion */
        ADC_InjectedConvCpltCallback();	//handle_injected_jeoc();
    }
}

void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR = DMA_IFCR_CTCIF1; // clear flag

        ui8_adc_regular_flag = 1;
    }
}
