/*
 * adc_interface.h
 *
 *  Created on: Jan 11, 2021
 *      Author: jake
 */

#ifndef ADC_INTERFACE_H_
#define ADC_INTERFACE_H_

#include "main.h"

void adc_interface_init(ADC_HandleTypeDef ch1, ADC_HandleTypeDef ch2);
void sample_filter_adc(void);
uint32_t get_motor_current_ma(uint8_t channel);

#endif /* ADC_INTERFACE_H_ */
