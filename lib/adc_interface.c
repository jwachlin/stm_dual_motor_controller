/*
 * adc_interface.c
 *
 *  Created on: Jan 11, 2021
 *      Author: jake
 */

#include "main.h"

#include "adc_interface.h"

static ADC_HandleTypeDef m1;
static ADC_HandleTypeDef m2;

void adc_interface_init(ADC_HandleTypeDef ch1, ADC_HandleTypeDef ch2)
{
	m1 = ch1;
	m2 = ch2;
	HAL_ADC_Start(&m1);
	HAL_ADC_Start(&m2);
}

uint32_t get_motor_current_ma(uint8_t channel)
{
	if(channel == 0)
	{
		HAL_ADC_PollForConversion(&m1, 5);
		uint16_t raw = HAL_ADC_GetValue(&m1);
		uint32_t current_ma = (uint32_t) (((raw / 4095.0) * 3.0)/2.15) * 1000.0;
		return current_ma;
	}
	else if(channel == 1)
	{
		HAL_ADC_PollForConversion(&m2, 5);
		uint16_t raw = HAL_ADC_GetValue(&m2);
		uint32_t current_ma = (uint32_t) (((raw / 4095.0) * 3.0)/2.15) * 1000.0;
		return current_ma;
	}
	else
	{
		return 0;
	}
}
