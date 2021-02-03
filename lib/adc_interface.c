/*
 * adc_interface.c
 *
 *  Created on: Jan 11, 2021
 *      Author: jake
 */

#include "main.h"

#include "adc_interface.h"
#include "arm_math.h"

static ADC_HandleTypeDef m1;
static ADC_HandleTypeDef m2;
static float current_ma_filt[2];
static float alpha = 0.99;

void adc_interface_init(ADC_HandleTypeDef ch1, ADC_HandleTypeDef ch2)
{
	m1 = ch1;
	m2 = ch2;
	HAL_ADC_Start(&m1);
	HAL_ADC_Start(&m2);
}

void sample_filter_adc(void)
{
	uint8_t channel = 0;
	HAL_ADC_PollForConversion(&m1, 5);
	uint16_t raw = HAL_ADC_GetValue(&m1);
	float current_ma_f = ((((raw / 4095.0) * 3.0)/2.15) * 1000.0);
	current_ma_filt[channel] = alpha * current_ma_filt[channel] + (1.0f-alpha) * current_ma_f;

	channel = 1;
	HAL_ADC_PollForConversion(&m2, 5);
	raw = HAL_ADC_GetValue(&m2);
	current_ma_f = ((((raw / 4095.0) * 3.0)/2.15) * 1000.0);
	current_ma_filt[channel] = alpha * current_ma_filt[channel] + (1.0f-alpha) * current_ma_f;
}

uint32_t get_motor_current_ma(uint8_t channel)
{
	if(channel == 0)
	{
		uint32_t current_ma = (uint32_t) current_ma_filt[channel];
		return current_ma;
	}
	else if(channel == 1)
	{
		uint32_t current_ma = (uint32_t) current_ma_filt[channel];
		return current_ma;
	}
	else
	{
		return 0;
	}
}
