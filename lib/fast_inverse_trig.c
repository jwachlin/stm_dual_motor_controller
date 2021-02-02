/*
 * fast_inverse_trig.c
 *
 *  Created on: Jan 12, 2021
 *      Author: jake
 */

#include <stdint.h>
#include <math.h>

#include "fast_inverse_trig.h"

#define PI						3.14159
#define ATAN_ARG_RANGE			(10)

static float acos_arg_table[NUM_ELEMENTS_IT]; // TODO symmetric, can reduce space
static float acos_out_table[NUM_ELEMENTS_IT];
static float atan_arg_table[NUM_ELEMENTS_IT];
static float atan_out_table[NUM_ELEMENTS_IT];
static float acos_div = 0.1;
static float atan_div = 0.1;
int32_t half_range = 10;

void precalc_inverse_trig(void)
{
	int32_t i;
	// acos domain from -1 to 1
	float darg = 2.0 / NUM_ELEMENTS_IT;
	for(i = 0; i < NUM_ELEMENTS_IT; i++)
	{
		float arg = -1.0 + i*darg;
		acos_arg_table[i] = arg;
		acos_out_table[i] = acos(acos_arg_table[i]);
	}
	acos_div = 1.0f / (1.0f / (NUM_ELEMENTS_IT/2));
	half_range = NUM_ELEMENTS_IT / 2;

	// atan domain all real numbers, but asymptotic to +-pi/2 outside of roughly +-20.0
	darg = (2.0 * ATAN_ARG_RANGE) / NUM_ELEMENTS_IT;
	for(i = 0; i < NUM_ELEMENTS_IT; i++)
	{
		float arg = -ATAN_ARG_RANGE + i*darg;
		atan_arg_table[i] = arg;
		atan_out_table[i] = atan(atan_arg_table[i]);
	}
	atan_div = 1.0f / ((1.0f * ATAN_ARG_RANGE) / (NUM_ELEMENTS_IT/2));
}

float fast_acos(float x)
{
	// Wrap to +-1
	if(x > 1.0 || x < -1.0)
	{
		return acos(x); // TODO
	}
	// Fast search
	float temp = x * acos_div; // Divided by 1.0 / NUM_ELEMENTS/2. E.g. 0.5 /( 1.0/ 10 ) goes to 5, -0.5 goes to -5
	int32_t offset = half_range + (int32_t) temp;
	if(offset < 0){offset = 0;}
	if(offset >= NUM_ELEMENTS_IT){offset = NUM_ELEMENTS_IT-1;}
	float val = acos_out_table[offset];
	return val;
}

float fast_atan(float x)
{
	if(x > ATAN_ARG_RANGE)
	{
		return M_PI_2; // TODO interpolate?
	}
	else if(x < -ATAN_ARG_RANGE)
	{
		return -M_PI_2; // TODO interpolate?
	}
	else
	{
		// Search through
		// Fast search
		float temp = x * atan_div; // Divided by RANGE / NUM_ELEMENTS/2. E.g. 10.0 /( 20.0/ 10 ) goes to 5, -10 goes to -5
		int32_t offset = half_range + (int32_t) temp;
		if(offset < 0){offset = 0;}
		if(offset >= NUM_ELEMENTS_IT){offset = NUM_ELEMENTS_IT-1;}
		float val = atan_out_table[offset];
		return val;
	}
}
