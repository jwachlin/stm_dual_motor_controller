/*
 * motion_primitives.h
 *
 * Created: 9/27/2020 11:17:20 AM
 *  Author: Jake
 */


#ifndef MOTION_PRIMITIVES_H_
#define MOTION_PRIMITIVES_H_

#include <stdbool.h>

#define MAX_NUMBER_KEYFRAMES		6
#define NUMBER_LINEAR_PRIMITIVES	4
#define NUMBER_BEZIER_PRIMITIVES	2
#define NUMBER_PRIMITIVES			6


typedef struct
{
	float x;
	float y;
	float t_part;
} keyframe_t;

typedef struct
{
	uint8_t num_keyframes;
	uint8_t invert;
	uint8_t time_reverse;
	float x_offset_m;
	float y_offset_m;
	float x_scale;
	float y_scale;
	float tau; // period
	float t_offset;
	keyframe_t frames[MAX_NUMBER_KEYFRAMES];
} primitive_t;

void motion_primitive_init(void);
void motion_primitive_time_sync(uint32_t external_time);
void motion_primitive_set_index(uint8_t index);
void motion_primitive_set_timing(uint8_t index, float tau, float t_offset, uint8_t invert, uint8_t time_reverse);
void motion_primitive_set_scaling(uint8_t index, float x_offset, float y_offset, float x_scale, float y_scale);
void motion_primitive_get_position(float * x, float * y);
void motion_primitive_get_position_bezier_quadratic(float * x, float * y);
bool motion_primitive_is_inverted(void);
void motion_primitive_set_keyframe(uint8_t index, uint8_t keyframe_index, float x, float y, float t_part);
uint8_t get_motion_primitive(void);

#endif /* MOTION_PRIMITIVES_H_ */
