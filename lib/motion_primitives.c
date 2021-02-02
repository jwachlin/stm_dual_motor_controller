/*
 * motion_primitives.c
 *
 * Created: 9/27/2020 11:17:06 AM
 *  Author: Jake
 */

 #include <stdint.h>
 #include <stdbool.h>
 #include <math.h>

 #include "motion_primitives.h"

 static volatile uint8_t primitive_index = 0;
 static uint32_t time_offset = 0;
 static primitive_t primitives[NUMBER_PRIMITIVES];

 void motion_primitive_init(void)
 {
	// Leg straight, motors at zero is leg straight down +x. Forward is +y
	/*
	*				O-> +y
	*				| \
	*			+x \/	\
	*					 O
	*					 |
	*					 |
	*					 O
	*
	*/

	// t_part must be always increasing, never > 1.0. Must be a cyclical motion primitive

	// Slow trot walk
	// Triangular, 1Hz, roughly 3X longer on ground than in air, front slightly forward of hip
	primitives[0].num_keyframes = 3;
	primitives[0].tau = 2.0;
	primitives[0].t_offset = 0.0;
	primitives[0].invert = 0;
	primitives[0].time_reverse = 0;
	primitives[0].x_offset_m = 0;
	primitives[0].y_offset_m = 0;
	primitives[0].x_scale = 1.0;
	primitives[0].y_scale = 1.0;

	primitives[0].frames[0].t_part = 0.0;
	primitives[0].frames[0].x = 0.105;
	primitives[0].frames[0].y = 0.01;

	primitives[0].frames[1].t_part = 0.75;
	primitives[0].frames[1].x = 0.105;
	primitives[0].frames[1].y = -0.05;

	primitives[0].frames[2].t_part = 0.875;
	primitives[0].frames[2].x = 0.06;
	primitives[0].frames[2].y = -0.02;

	// Fast trot walk
	// Mostly triangular, in air roughly same as on ground, rear slightly backward of hip
	primitives[1].num_keyframes = 4;
	primitives[1].tau = 2.0;
	primitives[1].t_offset = 0.0;
	primitives[1].invert = 0;
	primitives[1].time_reverse = 0;
	primitives[1].x_offset_m = 0;
	primitives[1].y_offset_m = 0;
	primitives[1].x_scale = 1.0;
	primitives[1].y_scale = 1.0;

	primitives[1].frames[0].t_part = 0.0;
	primitives[1].frames[0].x = 0.09;
	primitives[1].frames[0].y = 0.06;

	primitives[1].frames[1].t_part = 0.5;
	primitives[1].frames[1].x = 0.09;
	primitives[1].frames[1].y = -0.01;

	primitives[1].frames[2].t_part = 0.75;
	primitives[1].frames[2].x = 0.05;
	primitives[1].frames[2].y = 0.02;

	primitives[1].frames[3].t_part = 0.95;
	primitives[1].frames[3].x = 0.08;
	primitives[1].frames[3].y = 0.065;

	// Bound
	// Mostly triangular, in air roughly same as on ground, centered on hip
	primitives[2].num_keyframes = 4;
	primitives[2].tau = 2.0;
	primitives[2].t_offset = 0.0;
	primitives[2].invert = 0;
	primitives[2].time_reverse = 0;
	primitives[2].x_offset_m = 0;
	primitives[2].y_offset_m = 0;
	primitives[2].x_scale = 1.0;
	primitives[2].y_scale = 1.0;

	primitives[2].frames[0].t_part = 0.0;
	primitives[2].frames[0].x = 0.105;
	primitives[2].frames[0].y = 0.03;

	primitives[2].frames[1].t_part = 0.5;
	primitives[2].frames[1].x = 0.105;
	primitives[2].frames[1].y = -0.03;

	primitives[2].frames[2].t_part = 0.7;
	primitives[2].frames[2].x = 0.08;
	primitives[2].frames[2].y = -0.015;

	primitives[2].frames[3].t_part = 0.9;
	primitives[2].frames[3].x = 0.095;
	primitives[2].frames[3].y = 0.04;

	// Pronk
	// Only vertical motion, slow down, fast down, fast up, fast to nominal
	primitives[3].num_keyframes = 5;
	primitives[3].tau = 2.0;
	primitives[3].t_offset = 0.0;
	primitives[3].invert = 0;
	primitives[3].time_reverse = 0;
	primitives[3].x_offset_m = 0;
	primitives[3].y_offset_m = 0;
	primitives[3].x_scale = 1.0;
	primitives[3].y_scale = 1.0;

	primitives[3].frames[0].t_part = 0.0;
	primitives[3].frames[0].x = 0.07;
	primitives[3].frames[0].y = 0.0;

	primitives[3].frames[1].t_part = 0.3;
	primitives[3].frames[1].x = 0.09;
	primitives[3].frames[1].y = 0.0;

	primitives[3].frames[2].t_part = 0.4;
	primitives[3].frames[2].x = 0.105;
	primitives[3].frames[2].y = 0.0;

	primitives[3].frames[3].t_part = 0.5;
	primitives[3].frames[3].x = 0.05;
	primitives[3].frames[3].y = 0.0;

	primitives[3].frames[4].t_part = 0.6;
	primitives[3].frames[4].x = 0.07;
	primitives[3].frames[4].y = 0.0;

	/*************  Bezier curves starting  **************************/

	// Slow trot walk with quadratic Bezier curve
	// Triangular, 1Hz, roughly 3X longer on ground than in air, front slightly forward of hip
	primitives[4].num_keyframes = 6;
	primitives[4].tau = 2.0;
	primitives[4].t_offset = 0.0;
	primitives[4].invert = 0;
	primitives[4].time_reverse = 0;
	primitives[4].x_offset_m = 0;
	primitives[4].y_offset_m = 0;
	primitives[4].x_scale = 1.0;
	primitives[4].y_scale = 1.0;

	primitives[4].frames[0].t_part = 0.0;
	primitives[4].frames[0].x = 0.105;
	primitives[4].frames[0].y = 0.01;

	primitives[4].frames[1].t_part = 0.375; // Time not used
	primitives[4].frames[1].x = 0.115;
	primitives[4].frames[1].y = -0.02;

	primitives[4].frames[2].t_part = 0.75;
	primitives[4].frames[2].x = 0.105;
	primitives[4].frames[2].y = -0.05;

	primitives[4].frames[3].t_part = 0.8; // Time not used
	primitives[4].frames[3].x = 0.07;
	primitives[4].frames[3].y = -0.075;

	primitives[4].frames[4].t_part = 0.875;
	primitives[4].frames[4].x = 0.06;
	primitives[4].frames[4].y = -0.02;

	primitives[4].frames[5].t_part = 0.9; // Time not used
	primitives[4].frames[5].x = 0.07;
	primitives[4].frames[5].y = 0.035;

	// Fast trot walk
	// Mostly triangular, in air roughly same as on ground, rear slightly backward of hip
	primitives[5].num_keyframes = 6;
	primitives[5].tau = 2.0;
	primitives[5].t_offset = 0.0;
	primitives[5].invert = 0;
	primitives[5].time_reverse = 0;
	primitives[5].x_offset_m = 0;
	primitives[5].y_offset_m = 0;
	primitives[5].x_scale = 1.0;
	primitives[5].y_scale = 1.0;

	primitives[5].frames[0].t_part = 0.0;
	primitives[5].frames[0].x = 0.09;
	primitives[5].frames[0].y = 0.06;

	primitives[5].frames[1].t_part = 0.25; // Time not used
	primitives[5].frames[1].x = 0.105;
	primitives[5].frames[1].y = 0.025;

	primitives[5].frames[2].t_part = 0.5;
	primitives[5].frames[2].x = 0.09;
	primitives[5].frames[2].y = -0.01;

	primitives[5].frames[3].t_part = 0.6; // Time not used
	primitives[5].frames[3].x = 0.06;
	primitives[5].frames[3].y = -0.025;

	primitives[5].frames[4].t_part = 0.75;
	primitives[5].frames[4].x = 0.06;
	primitives[5].frames[4].y = 0.02;

	primitives[5].frames[5].t_part = 0.85; // Time not used
	primitives[5].frames[5].x = 0.06;
	primitives[5].frames[5].y = 0.08;
 }

 void motion_primitive_time_sync(uint32_t external_time)
 {
	time_offset = (xTaskGetTickCount() - external_time);
 }

 void motion_primitive_set_index(uint8_t index)
 {
	if(index >= NUMBER_PRIMITIVES)
	{
		return;
	}
	primitive_index = index;
 }

 void motion_primitive_set_timing(uint8_t index, float tau, float t_offset, uint8_t invert, uint8_t time_reverse)
 {
	if(index >= NUMBER_PRIMITIVES)
	{
		return;
	}
	primitives[index].tau = tau;
	primitives[index].t_offset = t_offset;
	primitives[index].invert = invert;
	primitives[index].time_reverse = time_reverse;
 }

 void motion_primitive_set_scaling(uint8_t index, float x_offset, float y_offset, float x_scale, float y_scale)
 {
	if(index >= NUMBER_PRIMITIVES)
	{
		return;
	}
	primitives[index].x_offset_m = x_offset;
	primitives[index].y_offset_m = y_offset;
	primitives[index].x_scale = x_scale;
	primitives[index].y_scale = y_scale;
 }

 void motion_primitive_get_position(float * x, float * y)
 {
	// First find global time,taking into account time syncs and primitive local offsets
	float current_t = 0.001 * (xTaskGetTickCount() - time_offset) - primitives[primitive_index].t_offset;

	float time_in_cycle = fmod(current_t, primitives[primitive_index].tau);
	float time_in_cycle_part = time_in_cycle / primitives[primitive_index].tau; // Prevent further mults

	if(primitives[primitive_index].time_reverse > 0)
	{
		time_in_cycle_part = 1.0 - time_in_cycle_part;
	}

	// Now find where we are in the cycle and linearly interpolate
	int i;
	float x_cmd, y_cmd;
	for(i = 1; i < primitives[primitive_index].num_keyframes; i++)
	{
		// Are we between keyframes?
		if(time_in_cycle_part >= primitives[primitive_index].frames[i-1].t_part && time_in_cycle_part < primitives[primitive_index].frames[i].t_part)
		{
			// Last value plus section of new value
			float dt = (time_in_cycle_part - primitives[primitive_index].frames[i-1].t_part);
			float d_section_dt = 1.0 / (primitives[primitive_index].frames[i].t_part - primitives[primitive_index].frames[i-1].t_part);
			x_cmd = primitives[primitive_index].frames[i-1].x + (dt * (primitives[primitive_index].frames[i].x - primitives[primitive_index].frames[i-1].x) * d_section_dt);
			y_cmd = primitives[primitive_index].frames[i-1].y + (dt * (primitives[primitive_index].frames[i].y - primitives[primitive_index].frames[i-1].y) * d_section_dt);
			// Apply offset, then scale it
			x_cmd += primitives[primitive_index].x_offset_m;
			x_cmd *= primitives[primitive_index].x_scale;
			y_cmd += primitives[primitive_index].y_offset_m;
			y_cmd *= primitives[primitive_index].y_scale;

			*x = x_cmd;
			*y = y_cmd;
			return;
		}
		// Are we at the end, and after last keyframe
		if(i == (primitives[primitive_index].num_keyframes-1) && time_in_cycle_part >= primitives[primitive_index].frames[i].t_part)
		{
			// Last value plus section of new value
			float dt = (time_in_cycle_part - primitives[primitive_index].frames[i].t_part);
			float d_section_dt = 1.0 / (1.0 - primitives[primitive_index].frames[i].t_part);
			x_cmd = primitives[primitive_index].frames[i].x + (dt * (primitives[primitive_index].frames[0].x - primitives[primitive_index].frames[i].x) * d_section_dt); // Cyclical, so zero index is next frame at end
			y_cmd = primitives[primitive_index].frames[i].y + (dt * (primitives[primitive_index].frames[0].y - primitives[primitive_index].frames[i].y) * d_section_dt);
			// Apply offset, then scale it
			x_cmd += primitives[primitive_index].x_offset_m;
			x_cmd *= primitives[primitive_index].x_scale;
			y_cmd += primitives[primitive_index].y_offset_m;
			y_cmd *= primitives[primitive_index].y_scale;

			*x = x_cmd;
			*y = y_cmd;
			return;
		}
	}
 }

 void motion_primitive_get_position_bezier_quadratic(float * x, float * y)
 {
	if( primitives[primitive_index].num_keyframes % 2 != 0)
	{
		// Stay at first element, assume it is safe.
		// Bezier quadratic curve cycle must have even number of elements
		*x = primitives[primitive_index].frames[0].x;
		*y = primitives[primitive_index].frames[0].y;
	}
	// First find global time,taking into account time syncs and primitive local offsets
	float current_t = 0.001 * (xTaskGetTickCount() - time_offset) - primitives[primitive_index].t_offset;

	float time_in_cycle = fmod(current_t, primitives[primitive_index].tau);
	float time_in_cycle_part = time_in_cycle / primitives[primitive_index].tau; // Prevent further mults

	if(primitives[primitive_index].time_reverse > 0)
	{
		time_in_cycle_part = 1.0 - time_in_cycle_part;
	}

	// Now find where we are in the cycle and Bezier quadratic interpolation
	// Every odd keyframe is a guide point. Path moves through even keyframes
	int i;
	float x_cmd, y_cmd;
	for(i = 1; i < primitives[primitive_index].num_keyframes/2; i++)
	{
		// find the keyframe, looking between every other, starting at index 0
		// e.g. for 6 keyframes, we look at 0, 2, 4, and consider "6" as 0 again
		if(time_in_cycle_part >= primitives[primitive_index].frames[2*i-2].t_part && time_in_cycle_part < primitives[primitive_index].frames[2*i].t_part)
		{
			// scale to something that varies 0 <= dt < 1 between the points
			float dt = (primitives[primitive_index].frames[2*i].t_part - time_in_cycle_part) / (primitives[primitive_index].frames[2*i].t_part - primitives[primitive_index].frames[2*i - 2].t_part);
			float one_min_dt = 1.0 - dt; // used many times
			x_cmd = one_min_dt*(one_min_dt*primitives[primitive_index].frames[2*i].x + dt*primitives[primitive_index].frames[2*i-1].x) + dt * (one_min_dt * primitives[primitive_index].frames[2*i-1].x + dt * primitives[primitive_index].frames[2*i-2].x);
			y_cmd = one_min_dt*(one_min_dt*primitives[primitive_index].frames[2*i].y + dt*primitives[primitive_index].frames[2*i-1].y) + dt * (one_min_dt * primitives[primitive_index].frames[2*i-1].y + dt * primitives[primitive_index].frames[2*i-2].y);
			// Apply offset, then scale it
			x_cmd += primitives[primitive_index].x_offset_m;
			x_cmd *= primitives[primitive_index].x_scale;
			y_cmd += primitives[primitive_index].y_offset_m;
			y_cmd *= primitives[primitive_index].y_scale;

			*x = x_cmd;
			*y = y_cmd;
			return;
		}
		// Are we at the end, and after last keyframe
		if(i == ((primitives[primitive_index].num_keyframes/2)-1) && time_in_cycle_part >= primitives[primitive_index].frames[2*i].t_part)
		{
			// Cyclical, so zero index is last
			// scale to something that varies 0 <= dt < 1 between the points
			float dt = (1.0 - time_in_cycle_part) / (1.0 - primitives[primitive_index].frames[2*i].t_part);
			float one_min_dt = 1.0 - dt; // used many times
			x_cmd = one_min_dt*(one_min_dt*primitives[primitive_index].frames[0].x + dt*primitives[primitive_index].frames[2*i+1].x) + dt * (one_min_dt * primitives[primitive_index].frames[2*i+1].x + dt * primitives[primitive_index].frames[2*i].x);
			y_cmd = one_min_dt*(one_min_dt*primitives[primitive_index].frames[0].y + dt*primitives[primitive_index].frames[2*i+1].y) + dt * (one_min_dt * primitives[primitive_index].frames[2*i+1].y + dt * primitives[primitive_index].frames[2*i].y);
			// Apply offset, then scale it
			x_cmd += primitives[primitive_index].x_offset_m;
			x_cmd *= primitives[primitive_index].x_scale;
			y_cmd += primitives[primitive_index].y_offset_m;
			y_cmd *= primitives[primitive_index].y_scale;

			*x = x_cmd;
			*y = y_cmd;
			return;
		}
	}
 }

 bool motion_primitive_is_inverted(void)
 {
	return (primitives[primitive_index].invert > 0);
 }

 void motion_primitive_set_keyframe(uint8_t index, uint8_t keyframe_index, float x, float y, float t_part)
 {
	if(index >= NUMBER_PRIMITIVES)
	{
		return;
	}
	if(keyframe_index >= MAX_NUMBER_KEYFRAMES)
	{
		return;
	}
	primitives[index].frames[keyframe_index].t_part = t_part;
	primitives[index].frames[keyframe_index].x = x;
	primitives[index].frames[keyframe_index].y = y;
 }

 uint8_t get_motion_primitive(void)
 {
	return primitive_index;
 }
