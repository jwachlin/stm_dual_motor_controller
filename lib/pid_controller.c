/*
 * pid_controller.c
 *
 *  Created on: Jan 4, 2021
 *      Author: jake
 */


 #include <stdint.h>
 #include "pid_controller.h"

 float calculate_pid(pid_control_t * pid, int32_t setpoint, int32_t current_position)
 {
	float error = setpoint - current_position;

	// Use current vs last position instead of error changes. This is the same if setpoint does not change, but setpoint jumps
	// can cause odd spikes in command otherwise
	pid->speed = (pid->speed_alpha)*pid->speed + (1.0-pid->speed_alpha)*(current_position - pid->last_position);

	pid->integral += error;

	if(pid->integral > pid->integral_max)
	{
		pid->integral = pid->integral_max;
	}
	else if(pid->integral < pid->integral_min)
	{
		pid->integral = pid->integral_min;
	}

	float cmd = pid->kp * error + pid->kd * pid->speed + pid->ki * pid->integral;

	if(cmd > pid->cmd_max)
	{
		cmd	= pid->cmd_max;
	}
	else if(cmd < pid->cmd_min)
	{
		cmd = pid->cmd_min;
	}

	pid->last_error = error;
	pid->last_position = current_position;
	return cmd;
 }
