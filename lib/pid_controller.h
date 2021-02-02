/*
 * pid_controller.h
 *
 *  Created on: Jan 4, 2021
 *      Author: jake
 */

#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

typedef struct
{
	int32_t last_position;
	float last_error;
	float speed;
	float integral;
	float integral_max;
	float integral_min;
	float cmd_max;
	float cmd_min;
	float speed_alpha;
	float kp;
	float ki;
	float kd;
} pid_control_t;

float calculate_pid(pid_control_t * pid, int32_t setpoint, int32_t current_position);

#endif /* PID_CONTROLLER_H_ */
