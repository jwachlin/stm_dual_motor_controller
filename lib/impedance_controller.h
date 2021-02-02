/*
 * impedance_controller.h
 *
 * Created: 8/30/2020 3:48:53 PM
 *  Author: Jake
 */


#ifndef IMPEDANCE_CONTROLLER_H_
#define IMPEDANCE_CONTROLLER_H_

#include "inverse_kinematics.h"

typedef struct
{
	float gain_current_per_torque; // In A / N-m
	float k_eff_x; // In N / m
	float c_eff_x; // In N / m/s
	float k_eff_y; // In N / m
	float c_eff_y; // In N / m/s
	float gear_ratio;
} impedance_control_params_t;

typedef struct
{
	int32_t hip_cmd_ma;
	int32_t knee_cmd_ma;
} impedance_control_cmds_t;

void calculate_impedance_control(const impedance_control_params_t params, const leg_ik_t leg, const pos_joint_space_t current_pos, const pos_cartesian_t desired_pos, impedance_control_cmds_t * cmds);

#endif /* IMPEDANCE_CONTROLLER_H_ */
