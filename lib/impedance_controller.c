/*
 * impedance_controller.c
 *
 * Created: 8/30/2020 3:48:37 PM
 *  Author: Jake
 */

 #include <stdint.h>
 #include <stdbool.h>
 #include "FreeRTOS.h"
 #include <math.h>

 #include "motor_task.h"
 #include "inverse_kinematics.h"

 #include "impedance_controller.h"

 void calculate_impedance_control(const impedance_control_params_t params, const leg_ik_t leg, const pos_joint_space_t current_pos, const pos_cartesian_t desired_pos, impedance_control_cmds_t * cmds)
 {
	static pos_cartesian_t last_pos_cart;
	static pos_cartesian_t last_des_pos_cart;
	static uint32_t last_time;
	static pos_cartesian_t act_cart_speed_mps;
	static pos_cartesian_t des_cart_speed_mps;
	pos_cartesian_t current_pos_cart;
	jacobian_t jac;

	//calculate_fk(&leg, &current_pos_cart, current_pos);
	calculate_fk_and_jacobian(&leg, &current_pos_cart, current_pos, &jac);

	// Calculate speed
	/*uint32_t current_time = xTaskGetTickCount();
	float dt = ((current_time - last_time) * 0.001);
	act_cart_speed_mps.x = 0.95 * act_cart_speed_mps.x + 0.05 * ((current_pos_cart.x - last_pos_cart.x) / dt);
	act_cart_speed_mps.y = 0.95 * act_cart_speed_mps.y + 0.05 * ((current_pos_cart.y - last_pos_cart.y) / dt);

	des_cart_speed_mps.x = 0.95 * des_cart_speed_mps.x + 0.05 *((desired_pos.x - last_des_pos_cart.x) / dt);
	des_cart_speed_mps.y = 0.95 * des_cart_speed_mps.y + 0.05 *((desired_pos.y - last_des_pos_cart.y) / dt);

	last_pos_cart.x = current_pos_cart.x;
	last_pos_cart.y = current_pos_cart.y;
	last_des_pos_cart.x = desired_pos.x;
	last_des_pos_cart.y = desired_pos.y;
	last_time = current_time;*/

	// Calculate desired force from springs
	float fx = params.k_eff_x * (desired_pos.x - current_pos_cart.x); // In N
	float fy = params.k_eff_y * (desired_pos.y - current_pos_cart.y);
	// Damping
	//fx += params.c_eff_x * (act_cart_speed_mps.x); // TODO use relative
	//fy += params.c_eff_y * (act_cart_speed_mps.y);

	// Calculate desired torque, tau_d = J^T * f_d
	float tau_hip = fx*jac.j_00 + fy*jac.j_10; // In N * m
	float tau_knee = fx*jac.j_01 + fy*jac.j_11;

	// Calculate using inverse Jacobian
	/*float det_inv = 1.0 / ((jac.j_00 * jac.j_11) - (jac.j_01 * jac.j_10));
	float j_00_inv = det_inv * jac.j_11;
	float j_01_inv = det_inv * (-jac.j_01);
	float j_10_inv = det_inv * (-jac.j_10);
	float j_11_inv = det_inv * jac.j_00;
	float tau_hip = fx*j_00_inv + fy*j_01_inv; // In N * m
	float tau_knee = fx*j_10_inv + fy*j_11_inv;*/

	// Account for gear ratio
	tau_hip = tau_hip / params.gear_ratio;
	tau_knee = tau_knee / params.gear_ratio;

	// Calculate desired current to meet torque
	float current_hip = params.gain_current_per_torque * tau_hip;
	float current_knee = params.gain_current_per_torque * tau_knee;

	// Current controller outside of here
	cmds->hip_cmd_ma = current_hip * 1000.0;
	cmds->knee_cmd_ma = current_knee * 1000.0;
 }
