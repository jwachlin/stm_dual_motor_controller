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

 #include "canbus_task.h"
 #include "motor_task.h"
 #include "inverse_kinematics.h"
 #include "can_messages.h"

 #include "impedance_controller.h"

 static const float speed_alpha = 0.995;

 void calculate_impedance_control(const impedance_control_params_t params, const leg_ik_t leg, const pos_joint_space_t current_pos, const pos_cartesian_t desired_pos, impedance_control_cmds_t * cmds)
 {
	static pos_cartesian_t last_pos_cart;
	static pos_cartesian_t last_des_pos_cart;
	static uint32_t last_time;
	static pos_cartesian_t act_cart_speed_mps;
	static pos_cartesian_t des_cart_speed_mps;
	static uint32_t cycle_count = 0;
	pos_cartesian_t current_pos_cart;
	jacobian_t jac;

	calculate_fk_and_jacobian(&leg, &current_pos_cart, current_pos, &jac);

	// Calculate speed
	uint32_t current_time = xTaskGetTickCount();
	float dt = 0.00025; // 4kHz loop
	act_cart_speed_mps.x = speed_alpha * act_cart_speed_mps.x + (1.0-speed_alpha) * ((current_pos_cart.x - last_pos_cart.x) / dt);
	act_cart_speed_mps.y = speed_alpha * act_cart_speed_mps.y + (1.0-speed_alpha) * ((current_pos_cart.y - last_pos_cart.y) / dt);

	des_cart_speed_mps.x = speed_alpha * des_cart_speed_mps.x + (1.0-speed_alpha) *((desired_pos.x - last_des_pos_cart.x) / dt);
	des_cart_speed_mps.y = speed_alpha * des_cart_speed_mps.y + (1.0-speed_alpha) *((desired_pos.y - last_des_pos_cart.y) / dt);

	last_pos_cart.x = current_pos_cart.x;
	last_pos_cart.y = current_pos_cart.y;
	last_des_pos_cart.x = desired_pos.x;
	last_des_pos_cart.y = desired_pos.y;
	last_time = current_time;

	// Calculate desired force from springs
	float fx = params.k_eff_x * (desired_pos.x - current_pos_cart.x); // In N
	float fy = params.k_eff_y * (desired_pos.y - current_pos_cart.y);
	// Damping
	fx += params.c_eff_x * (act_cart_speed_mps.x - des_cart_speed_mps.x); // TODO use relative
	fy += params.c_eff_y * (act_cart_speed_mps.y - des_cart_speed_mps.y);

	if(cycle_count++ % 40)
	{
		canbus_frame_t frame;
		can_message_id_t id_helper;

		id_helper.can_msg_type = CAN_MSG_TYPE_INFO;
		id_helper.can_class = CAN_MSG_CLASS_INFO_TELEMETRY;
		id_helper.can_device = get_device_index();

		id_helper.can_index = CAN_MSG_INDEX_INFO_PROPRIO_FORCE;
		pack_can_message(&id_helper);

		frame.id = id_helper.raw_id;
		frame.length = 8;
		memcpy(&frame.data[0], &fx, 4);
		memcpy(&frame.data[4], &fy, 4);
		add_can_frame_to_tx_queue(frame);
	}

	// Calculate desired torque, tau_d = J^T * f_d
	// In N * m
	float tau_hip = fx*jac.j_00 + fy*jac.j_10;
	float tau_knee = fx*jac.j_01 + fy*jac.j_11;


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
