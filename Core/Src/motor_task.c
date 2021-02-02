/*
 * motor_task.c
 *
 *  Created on: Jan 4, 2021
 *      Author: jake
 */

 #include <stdint.h>
 #include <string.h>

 #include "main.h"
 #include "canbus_task.h"
 #include "../../Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h"
 #include "../../Middlewares/Third_Party/FreeRTOS/Source/include/timers.h"
 #include "../../Middlewares/Third_Party/FreeRTOS/Source/include/task.h"

 #include "../../lib/pid_controller.h"
 #include "../../lib/encoder_interface.h"
 #include "../../lib/inverse_kinematics.h"
 #include "../../lib/impedance_controller.h"
 #include "../../lib/adc_interface.h"
 #include "../../lib/can_messages.h"
 #include "../../lib/motion_primitives.h"
 #include "../../lib/fast_inverse_trig.h"

 #include <math.h>

 #include "motor_task.h"

 #define RAD_TO_DEG			(57.2958)
 #define DEG_TO_RAD			(0.0174533)

 static TimerHandle_t motor_tele_timer_handle = NULL;

 static volatile CONTROL_TYPE control_type[NUMBER_MOTORS];
 static volatile motor_t motors[NUMBER_MOTORS];
 static pid_control_t pos_params[NUMBER_MOTORS];
 static pid_control_t speed_params[NUMBER_MOTORS];
 static pid_control_t cur_params[NUMBER_MOTORS];

 static impedance_control_params_t ic_params; // TODO different per motor?
 static leg_ik_t leg;

 static drive_motor(uint8_t index, float cmd)
{
	 if(index == 0)
	 {
		 // IN1 and IN2, TIM2_CH3/4
	 	 if(cmd > 0)
	 	 {
	 		if(motors[index].reverse_direction)
	 		{
	 			TIM2->CCR4 = 0;
	 			TIM2->CCR3 = (uint16_t) cmd;
	 		}
	 		else
	 		{
	 			TIM2->CCR4 = (uint16_t) cmd;
	 			TIM2->CCR3 = 0;
	 		}
	 	 }
	 	 else
	 	 {
	 		 if(motors[index].reverse_direction)
	 		 {
	 		 	TIM2->CCR3 = 0;
	 		 	TIM2->CCR4 = (uint16_t) -cmd;
	 		 }
	 		 else
	 		 {
	 		 	TIM2->CCR3 = (uint16_t) -cmd;
	 		 	TIM2->CCR4 = 0;
	 		 }
	 	 }
	  }
	 else if(index == 1)
	 {
		 // IN3 - TIM13_CH1
		 // IN4 - TIM14_CH1
		 if(cmd > 0)
		 {
			 if(motors[index].reverse_direction)
		 	{
		 		TIM14->CCR1 = 0;
		 		TIM13->CCR1 = (uint16_t) cmd;
		 	}
		 	else
		 	{
		 		TIM14->CCR1 = (uint16_t) cmd;
		 		TIM13->CCR1 = 0;
		 	}
		 }
		 else
		 {
		 	 if(motors[index].reverse_direction)
		 	 {
		 	 	TIM13->CCR1 = 0;
		 	 	TIM14->CCR1 = (uint16_t) -cmd;
		 	 }
		 	 else
		 	 {
		 	 	TIM13->CCR1 = (uint16_t) -cmd;
		 	 	TIM14->CCR1 = 0;
		 	 }
		 }
	 }
}

 static void vMotorTelemetryTimerCallback( TimerHandle_t xTimer )
  {
 	 canbus_frame_t frame;
 	 can_message_id_t id_helper;

 	 id_helper.can_msg_type = CAN_MSG_TYPE_INFO;
 	 id_helper.can_class = CAN_MSG_CLASS_INFO_TELEMETRY;
 	 id_helper.can_device = get_device_index();


 	id_helper.can_index = CAN_MSG_INDEX_INFO_POSITION;
 	pack_can_message(&id_helper);

 	frame.id = id_helper.raw_id;
 	frame.length = 8;
 	memcpy(&frame.data[0], &motors[0].ticks_count, 4);
 	memcpy(&frame.data[4], &motors[1].ticks_count, 4);
 	add_can_frame_to_tx_queue(frame);

 	id_helper.can_index = CAN_MSG_INDEX_INFO_CURRENT;
 	pack_can_message(&id_helper);

 	frame.id = id_helper.raw_id;
 	frame.length = 8;
 	memcpy(&frame.data[0], &motors[0].current_mA, 4);
 	memcpy(&frame.data[4], &motors[1].current_mA, 4);
 	add_can_frame_to_tx_queue(frame);

 	id_helper.can_index = CAN_MSG_INDEX_INFO_SPEED;
 	pack_can_message(&id_helper);

 	frame.id = id_helper.raw_id;
 	frame.length = 8;
 	memcpy(&frame.data[0], &motors[0].speed, 4);
 	memcpy(&frame.data[4], &motors[1].speed, 4);
 	add_can_frame_to_tx_queue(frame);

 	id_helper.can_index = CAN_MSG_INDEX_INFO_POSITION_SETPOINT;
 	pack_can_message(&id_helper);

 	frame.id = id_helper.raw_id;
 	frame.length = 8;
 	memcpy(&frame.data[0], &motors[0].ticks_setpoint, 4);
 	memcpy(&frame.data[4], &motors[1].ticks_setpoint, 4);
 	add_can_frame_to_tx_queue(frame);

 	id_helper.can_index = CAN_MSG_INDEX_INFO_CURRENT_SETPOINT;
 	pack_can_message(&id_helper);

 	frame.id = id_helper.raw_id;
 	frame.length = 8;
 	memcpy(&frame.data[0], &motors[0].current_ma_setpoint, 4);
 	memcpy(&frame.data[4], &motors[1].current_ma_setpoint, 4);
 	add_can_frame_to_tx_queue(frame);

 		// TODO other telemetry
  }

 static void vPrimitivesCalculation(void)
  {
	 static uint32_t prim_count = 0;
	int32_t i;
	// Only calculate if all primitive or current primitive
	for(i=0; i < NUMBER_MOTORS; i++)
	{
		if(control_type[i] != PRIMITIVE && control_type[i] != PROPRIOCEPTIVE_PRIMITIVE)
		{
			return;
		}
	}

	pos_cartesian_t cart_pos;
	pos_joint_space_t js_pos;

	if(get_motion_primitive() < NUMBER_LINEAR_PRIMITIVES)
	{
		motion_primitive_get_position(&cart_pos.x, &cart_pos.y);
	}
	else
	{
		motion_primitive_get_position_bezier_quadratic(&cart_pos.x, &cart_pos.y);
	}

	prim_count++;

	if(prim_count % 20 == 0)
	{
		// 200 Hz output
		canbus_frame_t frame;
		can_message_id_t id_helper;

		id_helper.can_msg_type = CAN_MSG_TYPE_INFO;
		id_helper.can_class = CAN_MSG_CLASS_INFO_TELEMETRY;
		id_helper.can_device = get_device_index();

		id_helper.can_index = CAN_MSG_INDEX_INFO_PRIMITIVE_SETPOINT;
		pack_can_message(&id_helper);

		frame.id = id_helper.raw_id;
		frame.length = 8;
		memcpy(&frame.data[0], &cart_pos.x, 4);
		memcpy(&frame.data[4], &cart_pos.y, 4);
		add_can_frame_to_tx_queue(frame);
	}

	if(control_type[0] == PRIMITIVE)
	{
		calculate_ik(&leg, &js_pos, cart_pos);

		if(motion_primitive_is_inverted())
		{
			js_pos.thigh_angle_rad = -js_pos.thigh_angle_rad;
			js_pos.knee_angle_rad = -js_pos.knee_angle_rad;
		}

		set_motor_position(RAD_TO_DEG * js_pos.thigh_angle_rad, 0);
		set_motor_position(RAD_TO_DEG * js_pos.knee_angle_rad, 1);
	} else if(control_type[0] == PROPRIOCEPTIVE_PRIMITIVE)
	{
		impedance_control_cmds_t ic_cmds;
		js_pos.thigh_angle_rad = DEG_TO_RAD * get_motor_position(0);
		js_pos.knee_angle_rad = DEG_TO_RAD * get_motor_position(1);
		calculate_impedance_control(ic_params, leg, js_pos, cart_pos, &ic_cmds);
		motors[0].current_ma_setpoint = ic_cmds.hip_cmd_ma;
		motors[1].current_ma_setpoint = ic_cmds.knee_cmd_ma;
	}
  }

 static void vControlTimerCallback( void )
 {
	static uint32_t count = 0;
	static int32_t last_ticks_for_speed[NUMBER_MOTORS] = {0};

	int32_t i;
	vPrimitivesCalculation();

	// Update data
	for(i=0; i < NUMBER_MOTORS; i++)
	{
		// Speed is in deg/s
		if(count % 20 == 0)
		{
			// 25 Hz here
			int32_t ticks_per_40ms = (get_motor_encoder_ticks(i) - last_ticks_for_speed[i]);
			int32_t deg_p_s = 25 * ticks_per_40ms * (360.0 / motors[i].ticks_per_rev);
			motors[i].speed = (int32_t) (0.7 * motors[i].speed) + (int32_t) (0.3 * deg_p_s); // TODO filter differently?

			last_ticks_for_speed[i] = get_motor_encoder_ticks(i);
		}

		motors[i].ticks_count = get_motor_encoder_ticks(i);
		motors[i].current_mA = get_motor_current_ma(i);

		if(control_type[i] == POSITION || control_type[i] == PRIMITIVE)
		{
			float cmd = calculate_pid(&pos_params[i], motors[i].ticks_setpoint, motors[i].ticks_count);
			drive_motor(i, cmd);
		} else if(control_type[i] == SPEED)
		{
			float cmd = calculate_pid(&speed_params[i], motors[i].speed_setpoint, motors[i].speed);
			drive_motor(i, cmd);
		} else if(control_type[i] == CURRENT || control_type[i] == PROPRIOCEPTIVE_PRIMITIVE)
		{
			if(motors[i].current_ma_setpoint >= 0)
			{
				float cmd = calculate_pid(&cur_params[i], motors[i].current_ma_setpoint, motors[i].current_mA);
				// Only allow forward drive

				if(cmd > 0)
				{
					drive_motor(i, cmd);
				}
				else
				{
					drive_motor(i, 0);
				}
			}
			else
			{
				// Flip direction of control and direction of setpoint
				float cmd = calculate_pid(&cur_params[i], -motors[i].current_ma_setpoint, motors[i].current_mA);
				// Only allow reverse drive
				if(cmd < 0)
				{
					drive_motor(i, 0);
				}
				else
				{
					drive_motor(i, -cmd);
				}
			}
		} else if (control_type[i] == DUTY)
		{
			float cmd = motors[i].duty * PWM_PERIOD;
			drive_motor(i, cmd);
		}// control type
	} // Loop through motors

	count++;

	/*if(count % 8000 == 0)
	{
		static bool dir = true;

		if(dir)
		{
			motors[1].ticks_setpoint = 4200;
			//motors[1].duty = 0.5;
		}
		else
		{
			motors[1].ticks_setpoint = 0;
			//motors[1].duty = -0.5;
		}
		dir = !dir;
	}*/
 }

 void motor_task_init(void)
 {
	 motion_primitive_init();
	 precalc_inverse_trig();

	 //motion_primitive_set_index(5);
	 control_type[0] = POSITION; // TODO default position
	 control_type[1] = POSITION;

	 leg.thigh_length_m = 0.055;
	 leg.calf_length_m = 0.065;
	 init_leg_precalcs(&leg);

	 ic_params.gain_current_per_torque = 15.0; // 1.5A stall at 0.196Nm torque
	 ic_params.c_eff_x = -0.2;
	 ic_params.k_eff_x = 600.0; // 2 lbs/ 1.5cm = ~600N/m
	 ic_params.c_eff_y = -0.2;
	 ic_params.k_eff_y = 600.0;
	 ic_params.gear_ratio = 150.0;

	 // position control params
	 pos_params[0].kp = 250.0;
	 pos_params[0].kd = -5000.0;
	 pos_params[0].cmd_max = 10000.0;
	 pos_params[0].cmd_min = -10000.0;
	 pos_params[0].speed_alpha = 0.98;
	 pos_params[0].integral_max = 8.0e6; // Accumulates difference of ticks at 4kHz, must be big
	 pos_params[0].integral_min = -8.0e6; // Accumulates difference of ticks at 4kHz, must be big
	 pos_params[0].ki = (0.4 * pos_params[0].cmd_max) / pos_params[0].integral_max; // Maximum is 40% of command

	 pos_params[1].kp = 250.0;
	 pos_params[1].kd = -5000.0;
	 pos_params[1].cmd_max = 10000.0;
	 pos_params[1].cmd_min = -10000.0;
	 pos_params[1].speed_alpha = 0.98;
	 pos_params[1].integral_max = 8.0e6; // Accumulates difference of ticks at 4kHz, must be big
	 pos_params[1].integral_min = -8.0e6; // Accumulates difference of ticks at 4kHz, must be big
	 pos_params[1].ki = (0.4 * pos_params[1].cmd_max) / pos_params[1].integral_max; // Maximum is 40% of command

	 // speed control params
	 speed_params[0].kp = 50.0;
	 speed_params[0].kd = -100.0;
	 speed_params[0].cmd_max = 10000.0;
	 speed_params[0].cmd_min = -10000.0;
	 speed_params[0].speed_alpha = 0.95;
	 speed_params[0].integral_max = 8.0e6; // Accumulates difference of ticks at 4kHz, must be big
	 speed_params[0].integral_min = -8.0e6; // Accumulates difference of ticks at 4kHz, must be big
	 speed_params[0].ki = 0.0;

 	 speed_params[1].kp = 50.0;
	 speed_params[1].kd = -100.0;
	 speed_params[1].cmd_max = 10000.0;
	 speed_params[1].cmd_min = -10000.0;
	 speed_params[1].speed_alpha = 0.95;
	 speed_params[1].integral_max = 8.0e6; // Accumulates difference of ticks at 4kHz, must be big
	 speed_params[1].integral_min = -8.0e6; // Accumulates difference of ticks at 4kHz, must be big
	 speed_params[1].ki = 0.0;

	 // current control params
	 cur_params[0].kp = 3.0;
	 cur_params[0].kd = 0.0;
	 cur_params[0].cmd_max = 10000.0;
	 cur_params[0].cmd_min = -10000.0;
	 cur_params[0].speed_alpha = 0.98;
	 cur_params[0].integral_max = 10000.0; // Accumulates difference of ticks at 4kHz, must be big
	 cur_params[0].integral_min = -10000.0; // Accumulates difference of ticks at 4kHz, must be big
	 cur_params[0].ki = (0.95 * cur_params[0].cmd_max) / cur_params[0].integral_max;

	 cur_params[1].kp = cur_params[0].kp;
	 cur_params[1].kd = cur_params[0].kd;
	 cur_params[1].cmd_max = cur_params[0].cmd_max;
	 cur_params[1].cmd_min = cur_params[0].cmd_min;
	 cur_params[1].speed_alpha = cur_params[0].speed_alpha;
	 cur_params[1].integral_max = cur_params[0].integral_max;
	 cur_params[1].integral_min = cur_params[0].integral_min;
	 cur_params[1].ki = cur_params[0].ki;

	 motors[0].reverse_direction = true;
	 motors[0].duty = 0.0;
	 motors[0].ticks_per_rev = TICKS_PER_REVOLUTION_DEFAULT;
	 motors[1].reverse_direction = false;
	 motors[1].duty = 0.0;
	 motors[1].ticks_per_rev = TICKS_PER_REVOLUTION_DEFAULT;

	 motor_tele_timer_handle = xTimerCreate(
	 		 "Tele",
	 		 pdMS_TO_TICKS(20),
	 		 pdTRUE,
	 		 NULL,
	 		 vMotorTelemetryTimerCallback);

	 if(motor_tele_timer_handle)
	 {
	 	 xTimerStart(motor_tele_timer_handle, 10);
	 }

	 // Enable PWM channels for lights and motor driving
	 TIM2->CR1 |= TIM_CR1_CEN;
	 TIM13->CR1 |= TIM_CR1_CEN;
	 TIM14->CR1 |= TIM_CR1_CEN;
	 TIM4->CR1 |= TIM_CR1_CEN; // Control loop timer

	 TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
	 TIM13->CCER |= (TIM_CCER_CC1E);
	 TIM14->CCER |= (TIM_CCER_CC1E);
	 TIM4->CCER |= (TIM_CCER_CC1E);

	 TIM4->DIER |= (TIM_DIER_CC1IE);

	 // Enable encoder timers
	 TIM1->CR1 |= TIM_CR1_CEN;
	 TIM3->CR1 |= TIM_CR1_CEN;

	  vTaskDelay(pdMS_TO_TICKS(2500));
 }

 void motor_task(void)
 {
	 vControlTimerCallback();
 }

 /*
 *	\brief Sets the motor control type
 *
 *	\param mode The controller mode
 *	\param index The motor index
 */
 void set_control_mode(CONTROL_TYPE mode, uint8_t index)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	control_type[index] = mode;
 }

 /*
 *	\brief Sets the motor desired position
 *
 *	\param rotations_deg The desired position
 *	\param index The motor index
 */
 void set_motor_position(float rotations_deg, uint8_t index)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	motors[index].desired_position_deg = rotations_deg;
	motors[index].ticks_setpoint = (int32_t) (motors[index].ticks_per_rev) * (rotations_deg / 360.0);
 }

 /*
 *	\brief Gets the motor current position
 *
 *	\param index The motor index
 */
 float get_motor_position(uint8_t index)
 {
	if(index >= NUMBER_MOTORS)
	{
		return 0.0;
	}
	motors[index].current_position_deg = ((float) motors[index].ticks_count / (float) motors[index].ticks_per_rev) * 360.0;
	return motors[index].current_position_deg;
 }

 /*
 *	\brief Sets the motor speed
 *
 *	\param speed The desired speed TODO units?
 *	\param index The motor index
 */
 void set_motor_speed(int32_t speed, uint8_t index)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	motors[index].speed_setpoint = speed;
 }

 /*
 *	\brief Sets the motor duty cycle
 *
 *	\param speed The duty [-1,1]
 *	\param index The motor index
 */
 void set_motor_duty(float duty, uint8_t index)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	if(duty > 1.0){duty = 1.0;}
	if(duty < -1.0){duty = -1.0;}

	motors[index].duty = duty;
 }

  /*
 *	\brief Sets various motor control parameters
 *
 *	\param index The motor index
 *  \param ctrl_index Which controller to set, 0-position, 1-speed, 2-current
 *  \param kp The proportional gain
 *  \param kd The derivative gain
 *  \param alpha The speed filter parameter
 *  \param cmd_min The minimum command scaled to [0-1] of pwm period
 *  \param cmd_max The maximum command scaled to [0-1] of pwm period
 */
 void set_control_params_kp_min_max(uint8_t index, uint8_t ctrl_index, float kp, float kd, float alpha, float cmd_min, float cmd_max)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	if(ctrl_index > 2)
	{
		return;
	}

	if(ctrl_index == 0)
	{
		pos_params[index].kp = kp;
		pos_params[index].kd = kd;
		pos_params[index].cmd_max = cmd_max * PWM_PERIOD;
		pos_params[index].cmd_min = -cmd_min * PWM_PERIOD;
		pos_params[index].speed_alpha = alpha;
	} else if(ctrl_index == 1)
	{
		speed_params[index].kp = kp;
		speed_params[index].kd = kd;
		speed_params[index].cmd_max = cmd_max * PWM_PERIOD;
		speed_params[index].cmd_min = -cmd_min * PWM_PERIOD;
		speed_params[index].speed_alpha = alpha;
	} else if(ctrl_index == 2)
	{
		cur_params[index].kp = kp;
		cur_params[index].kd = kd;
		cur_params[index].cmd_max = cmd_max * PWM_PERIOD;
		cur_params[index].cmd_min = -cmd_min * PWM_PERIOD;
		cur_params[index].speed_alpha = alpha;
	}
 }

 /*
 *	\brief Sets various motor control parameters
 *
 *	\param index The motor index
 *  \param ctrl_index Which controller to set, 0-position, 1-speed, 2-current
 *  \param ki The integral gain
 */
 void set_control_params_ki(uint8_t index, uint8_t ctrl_index, float ki)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	if(ctrl_index > 2)
	{
		return;
	}

	if(ctrl_index == 0)
	{
		pos_params[index].ki = ki;
	} else if(ctrl_index == 1)
	{
		speed_params[index].ki = ki;
	} else if(ctrl_index == 2)
	{
		cur_params[index].ki = ki;
	}
 }

 /*
 *	\brief Sets various motor control parameters
 *
 *	\param index The motor index
 *  \param ctrl_index Which controller to set, 0-position, 1-speed, 2-current
 *  \param int_max The maximum integral windup
 */
 void set_control_params_int_max(uint8_t index, uint8_t ctrl_index, float int_max)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	if(ctrl_index > 2)
	{
		return;
	}

	if(ctrl_index == 0)
	{
		pos_params[index].integral_max = int_max;
	} else if(ctrl_index == 1)
	{
		speed_params[index].integral_max = int_max;
	} else if(ctrl_index == 2)
	{
		cur_params[index].integral_max = int_max;
	}
 }

 /*
 *	\brief Sets various motor control parameters
 *
 *	\param index The motor index
 *  \param ctrl_index Which controller to set, 0-position, 1-speed, 2-current
 *  \param int_min The minimum integral windup
 */
 void set_control_params_int_min(uint8_t index, uint8_t ctrl_index, float int_min)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	if(ctrl_index > 2)
	{
		return;
	}

	if(ctrl_index == 0)
	{
		pos_params[index].integral_min = int_min;
	} else if(ctrl_index == 1)
	{
		speed_params[index].integral_min = int_min;
	} else if(ctrl_index == 2)
	{
		cur_params[index].integral_min = int_min;
	}
 }

 /*
 *	\brief Sets motor ticks per revolution
 *
 *	\param ticks_per_rev The number of quadrature counts per output shaft rotation
  *	\param index The motor index
 */
 void set_motor_ticks_per_rev(int32_t ticks_per_rev, uint8_t index)
 {
	if(index >= NUMBER_MOTORS)
	{
		return;
	}
	motors[index].ticks_per_rev = ticks_per_rev;
 }
