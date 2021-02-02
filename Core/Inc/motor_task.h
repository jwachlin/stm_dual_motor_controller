/*
 * motor_task.h
 *
 *  Created on: Jan 4, 2021
 *      Author: jake
 */

#ifndef INC_MOTOR_TASK_H_
#define INC_MOTOR_TASK_H_


#define NUMBER_MOTORS						(2)
#define PWM_PERIOD							(10000)
#define TICKS_PER_REVOLUTION_DEFAULT		(4200.0)		// 7 ppr, 28 cpr and 150:1 gearbox

typedef enum
{
	POSITION,
	SPEED,
	CURRENT,
	DUTY,
	PRIMITIVE,
	PROPRIOCEPTIVE_PRIMITIVE
} CONTROL_TYPE;

typedef struct
{
	float current_position_deg;
	float desired_position_deg;
	float rate_dps;
	float duty;
	int32_t speed;		// deg/s
	int32_t speed_setpoint; // deg/s
	int32_t current_mA;
	int32_t current_ma_setpoint;
	int32_t ticks_count;
	int32_t ticks_setpoint;
	int32_t ticks_per_rev;
	bool reverse_direction;
} motor_t;

void motor_task_init(void);
void motor_task(void);

void set_control_mode(CONTROL_TYPE mode, uint8_t index);
void set_motor_position(float rotations_deg, uint8_t index);
float get_motor_position(uint8_t index);
void set_motor_speed(int32_t speed, uint8_t index);
void set_motor_duty(float duty, uint8_t index);
void set_control_params_kp_min_max(uint8_t index, uint8_t ctrl_index, float kp, float kd, float alpha, float cmd_min, float cmd_max);
void set_control_params_ki(uint8_t index, uint8_t ctrl_index, float ki);
void set_control_params_int_max(uint8_t index, uint8_t ctrl_index, float int_max);
void set_control_params_int_min(uint8_t index, uint8_t ctrl_index, float int_min);
void set_motor_ticks_per_rev(int32_t ticks_per_rev, uint8_t index);

#endif /* INC_MOTOR_TASK_H_ */
