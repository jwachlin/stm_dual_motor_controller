/*
 * inverse_kinematics.c
 *
 * Created: 8/30/2020 2:27:29 PM
 *  Author: Jake
 */

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

 #include "inverse_kinematics.h"
 #include <stdint.h>
 #include <math.h>
 #include "fast_inverse_trig.h"
 #include "arm_math.h"

 void init_leg_precalcs(leg_ik_t * leg)
 {
	leg->alpha_1 = -(leg->calf_length_m*leg->calf_length_m + leg->thigh_length_m*leg->thigh_length_m);
	leg->alpha_2 = 1.0 / (2.0 * leg->calf_length_m * leg->thigh_length_m);
 }

 void calculate_ik(leg_ik_t * leg, pos_joint_space_t * joint_angles, const pos_cartesian_t pos)
 {
	joint_angles->knee_angle_rad = fast_acos(leg->alpha_2 * (pos.x*pos.x + pos.y*pos.y + leg->alpha_1) );
	joint_angles->thigh_angle_rad = fast_atan( pos.y / pos.x ) - fast_atan( (leg->calf_length_m * arm_sin_f32(joint_angles->knee_angle_rad)) / (leg->thigh_length_m + leg->calf_length_m * arm_cos_f32(joint_angles->knee_angle_rad)) );
 }

 void calculate_fk(leg_ik_t * leg, pos_cartesian_t * pos, const pos_joint_space_t joint_angles)
 {
	pos->x = leg->thigh_length_m * arm_cos_f32(joint_angles.thigh_angle_rad) + leg->calf_length_m * arm_cos_f32(joint_angles.thigh_angle_rad + joint_angles.knee_angle_rad);
	pos->y = leg->thigh_length_m * arm_sin_f32(joint_angles.thigh_angle_rad) + leg->calf_length_m * arm_sin_f32(joint_angles.thigh_angle_rad + joint_angles.knee_angle_rad);
 }

 void calculate_fk_and_jacobian(leg_ik_t * leg, pos_cartesian_t * pos, const pos_joint_space_t joint_angles, jacobian_t * j)
 {
	// Optimization, avoid calculating sin and cos functions twice
	float ct = arm_cos_f32(joint_angles.thigh_angle_rad);
	float st = arm_sin_f32(joint_angles.thigh_angle_rad);
	float ctk = arm_cos_f32(joint_angles.thigh_angle_rad + joint_angles.knee_angle_rad);
	float stk = arm_sin_f32(joint_angles.thigh_angle_rad + joint_angles.knee_angle_rad);

	pos->x = leg->thigh_length_m * ct + leg->calf_length_m * ctk;
	pos->y = leg->thigh_length_m * st + leg->calf_length_m * stk;

	j->j_01 = -leg->calf_length_m * stk;
	j->j_00 = -leg->thigh_length_m * st + j->j_01;
	j->j_11 = leg->calf_length_m * ctk;
	j->j_10 = leg->thigh_length_m * ct + j->j_11;

 }
