/*
 * inverse_kinematics.h
 *
 * Created: 8/30/2020 2:27:14 PM
 *  Author: Jake
 */


#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

typedef struct
{
	float thigh_length_m;
	float calf_length_m;
	float alpha_1;
	float alpha_2;
} leg_ik_t;

typedef struct
{
	float x;
	float y;
} pos_cartesian_t;

typedef struct
{
	float thigh_angle_rad;
	float knee_angle_rad;
} pos_joint_space_t;

typedef struct
{
	float j_00;
	float j_01;
	float j_10;
	float j_11;
} jacobian_t;

void init_leg_precalcs(leg_ik_t * leg);
void calculate_ik(leg_ik_t * leg, pos_joint_space_t * joint_angles, const pos_cartesian_t pos);
void calculate_fk(leg_ik_t * leg, pos_cartesian_t * pos, const pos_joint_space_t joint_angles);
void calculate_fk_and_jacobian(leg_ik_t * leg, pos_cartesian_t * pos, const pos_joint_space_t joint_angles, jacobian_t * j);

#endif /* INVERSE_KINEMATICS_H_ */
