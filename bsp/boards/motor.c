#include "motor.h"
#include "main.h"
#include "CAN_receive.h"
//extern Motor motor;
//extern arm_pid_instance_f32 S_pos;
//extern arm_pid_instance_f32 S_vel;
//void angle_control(Motor *motor){
//	float angle_err;
//	float angle_out;
//	float speed_err;
//	int16_t I_out;
//	angle_err = motor->target_angle - motor->absolute_angle;
//	angle_out = arm_pid_f32(&S_pos, angle_err);
//	speed_err = angle_out - motor->speed;
//	I_out = (int16_t)(arm_pid_f32(&S_vel,speed_err));
//	I_out = (I_out > 10000) ? 10000 : (I_out < -10000) ? -10000 : I_out;
//	CAN_cmd_chassis(I_out, 0, 0, 0);
//}

