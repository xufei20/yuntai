#include "arm_math.h"
#include "motor.h"
#include "bsp_led.h"
#include "bsp_can.h"

#include "CAN_receive.h"


/* Private define ------------------------------------------------------------*/
#define RESOLUTION 2
// TODO: after every installation , need to check this value
#define INSTALL_OFFSET_ANGLE 1.1f

/* Public variables ---------------------------------------------------------*/
volatile float target_angle = 0.0;

/* Private variables ---------------------------------------------------------*/
Motor motor = {.id = 0,
               .target_angle = 0.f,
               .max_acc = 40000.f,
               .max_speed = 600.f,
               .max_current = 10000.f};
/*
arm_pid_instance_f32 S_pos = {
    .Kp = 1.0f,
    .Ki = 0.004f,
    .Kd = 0.0,
};
arm_pid_instance_f32 S_vel = {
    .Kp = 30.f,
    .Ki = 1.5f,
    .Kd = 0.f,
};
// TODO: need check this pid value
arm_pid_instance_f32 S_ang = {
    .Kp = 32.0f,
    .Ki = 0.0f,
    .Kd = 0.f,
};
 */
							 
arm_pid_instance_f32 S_pos = {
    .Kp = 1.0f,
    .Ki = 0.004f,
    .Kd = 0.0,
};
arm_pid_instance_f32 S_vel = {
    .Kp = 6.f,
    .Ki = 0.5f,
    .Kd = 0.f,
};
arm_pid_instance_f32 S_ang = {
    .Kp = 32.0f,
    .Ki = 0.0f,
    .Kd = 0.f,
};

/* External variables ---------------------------------------------------------*/
extern float angle_in_deg[];

float angle_err;
float speed_ref;
float speed_err;
float iq_ref;
float output_current;


void yuntai_pid(void)
{ // 1ms
  // static uint32_t argb = 0xff00ff00;
//   Motor *motor = &motor;
  // ! AHRS angle pid
  angle_err = target_angle + INSTALL_OFFSET_ANGLE + angle_in_deg[2];

#if RESOLUTION == 0
    if (fabsf(angle_err) < 0.5f)
    {
      arm_pid_reset_f32(&S_ang);
      arm_pid_reset_f32(&S_vel);
    }
    float out_angle = arm_pid_f32(&S_ang, angle_err);

    // ! speed -> current pid
    float speed_err = fabsf(out_angle) > 30.0f ? 
                                    (out_angle >= 0 ? 1 : -1) * motor.max_acc / 1000 : 
                                    out_angle * motor.max_acc / (30 * 1000) ;
    if (fabsf(motor.speed) >= motor.max_speed)
    {
      speed_err = motor.max_speed - motor.speed;
    }

    float output_current = arm_pid_f32(&S_vel, speed_err);
    if (output_current > motor.max_current)
    {
      output_current = motor.max_current;
    }
    else if (output_current < -motor.max_current)
    {
      output_current = -motor.max_current;
    }

    output_current = (output_current >= 0.0f ? 1 : -1) * 3000.0f * sqrtf(fabs(output_current / 3000.0f));
#elif RESOLUTION == 1
    // ! speed -> current pid
    float target_speed = fabsf(angle_err) > 30.0f ? 
                                    (angle_err >= 0.0f ? 1 : -1) * motor.max_acc / 1000 : 
                                    angle_err * motor.max_acc / (30 * 1000) ;
    float speed_err = target_speed - motor.speed;
    if (motor.speed > motor.max_speed){ // speed limit
      speed_err = motor.max_speed - motor.speed;
    }else if (motor.speed < -motor.max_speed){
      speed_err = -motor.max_speed - motor.speed;
    }

    float output_current = arm_pid_f32(&S_vel, speed_err);
    if (output_current > motor.max_current)
    {
      output_current = motor.max_current;
    }
    else if (output_current < -motor.max_current)
    {
      output_current = -motor.max_current;
    }

    output_current = (output_current >= 0.0f ? 1 : -1) * 3000.0f * sqrtf(fabs(output_current / 3000.0f));
#elif RESOLUTION == 2
    // classic mode
    speed_ref = arm_pid_f32(&S_ang, angle_err);
    speed_err = speed_ref - motor.speed;
    iq_ref    = arm_pid_f32(&S_vel, speed_err);
		output_current = - 3800 * cosf((20 + angle_in_deg[2]) * PI / 180) + iq_ref; 
    // output_current = (iq_ref >= 0.0f ? 1 : -1) * 3500.0f * sqrtf(fabs(iq_ref / 3500.0f));
#endif
  CAN_cmd_chassis(output_current, 0, 0, 0);

}






  // ! encoder -> speed pid
  // motor.target_angle = motor.absolute_angle - angle_out;
  // float delta_angle = motor.target_angle - motor.absolute_angle;
  // if (fabsf(delta_angle) < 15.0f)
  // {
  //   arm_pid_reset_f32(&S_pos);
  //   arm_pid_reset_f32(&S_vel);
  // }
  // float out_angle = arm_pid_f32(&S_pos, delta_angle);
