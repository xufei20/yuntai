#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"
#include "bsp_can.h"
#include "arm_math.h"
typedef struct{
    uint8_t id;      
    float32_t speed;
    float32_t target_speed;
    float32_t angle;
    float32_t target_angle;
    float32_t already_angle;
		int16_t circle;
		float32_t Iout;
    float32_t absolute_angle;
		uint16_t ecd;
    //控制限幅
    float32_t max_speed;
    float32_t max_acc;
    float32_t max_current;

    // Catcher 相关
    float Base_Angle;
    int16_t Tar_Angle;
}Motor;

#endif
