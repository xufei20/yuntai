#include "stm32f4xx_hal.h"
#include "CAN_receive.h"
#include "odrive.h"
#include "motor.h"

typedef enum {
    FAILURE = 0,
    SET_ANGLE_SPEED = 1,
    SET_ANGLE = 2,
    SET_SPEED = 3,
    SET_LENGTH = 4,
		RESET_ODRIVE = 5,
}Gimbal_commmond;

extern volatile float target_angle;
extern volatile float shoot_speed;

typedef union 
{
    uint8_t data[8];
    float value[2];
}cmd_data_s;


extern void aRGB_led_show(uint32_t aRGB);
uint32_t argb = 0xff000000;
extern HeartBeat_Box_T HeartBeat_Box;
extern Motor Catcher;
void CAN_Send_Message(HeartBeat_Box_T * const pHeartBeat);

void process_user_cmd(uint32_t id,  Gim_data * rx_data)
{
    switch(id)
    {
        case FAILURE:
            argb = 0xffffffff;
            break;
        case SET_ANGLE_SPEED:
            argb = 0xffff0000;
            target_angle = rx_data->value.angle;
            shoot_speed = rx_data->value.speed;
            break;
        case SET_ANGLE:
            argb = 0xff00ff00;
            target_angle = rx_data->value.angle;
						break;
        case SET_SPEED:
            // argb = 0xff0000ff;
            shoot_speed = rx_data->value.speed;
						break;
				case SET_LENGTH:
            argb = 0xff0000ff;
            Catcher.Tar_Angle = rx_data->value.length;
            break;
				case RESET_ODRIVE:
						shoot_speed = 10;
						odrv_write_msg(AXIS_0, MSG_RESET_ODRIVE);
			      odrv_write_msg(AXIS_1, MSG_RESET_ODRIVE);
				    break;
        default:
            break;
    }
    aRGB_led_show( argb );
}


extern CAN_HandleTypeDef hcan2;

CAN_TxHeaderTypeDef HeartBeat_Send_Data;

void CAN_Send_Message(HeartBeat_Box_T * const pHeartBeat)
{
  uint32_t send_mail_box;
  HeartBeat_Send_Data.StdId = 0x250; // check
  HeartBeat_Send_Data.IDE = CAN_ID_STD;
  HeartBeat_Send_Data.RTR = CAN_RTR_DATA;
  HeartBeat_Send_Data.DLC = 0x08;

  // chassis_can_send_data[0] = motor1 >> 8;
  // chassis_can_send_data[1] = motor1;
  // chassis_can_send_data[2] = motor2 >> 8;
  // chassis_can_send_data[3] = motor2;
  // chassis_can_send_data[4] = motor3 >> 8;
  // chassis_can_send_data[5] = motor3;
  // chassis_can_send_data[6] = motor4 >> 8;
  // chassis_can_send_data[7] = motor4;

	HAL_CAN_AddTxMessage(&hcan2, &HeartBeat_Send_Data, HeartBeat_Box.data, &send_mail_box);
}


