/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
#include "cmsis_os.h"

#include "detect_task.h"
#include "motor.h"
#include "odrive.h"
#include "main.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern Motor motor;
extern Motor Catcher;
// motor data read
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef gimbal_tx_message;
static uint8_t gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8];

/**
 * @brief          hal CAN fifo call back, receive motor data
 * @param[in]      hcan, the point to CAN handle
 * @retval         none
 */
/**
 * @brief          hal��CAN�ص�����,���յ������
 * @param[in]      hcan:CAN���ָ��
 * @retval         none
 */

extern volatile float target_angle;
extern volatile float shoot_speed;

extern void process_user_cmd(uint32_t id, Gim_data * rx_data);
Gim_data gim_data;

OdriveAxisGetState_t odrive_get_axis0;
OdriveAxisGetState_t odrive_get_axis1;

extern void aRGB_led_show(uint32_t aRGB);

extern HeartBeat_Box_T HeartBeat_Box;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan->Instance == hcan1.Instance)
  {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if(((rx_header.StdId & AXIS0_NODE_ID) == AXIS0_NODE_ID) || (rx_header.StdId & AXIS1_NODE_ID) == AXIS1_NODE_ID)
    {
      uint8_t first_word[4];
      uint8_t second_word[4];

      OdriveAxisGetState_t *odrive_get;

      if (rx_header.DLC == 8)                  // 不同数据长度，不同解码
      {
        memcpy(first_word, rx_data, 4);
        second_word[3] = rx_data[7];
        second_word[2] = rx_data[6];
        second_word[1] = rx_data[5];
        second_word[0] = rx_data[4];
      }
      else if (rx_header.DLC == 4)
      {
          memcpy(first_word, rx_data, 4);
      }

      if((rx_header.StdId & AXIS0_NODE_ID) == AXIS0_NODE_ID)  // 轴0来消息了
      {
        odrive_get = &odrive_get_axis0;
      }
      else if((rx_header.StdId & AXIS1_NODE_ID) == AXIS1_NODE_ID) // 轴1来消息了
      {
        odrive_get = & odrive_get_axis1;
      }

      if(rx_header.RTR == 0) // 远程帧，不带数据帧，相当于A给B发送一个指令，让B给A传送，A想要的数据
      {                      // 我传入的指令都是远程帧
        switch(rx_header.StdId & 0x1f)  // 取后5位，后5位为指令码
        {
            case (MSG_GET_ENCODER_COUNT):
              memcpy(&(odrive_get->encoder_shadow_count), first_word, sizeof(uint32_t));
              memcpy(&(odrive_get->encoder_cpr_count), second_word, sizeof(uint32_t));
              break;
            case (MSG_GET_VBUS_VOLTAGE):
              memcpy(&(odrive_get->vbus_voltage), first_word, sizeof(float));
              break;
            case (MSG_GET_ENCODER_ESTIMATES):
              memcpy(&(odrive_get->encoder_pos_estimate), first_word, sizeof(float));
              memcpy(&(odrive_get->encoder_vel_estimate), second_word, sizeof(float));
              break;
            case (MSG_GET_MOTOR_ERROR):
              memcpy(&(odrive_get->motor_error), first_word, sizeof(uint32_t));
            break;
            default:
              break;
        }
      }
    }
    else
    {
      switch (rx_header.StdId)
      {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        
        {
          uint8_t i = rx_header.StdId - CAN_3508_M1_ID;
          get_motor_measure(&motor_chassis[i], rx_data);

          motor.speed = motor_chassis[i].speed_rpm;
          if (motor_chassis[i].ecd - motor_chassis[i].last_ecd > 4096)
            motor_chassis[i].circle --;
          else if (motor_chassis[i].ecd - motor_chassis[i].last_ecd < -4096)
            motor_chassis[i].circle ++;

          motor.angle = motor_chassis[i].ecd * 360 / 8192.0;
          motor.circle = motor_chassis[i].circle;
          // already_angleָ����set_angle()֮����ת���ĽǶ�
          motor.absolute_angle = motor.angle + motor.circle * 360;
        }
        break;
				
				case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
				{
					uint8_t i = rx_header.StdId - CAN_3508_M1_ID;
          get_motor_measure(&motor_chassis[i], rx_data);
					Catcher.speed = motor_chassis[i].speed_rpm;
					
					if (motor_chassis[i].ecd - motor_chassis[i].last_ecd > 4096)
            motor_chassis[i].circle --;
          else if (motor_chassis[i].ecd - motor_chassis[i].last_ecd < -4096)
            motor_chassis[i].circle ++;
					
					Catcher.angle = motor_chassis[i].ecd * 360 / 8192.0;
          Catcher.circle = motor_chassis[i].circle;
          Catcher.absolute_angle = Catcher.angle + Catcher.circle * 360;
					
				}
				break;
				

        default:
          break;
      }
    }
  }
  else if (hcan->Instance == hcan2.Instance)
  {
		aRGB_led_show(0xFFFF00);
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, gim_data.data);
    
    // target_angle = gim_data.value.angle;
    // shoot_speed = gim_data.value.speed;
    process_user_cmd(rx_header.StdId, &gim_data);
  }
}

/**
 * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
 * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000]
 * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
 * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
 * @param[in]      rev: (0x208) reserve motor control current
 * @retval         none
 */
/**
 * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
 * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
 * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
 * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
 * @param[in]      rev: (0x208) ������������Ƶ���
 * @retval         none
 */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
  uint32_t send_mail_box;
  gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
  gimbal_tx_message.IDE = CAN_ID_STD;
  gimbal_tx_message.RTR = CAN_RTR_DATA;
  gimbal_tx_message.DLC = 0x08;
  gimbal_can_send_data[0] = (yaw >> 8);
  gimbal_can_send_data[1] = yaw;
  gimbal_can_send_data[2] = (pitch >> 8);
  gimbal_can_send_data[3] = pitch;
  gimbal_can_send_data[4] = (shoot >> 8);
  gimbal_can_send_data[5] = shoot;
  gimbal_can_send_data[6] = (rev >> 8);
  gimbal_can_send_data[7] = rev;
  HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
 * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
 * @param[in]      none
 * @retval         none
 */
void CAN_cmd_chassis_reset_ID(void)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = 0x700;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = 0;
  chassis_can_send_data[1] = 0;
  chassis_can_send_data[2] = 0;
  chassis_can_send_data[3] = 0;
  chassis_can_send_data[4] = 0;
  chassis_can_send_data[5] = 0;
  chassis_can_send_data[6] = 0;
  chassis_can_send_data[7] = 0;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
 * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384]
 * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384]
 * @retval         none
 */
/**
 * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
 * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
 * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
 * @retval         none
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  uint32_t send_mail_box;
  chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
  chassis_tx_message.IDE = CAN_ID_STD;
  chassis_tx_message.RTR = CAN_RTR_DATA;
  chassis_tx_message.DLC = 0x08;
  chassis_can_send_data[0] = motor1 >> 8;
  chassis_can_send_data[1] = motor1;
  chassis_can_send_data[2] = motor2 >> 8;
  chassis_can_send_data[3] = motor2;
  chassis_can_send_data[4] = motor3 >> 8;
  chassis_can_send_data[5] = motor3;
  chassis_can_send_data[6] = motor4 >> 8;
  chassis_can_send_data[7] = motor4;

  HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief          return the yaw 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          ����yaw 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
  return &motor_chassis[4];
}

/**
 * @brief          return the pitch 6020 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          ����pitch 6020�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
  return &motor_chassis[5];
}

/**
 * @brief          return the trigger 2006 motor data point
 * @param[in]      none
 * @retval         motor data point
 */
/**
 * @brief          ���ز������ 2006�������ָ��
 * @param[in]      none
 * @retval         �������ָ��
 */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
  return &motor_chassis[6];
}

/**
 * @brief          return the chassis 3508 motor data point
 * @param[in]      i: motor number,range [0,3]
 * @retval         motor data point
 */
/**
 * @brief          ���ص��̵�� 3508�������ָ��
 * @param[in]      i: ������,��Χ[0,3]
 * @retval         �������ָ��
 */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
  return &motor_chassis[(i & 0x03)];
}
