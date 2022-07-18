#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "usart.h"
#include "odrive.h"

#define UART 0

#if UART
void orive_set_speed(int speed)
{
  static uint8_t buff[50];
  if (speed < 0)
  {
    return;
  }

  int ret = sprintf((char *)buff, "v 0 %d\r\n", -speed);
  HAL_UART_Transmit(&huart6, buff, ret, 1000);
  ret = sprintf((char *)buff, "v 1 %d\r\n", speed);
  HAL_UART_Transmit(&huart6, buff, ret, 1000);
}

#else

OdriveAxisSetState_t odrive_set_axis0;
OdriveAxisSetState_t odrive_set_axis1;

State_t odrive_axis0_state;
State_t odrive_axis1_state;

void orive_set_speed(int speed) // TODO: check
{
  if(speed < 0)
    return;
  
  odrive_set_axis0.input_vel = -speed; 
  odrive_set_axis1.input_vel = speed; // TODO: check the rotating direction of the motor

  odrv_write_msg(AXIS_0, MSG_SET_INPUT_VEL);
	// delay
  odrv_write_msg(AXIS_1, MSG_SET_INPUT_VEL);
}

uint8_t odrv_write_msg(Axis_t axis, Odrive_Commond cmd)
{
  extern CAN_HandleTypeDef hcan1;

  CAN_TxHeaderTypeDef header;
  uint32_t send_mail_box;
  OdriveAxisSetState_t *odrive_set;

  uint8_t data[8] = {0};
  float_to_uint8_t pack;   // 共同体
  uint8_t tmp_word[4];
  header.IDE = CAN_ID_STD;
  if(axis == AXIS_0)	
  {
    header.StdId = AXIS0_NODE_ID  + cmd;
    odrive_set = &odrive_set_axis0;
  }
  else if(axis == AXIS_1)
  {
    header.StdId = AXIS1_NODE_ID  + cmd;
    odrive_set = &odrive_set_axis1;
  }
  else
  {
    return 0;    // 0 —> Error
  }   

  switch(cmd)
  {
    case MSG_ODRIVE_ESTOP:
        /* TODO: Implement */
        break;
    case MSG_GET_MOTOR_ERROR:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_GET_ENCODER_ERROR:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_GET_SENSORLESS_ERROR:
        /* TODO: Implement */
        break;
    case MSG_SET_AXIS_NODE_ID:
        /* TODO: Implement */
        break;
    case MSG_SET_AXIS_REQUESTED_STATE:
        memcpy(data, &(odrive_set->requested_state), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_SET_AXIS_STARTUP_CONFIG:
        /* TODO: Implement */
        break;
    case MSG_GET_ENCODER_ESTIMATES:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_GET_ENCODER_COUNT:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_SET_CONTROLLER_MODES:
        data[0] = odrive_set->control_mode;
        data[4] = odrive_set->input_mode;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 8;
        break;
    case MSG_SET_INPUT_POS:
        memcpy(data, &(odrive_set->input_pos), 4);
        data[4] = odrive_set->vel_ff & 0x00FF;
        data[5] = odrive_set->vel_ff >> 8;
        data[6] = odrive_set->current_ff & 0x00FF;
        data[7] = odrive_set->current_ff >> 8;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 8;
        break;
    case MSG_SET_INPUT_VEL:
        pack.value[0] = odrive_set->input_vel; // odrive_set_axis0.input_vel;
				// pack.value[0] = 10;		
        pack.value[1] = odrive_set->torque_vel; // odrive_set_axis0.torque_vel;
        header.RTR = CAN_RTR_DATA;
        header.DLC = 0x08;
        break;
    case MSG_SET_INPUT_CURRENT:
        memcpy(data, &(odrive_set->input_current), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_SET_VEL_LIMIT:
        memcpy(data, &(odrive_set->vel_limit), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_START_ANTICOGGING:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_SET_TRAJ_VEL_LIMIT:
        memcpy(data, &(odrive_set->traj_vel_limit), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_SET_TRAJ_ACCEL_LIMITS:
        memcpy(data, &(odrive_set->traj_accel_limit), 4);
        memcpy(tmp_word, &(odrive_set->traj_decel_limit), 4);
        data[4] = tmp_word[0];
        data[5] = tmp_word[1];
        data[6] = tmp_word[2];
        data[7] = tmp_word[3];
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_SET_TRAJ_A_PER_CSS:
        memcpy(data, &(odrive_set->traj_a_per_css), 4);
        header.RTR = CAN_RTR_DATA;
        header.DLC = 4;
        break;
    case MSG_GET_IQ:
        /* TODO: Implement */
        break;
    case MSG_GET_SENSORLESS_ESTIMATES:
        /* TODO: Implement */
        break;
    case MSG_RESET_ODRIVE:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_GET_VBUS_VOLTAGE:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_CLEAR_ERRORS:
        header.RTR = CAN_RTR_REMOTE;
        header.DLC = 0;
        break;
    case MSG_CO_HEARTBEAT_CMD:
        /* TODO: Implement */
        break;
    default:
        break;
  }
  if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
  {
    HAL_CAN_AddTxMessage(&hcan1, &header, pack.raw, &send_mail_box);
    return 1;
  }
  return 0;
} 





#endif

// CAN CONTROL BACKUP

// uint32_t send_mail_box;
// CAN_TxHeaderTypeDef gimbal_tx_message;
// union odrive_speed_cmd cmd;
// gimbal_tx_message.StdId = 1 << 5 |  0x00D;
// gimbal_tx_message.IDE = CAN_ID_STD;
// gimbal_tx_message.RTR = CAN_RTR_DATA;
// gimbal_tx_message.DLC = 0x08;
// cmd.output[0] = speed;
// cmd.output[1] = 0.0;

// if (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_READY)
// {
//   HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, cmd.data, &send_mail_box);
// }
// gimbal_tx_message.StdId = 2 << 5 |  0x00D;
// gimbal_tx_message.IDE = CAN_ID_STD;
// gimbal_tx_message.RTR = CAN_RTR_DATA;
// gimbal_tx_message.DLC = 0x08;
// if (HAL_CAN_GetState(&hcan2) == HAL_CAN_STATE_READY)
// {
//   HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, cmd.data, &send_mail_box);
// }
