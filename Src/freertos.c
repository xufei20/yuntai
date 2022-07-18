/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "calibrate_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "led_flow_task.h"

#include <math.h>
#include "arm_math.h"
#include "odrive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId calibrate_tast_handle;
osThreadId detect_handle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
osThreadId CylinderControlHandle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId testHandle;
osThreadId HeartBeatHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void Odrive_HeatBeat(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}   

/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of HeartBeat */
  osThreadDef(HeartBeat, Odrive_HeatBeat, osPriorityLow, 0, 128);
  HeartBeatHandle = osThreadCreate(osThread(HeartBeat), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
    calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

    // osThreadDef(DETECT, detect_task, osPriorityNormal, 0, 256);
    // detect_handle = osThreadCreate(osThread(DETECT), NULL);

    // osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 512);
    // gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

    osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

    // osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
    // led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */

uint8_t Odrive_Motor_Error = 0;

volatile float shoot_speed;

extern void orive_set_speed(int speed);
extern UART_HandleTypeDef huart6;
extern volatile float target_angle;
void aRGB_led_show(uint32_t aRGB);

extern OdriveAxisSetState_t odrive_set_axis0;
extern OdriveAxisSetState_t odrive_set_axis1;

/* USER CODE END Header_test_task */
void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  // osDelay(12000);
	// target_angle = 27.09;
	// target_angle = 25.33;  
  // int odrv_speed = 0;
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    orive_set_speed(shoot_speed);
    
		static uint8_t rgb = 0;
		if(rgb == 0)
		{
			aRGB_led_show(0xFF00FF00);
			rgb = 1;
		}
		else if(rgb == 1)
		{
			aRGB_led_show(0xFF000000);
			rgb = 0;
		}
    // Board_Heartbeat(&hcan2, target_angle, shoot_speed);
    // Board_Heartbeat(&hcan2, fabsf(angle_in_deg[2]), odrive_get_axis0.encoder_vel_estimate, odrive_get_axis1.encoder_vel_estimate); // angle_in_deg->实时角度�??
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_Odrive_HeatBeat */
/**
* @brief Function implementing the HeartBeat thread.
* @param argument: Not used
* @retval None
*/

extern float angle_in_deg[3];
extern OdriveAxisGetState_t odrive_get_axis0;
extern OdriveAxisGetState_t odrive_get_axis1;

extern OdriveAxisSetState_t odrive_set_axis0;
extern OdriveAxisSetState_t odrive_set_axis1;

extern State_t odrive_axis0_state;
extern State_t odrive_axis1_state;

#define INSTALL_OFFSET_ANGLE 1.1f
HeartBeat_Box_T HeartBeat_Box;

extern void CAN_Send_Message(HeartBeat_Box_T * const pHeartBeat);

/* USER CODE END Header_Odrive_HeatBeat */
void Odrive_HeatBeat(void const * argument)
{
  /* USER CODE BEGIN Odrive_HeatBeat */
  /* Infinite loop */
  for(;;)
  {
    odrv_write_msg(AXIS_0, MSG_GET_ENCODER_ESTIMATES);
    odrv_write_msg(AXIS_1, MSG_GET_ENCODER_ESTIMATES);
    osDelay(20);
    
    // 检查电机错误
    odrv_write_msg(AXIS_0, MSG_GET_MOTOR_ERROR);
    odrv_write_msg(AXIS_1, MSG_GET_MOTOR_ERROR);
		
    if((odrive_get_axis0.motor_error !=  0) || (odrive_get_axis1.motor_error !=  0))
    {
			shoot_speed = 10;
			Odrive_Motor_Error = 1;
			
      odrv_write_msg(AXIS_0, MSG_CLEAR_ERRORS);
      odrv_write_msg(AXIS_1, MSG_CLEAR_ERRORS);

			odrv_write_msg(AXIS_0, MSG_RESET_ODRIVE);
			odrv_write_msg(AXIS_1, MSG_RESET_ODRIVE);
			osDelay(4000);
		
			Odrive_Motor_Error = 0;
    }

    HeartBeat_Box.value.angle = -angle_in_deg[2] - INSTALL_OFFSET_ANGLE;
    HeartBeat_Box.value.average_speed = ( - odrive_get_axis0.encoder_vel_estimate + \
																				    odrive_get_axis1.encoder_vel_estimate) / 2;
		
		CAN_Send_Message(&HeartBeat_Box);
  }
  /* USER CODE END Odrive_HeatBeat */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
