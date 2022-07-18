/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_delay.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "calibrate_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "motor.h"

#include "math.h"
// #include "motor.h"

#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern void aRGB_led_show(uint32_t aRGB);
extern arm_pid_instance_f32 S_pos, S_vel, S_ang;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define MAX_I_OUT  10000

arm_pid_instance_f32 angle_pid = {
  .Kp = 15.0,
  .Ki = 0.001,
  .Kd = 10.0
};
arm_pid_instance_f32 speed_pid = {
  .Kp = 10.0,
  .Ki = 0.001,
  .Kd = 0.5
};

void speed_control(Motor * motor){
	float speed_err;
	int I_out;
	
	speed_err = motor->target_speed - motor->speed;
	I_out = arm_pid_f32(&speed_pid, speed_err);
	I_out = (I_out > 10000) ? 10000 : (I_out < -10000) ? -10000 : I_out;
	CAN_cmd_gimbal(I_out, 0, 0, 0);
}


void angle_control(Motor * motor){
  float angle_err;
  float speed_out;
  float speed_err;
  int32_t I_out;

  angle_err = (motor->target_angle - motor->absolute_angle);
  speed_out = arm_pid_f32(&angle_pid, angle_err); // 输出期望速度

  speed_out = (speed_out > 1300) ? 1300 : (speed_out < -1300) ? -1300 : speed_out;

  speed_err = speed_out - motor->speed;
  I_out = (int32_t)(arm_pid_f32(&speed_pid,speed_err));
	// I_out = (I_out >= 0.0f ? 1 : -1) * 3500.0f * sqrtf(fabs(I_out / 3500.0f));
  I_out = (I_out > MAX_I_OUT) ? MAX_I_OUT : (I_out < -MAX_I_OUT) ? -MAX_I_OUT : I_out;

	CAN_cmd_gimbal(I_out, 0, 0, 0);
}


Motor Catcher = {
  .Base_Angle= 0,
  .Tar_Angle = 160
};
static uint8_t is_catcher_Init = 0;
static float last_angle;

extern void yuntai_pid(void);
extern float angle_in_deg[];
static uint8_t is_yuntai_ready_for_work = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  if (htim->Instance == htim7.Instance)
  {
    /* 云台3508位控部分 */
    if (is_yuntai_ready_for_work == 0 && angle_in_deg[2] < -2.f){
      is_yuntai_ready_for_work = 1;
    }else if(is_yuntai_ready_for_work == 1){
      yuntai_pid();
    }
		
    /* 推杆2006位控部分 */
		if(is_catcher_Init != 1){
			static uint16_t i = 0;
			Catcher.target_speed = -1000;
			speed_control(&Catcher);
			if(i < 4000)
			{
				i++;
			}
			else
			{
				if(fabsf(Catcher.speed) < 10.f){
          is_catcher_Init = 1;
          Catcher.target_speed = 0;
          Catcher.Base_Angle = Catcher.absolute_angle;
				}
			}
		}else {
			Catcher.target_angle = Catcher.Tar_Angle * 36 + Catcher.Base_Angle; // 小齿轮分度圆直径
			angle_control(&Catcher);
		}
  }
}


union odrive_speed_cmd
{
  uint8_t data[8];
  float output[2];
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void orive_set_speed(int speed);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM7_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_init();
  delay_init();
  cali_param_init();
  remote_control_init();
  arm_pid_init_f32(&S_pos, 1);
  arm_pid_init_f32(&S_vel, 1);
  arm_pid_init_f32(&S_ang, 1);

  arm_pid_init_f32(&angle_pid, 1);
  arm_pid_init_f32(&speed_pid, 1);
  // aRGB_led_show(0xFF00FF00);
  HAL_Delay(200);
  HAL_TIM_Base_Start_IT(&htim7);
	
	// HAL_TIM_Base_Start_IT(&htim1);
//	HAL_TIM_RegisterCallback(&htim1, HAL_TIM_PERIOD_ELAPSED_CB_ID, Yuntai_CatcherCallBack);
	
	
  // orive_set_speed(20);
  // HAL_UART_Transmit(&huart3, (uint8_t *)"hello\r\n", 7, 100);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
