/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "shared_variables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* canOpen Node PHY settings & EPOS4 Communication object */
EPOS4 	epos4;
canOpen CO;

/* PID Controller for Steering motor */
PIDController pid;

/* DV basic structure */
DV 			   dv;
DV_Dynamics    dv_dynamics;
DV_Kinematics  dv_kinematics;
DV_Status 	   dv_status;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
BaseType_t xHigherPriorityTarget;
BaseType_t xHigherPriorityError;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* PID Settings */
float Kp 		= -4800;
float Ki 		= 0;
float Kd 		= 0;
float tau 		= 10;
float Ts 		= 0.01;
float limMin 	= -32500;
float limMax	= 32500;
float limMinInt = -32500;
float limMaxInt = 32500;

/* Position target as given from PC */
volatile float target_position;

/* RX buffer for USB Transactions */
uint8_t buffer[64];

/* CAN2USB TX Buffer */
char    RxSp[200];
uint8_t RxData[12];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USB full-duplex PHY communication functionalities */
uint8_t USB2CAN();

/* PC & EP0S4 status messages */
uint8_t DV_Status_Tx();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_CAN2_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
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
  RCC_OscInitStruct.PLL.PLLM = 4;
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

/* Manages USB Transactions between PC & Autonomous CAN bus */
uint8_t USB2CAN()
{
	/* 1 byte that indicates the message received from PC */
	dv.DV_Identifier = buffer[0];

	/* No Error from PC into transmitting data to P23! */
	switch(dv.DV_Identifier)
	{
	case(0x01):

		if(dv.DV_Status->dv_health == DV_DRIVING)
		{
			/* Position of the steering shaft, as given by the PC, in (mm) */
			target_position 			       = ((float)((int16_t)((buffer[1] << 8) | buffer[2])))/1024;
			dv.DV_Dynamics->steer_target_in_mm = target_position;

			dv.DV_Dynamics->steer_target_in_deg = 4.0933 * 66.0 * target_position;
			dv.DV_Dynamics->Steering_target     = (int8_t)(dv.DV_Dynamics->steer_target_in_deg);
//			/* Transforming the signal from (linear_mm) in the steering_shaft to motor degrees
//			 * Steering angle actual_logger : To be used from ADC */
//			target_position  			        = 4.0933 * 66.0 * target_position;
//			dv.DV_Dynamics->steer_target_in_deg = target_position;
//
//			/* From motor degrees to _encoder_inc */
//			target_position = target_position * ((2.0*4096.0)/360.0);

			/* Motor Torque for P23 DV commands:
			 * - Transmit the command bytes
			 * - Read Torque_Target to verify its within regions */
			dv.DV_Dynamics->Motor_torque_command_bytes[0] = buffer[3];
			dv.DV_Dynamics->Motor_torque_command_bytes[1] = buffer[4];

			dv.DV_Dynamics->Motor_torque_target
				= ((float)((int16_t)(((buffer[3] << 8) | buffer[4]))))/128;

			/* Brake Pressure Target */
			dv.DV_Dynamics->Brake_hydr_target = (uint8_t)buffer[5];

			/* Speed readings from PC */
			dv.DV_Dynamics->Speed_actual = (uint8_t)buffer[6];
			dv.DV_Dynamics->Speed_target = (uint8_t)buffer[7];

			xTaskNotifyFromISR(Target_Handle, 0x01, eSetValueWithOverwrite, &xHigherPriorityTarget);
			portYIELD_FROM_ISR(xHigherPriorityTarget);
		}
		else
		{
			target_position 					= 0;
			dv.DV_Dynamics->steer_target_in_mm  = 0;
			dv.DV_Dynamics->steer_target_in_deg = 0;
			dv.DV_Dynamics->Steering_target     = 0;

			dv.DV_Dynamics->Motor_torque_command_bytes[0] = 0;
			dv.DV_Dynamics->Motor_torque_command_bytes[1] = 0;
			dv.DV_Dynamics->Motor_torque_target = 0;

			dv.DV_Dynamics->Brake_hydr_target   = 0;

			dv.DV_Dynamics->Speed_target 		= 0;
		}
		return true;

	case(0x02):
		/* PC returns data concerning P23 Acceleration/Gyroscope sensing */
		dv.DV_Kinematics->acc_lat 	= ((float)((int16_t)(buffer[1] << 8 | buffer[2])))/512;
		dv.DV_Kinematics->acc_long 	= ((float)((int16_t)(buffer[3] << 8 | buffer[4])))/512;
		dv.DV_Kinematics->yaw_rate  = ((float)((int16_t)(buffer[5] << 8 | buffer[6])))/128;
		return true;

	case(0x03):
		/* Reads status "nibble" from DV PC */
		dv.DV_Status->dv_health	= (buffer[1] & 0x0F);

		/* DV error Indication */
		if(dv.DV_Status->dv_health == 0x0F)
		{
			xTaskNotifyFromISR(CAN2USB_Error_Handle, 0x03, eSetValueWithOverwrite, &xHigherPriorityError);
			portYIELD_FROM_ISR(xHigherPriorityError);
		}

		// We will add more about LapCounter & more...
		dv.DV_Status->Lap_counter 		 = (buffer[1] & 0xF0) >> 4;
		dv.DV_Status->Cones_count_actual =  buffer[2];
		dv.DV_Status->Cones_count_all    = (uint16_t)((buffer[3] << 8) | buffer[4]);

		/* Hardware/Software Interface & PC Temperature */
		dv.DV_Status->hardware_flags = buffer[5];
		dv.DV_Status->software_flags = buffer[6];
		dv.DV_Status->pc_temperature = buffer[7];	// in C

		xTaskNotifyFromISR(Target_Handle, 0x03, eSetValueWithOverwrite, &xHigherPriorityTarget);
		portYIELD_FROM_ISR(xHigherPriorityTarget);
		return true;

	case(0x04):
		/* PC returns the mission has received */
		dv.DV_Status->dv_says_mission =  buffer[1] & 0x7F;
		dv.DV_Status->dv_says_lock	  = (buffer[1] & 0x80) >> 7;
		return true;
	}
	return false;
}


/* Transmit PC Status flags */
uint8_t DV_Status_Tx()
{
	/* Hardware/Software flags & PC Temperature:
	 * CAN Frame needed for P23Telemetry */
	dv.can_tx[0] = dv.DV_Status->pc_temperature;
	dv.can_tx[1] = dv.DV_Status->hardware_flags;
	dv.can_tx[2] |= dv.DV_Status->software_flags;
	dv.can_tx[3] = (uint8_t)(epos4.ep0s4_temp);
	dv.can_tx[4] = epos4.Ep0s4_Error;
	dv.can_tx[5] = 0;
	dv.can_tx[6] = 0;
	dv.can_tx[7] = 0;

	if(AutoCAN_Tx(&dv, CAN2USB_ID2, 4, dv.can_tx) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
