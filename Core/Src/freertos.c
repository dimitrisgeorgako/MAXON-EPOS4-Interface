/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "shared_variables.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEERING_ZERO	0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* CAN2USB Semaphore for handling CAN_RX ISR */
SemaphoreHandle_t xSemaphoreCAN2USB;
BaseType_t xHigherPriorityCAN2USBTaskWoken = pdFALSE;

/* Main Error Handler that decides SD opening and notifies with CAN error message */
TaskHandle_t CAN2USB_Error_Handle;

/* CAN2USB USB Transactions each time we receive the needed CAN Frame */
TaskHandle_t CAN2USB_Tx_Handle;

/* Mission Selection Handler */
TaskHandle_t MissionSelect_Handle;
BaseType_t xHigherPriorityMission = pdFALSE;

/* EPOS4 Control Handler */
TaskHandle_t EP0S4_Control_Handle;
TaskHandle_t Target_Handle;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t open_sd_int = false;

/* CANOpen SYNC variables & EP0S4 control variables */
TickType_t sync_time, sync_tick;
TickType_t control_time, control_tick;
TickType_t pc_status_time, pc_status_tick;

/* USB flag every time data are received */
extern uint8_t received;

/* Linear feedback */
int16_t linear_feedback;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for timCAN_DataLogger */
osTimerId_t timCAN_DataLoggerHandle;
const osTimerAttr_t timCAN_DataLogger_attributes = {
  .name = "timCAN_DataLogger"
};
/* Definitions for timCO_SYNC */
osTimerId_t timCO_SYNCHandle;
const osTimerAttr_t timCO_SYNC_attributes = {
  .name = "timCO_SYNC"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* */
extern uint8_t DV_Status_Tx();

/* CAN2USB Error Handler */
void CAN2USB_Error_Handler();

/* CAN2USB USB Transactions each time we receive the needed CAN Frame */
void CAN2USB_Tx_Handler();

/* Mission Selection Handler */
void MissionSelection_Handler();

/* 	CANOpen Communication with EPOS4 for BLDC Control & Target Commands given from DV algorithms */
void EP0S4_Control_Handler();
void Target_Handler();

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void timDataLogger_Callback(void *argument);
void timSYNC_Callback(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  xSemaphoreCAN2USB = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of timCAN_DataLogger */
  timCAN_DataLoggerHandle = osTimerNew(timDataLogger_Callback, osTimerPeriodic, NULL, &timCAN_DataLogger_attributes);

  /* creation of timCO_SYNC */
  timCO_SYNCHandle = osTimerNew(timSYNC_Callback, osTimerPeriodic, NULL, &timCO_SYNC_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(timCO_SYNCHandle,	    50);
  osTimerStart(timCAN_DataLoggerHandle, 350);	/* DataLogger messages Timer */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* CAN2USB Error Handler */
  xTaskCreate(CAN2USB_Error_Handler, 	   "CAN2USB_Error_Handle Task",  	     128, NULL, 12, &CAN2USB_Error_Handle);

  /* CAN2USB USB Transactions each time we receive the needed CAN Frame */
  xTaskCreate(CAN2USB_Tx_Handler, 		   "CAN2USB_Transmit_Handle Task",       128, NULL, 12, &CAN2USB_Tx_Handle);

  /* Mission Selection Choice Handler */
  xTaskCreate(MissionSelection_Handler,    "MissionSelection_Handle Task",       128, NULL, 12, &MissionSelect_Handle);

  /* EPOS4 Control Task Initialization */
  xTaskCreate(EP0S4_Control_Handler, 		"EP0S4_Control_Handle Task",  		 128, NULL,  7, &EP0S4_Control_Handle);
  xTaskCreate(Target_Handler, 				"Target_Handle Task",  				 128, NULL, 12, &Target_Handle);

  /* Initialize DV settings */
  DV_Init(&dv, &dv_dynamics, &dv_kinematics, &dv_status, &hcan2, &RxHeader2, &TxHeader2, &canMailbox);

  /* CanOpen & EPOS4 Control Initialization */
  EPOS4_Init(&epos4, &CO, &hcan1, &RxHeader1, &TxHeader1, &canMailbox, 1);

  /* Start ADC conversions with DMA */
  Enable_ADC_Conversions(&hadc1);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  /* Reset USB Acknowledgments */
	  received = false;

	  /* Reset EP0S4 variables always */
	  if(!epos4.motor_enable)
	  {
		  dv.DV_Dynamics->steer_target_in_mm  = 0;
		  dv.DV_Dynamics->steer_target_in_deg = 0;
	  }

	  /* Send every 0.5sec the PC Status information of the DriverLess PC (5Hz)*/
	  if(xTaskGetTickCount() - pc_status_tick > 490)
	  {
		  pc_status_time = xTaskGetTickCount() - pc_status_tick;
		  pc_status_tick = xTaskGetTickCount();
		  DV_Status_Tx();
	  }

	  /* Open SD intention, due to EP0S4 or PC error:
	   * Sending PC_Error message either way, if SD is closed */
	  if(open_sd_int && (!dv.DV_Status->sd_is_open))
	  {
		  uint8_t sd_open[8];
		  sd_open[0] = 0x0F;	// Byte that indicates PC_Error

		  if(AutoCAN_Tx(&dv, CAN2USB_ID1, 8, sd_open) != HAL_OK);
	  }

	  osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* timDataLogger_Callback function */
void timDataLogger_Callback(void *argument)
{
  /* USER CODE BEGIN timDataLogger_Callback */
	HAL_GPIO_TogglePin(CANOPEN_GREEN_GPIO_Port, CANOPEN_GREEN_Pin);

//	if(DataLogger_TX(&dv) == HAL_OK);

  /* USER CODE END timDataLogger_Callback */
}

/* timSYNC_Callback function */
void timSYNC_Callback(void *argument)
{
  /* USER CODE BEGIN timSYNC_Callback */
	char dummySp[200];
	uint8_t data[2];

	/* TimeStamp for SYNC object */
	sync_time = xTaskGetTickCount() - sync_tick;
	sync_tick = xTaskGetTickCount();

	/* In case of AS Emergency, zero-out steering shaft, then disable EPOS4
	 * while AS Emergency is transmitted */
	if(dv.DV_Status->AS_state == AS_EMERGENCY)
	{
		dv.DV_Dynamics->Motor_torque_target = 0;
		dv.DV_Dynamics->Brake_hydr_target   = 0;
		Target_Message(&dv);
	}

	/* In case of wrong mission or mission NOT_SELECTED, don't take into account the message */
	if((dv.DV_Status->mission_selected == NOT_SELECTED) | (dv.DV_Status->mission_selected != dv.DV_Status->dv_says_mission))
	{
		dv.DV_Status->dv_says_mission = NOT_SELECTED;
	}

	/* Configure SFM for EPOS4 */
	EPOS4_app_process(&CO, &epos4);

//	/* Check Steering_Status, if EP0S4 is disabled or not */
//	if(!epos4.motor_enable)
//	{
//		dv.DV_Status->hardware_flags &= (0<<6);
//		dv.DV_Status->SteeringState = 0;
//	}
//	else
//	{
//		dv.DV_Status->hardware_flags |= STEERING_ON;
//		dv.DV_Status->SteeringState = 1;
//	}

	/* Process feedback from Linear */
	linear_feedback = (int16_t)(1024*dv.steering_linear_mm);
	data[0] = (linear_feedback >> 8);
	data[1] = (linear_feedback);

	sprintf(dummySp, "%02x%02x%02x%02x%02x\n", 0x00, 0x05, 0x02,
			data[1], data[0]);
	CDC_Transmit_FS((uint8_t*)dummySp, strlen(dummySp));

	/* Sending SYNC message in order for EPOS4 to return RPDO messages */
	if(CO.Slave_State == HB_OPERATIONAL)
	{
		send_SYNC(&CO);
	}

  /* USER CODE END timSYNC_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Task that handles PC Error if happened: Continuously transmits PC_ERROR */
void CAN2USB_Error_Handler()
{
	uint32_t error_notification;
	for(;;)
	{
		xTaskNotifyWait(0, 0xFFFFFFFF, &error_notification, portMAX_DELAY);

		/* In case of AS Emergency, zero-out steering shaft, then disable EPOS4
		 * while AS Emergency is transmitted */
		if(dv.DV_Status->AS_state == AS_EMERGENCY)
		{
			dv.DV_Dynamics->Motor_torque_target = 0;
			dv.DV_Dynamics->Brake_hydr_target   = 0;
			Target_Message(&dv);
		}

//		/* In case of any error, make sure to disable EPOS4 */
//		if(epos4.motor_enable && dv.steering_linear_mm == 0)
//		{
//			epos4.motor_enable = false;
//		}

		/* Transmit to open SD only if:
		 * - EMCY frame of EPOS4: Toggle warning LED, in case of EP0S4 EMCY problem
		 * - PC Error is indicated */
		if((CO.CO_Emergency)
		|| (dv.DV_Status->dv_health == PC_ERROR)
		|| (dv.DV_Status->AS_state == AS_EMERGENCY))
		{
			HAL_GPIO_TogglePin(CANOPEN_RED_GPIO_Port, CANOPEN_RED_Pin);
			open_sd_int = true;
		}

		osDelay(10);
	}
}


/* Task that runs continuously and handles EPOS4 control messages */
void EP0S4_Control_Handler()
{
	/* Must run continuously to update MAXON settings */
	for(;;)
	{
		/* EPOS4 Control frequency */
		control_time = xTaskGetTickCount() - control_tick;
		control_tick = xTaskGetTickCount();

		/* PID Control of MAXON Motor for Velocity control using LinearSensor measurement */
#if(!DEBUGGING)
		PIDController_Update(&pid, dv.DV_Dynamics->steer_target_in_mm, (float)dv.steering_linear_mm);
#else
		if(target_position > MAX_STEERING_MM)
			target_position = MAX_STEERING_MM;
		else if(target_position < MIN_STEERING_MM)
			target_position = MIN_STEERING_MM;

		PIDController_Update(&pid, target_position, (float)dv.steering_linear_mm);
#endif
		epos4.target_velocity = (int32_t)(pid.out);

//		/* Zero-out target velocity, whenever linear sensor reaches limits within 0.5mm */
//		if((dv.steering_linear_mm > MAX_STEERING_MM - 0.5)
//		|| (dv.steering_linear_mm < MIN_STEERING_MM + 0.5))
//		{
//			epos4.target_velocity = 0;
//		}

		/* In case of position/velocity control for EPOS4 */
		switch(epos4.mode_of_op)
		{
		case(1):	/* Profile Position (PPM) Mode control of EPOS4 */
			EPOS4_set_position(&CO, &epos4);
			break;
		case(9):	/* Profile Velocity (PVM) Mode control of EPOS4 */
			EPOS4_set_speed(&CO, &epos4);
			break;
		}

		osDelay(10);
	}
}


/* CAN2USB USB Transactions each time we receive the needed CAN Frame */
void CAN2USB_Tx_Handler()
{
	/* Infinite Loop */
	for(;;)
	{
		/* This task is notified only if CAN RX FIFO is filled with
		 * CAN Frames - Messages needed from PC and is notified, using a semaphore */
		if(xSemaphoreTake(xSemaphoreCAN2USB, portMAX_DELAY) == pdTRUE)
		{
			/* ------------------- CAN Communication with PC over USB -------------------------*/
			HAL_GPIO_TogglePin(CAN2USB_IND_GPIO_Port, CAN2USB_IND_Pin);

//			if(dv.DV_Status->AS_state == AS_EMERGENCY)
//			{
//				xTaskNotify(CAN2USB_Error_Handle, 0x00, eNoAction);
//			}

			CDC_Transmit_FS((uint8_t*)RxSp, strlen(RxSp));
			dv.rcv_time = xTaskGetTickCount() - dv.rcv_tick;
			dv.rcv_tick = xTaskGetTickCount();
			dv.rcv_flag = false;
		}
		osDelay(5);
	}
}


/* Mission Task that handles "SentOnce" messages from Dash about the mission as selected */
void MissionSelection_Handler()
{
	/* This code runs only if DashBoard sends over Mission messages */
	uint32_t tick0, lock_notification;
	for(;;)
	{
		/* Same for LOCKED / UNLOCKED commands from Dash */
		xTaskNotifyWait(0, 0xFFFFFFFF, &lock_notification, portMAX_DELAY);

		/* In case of Locked message, send ACK to PC in order to properly lock the mission.
		 * If everything checks OK, send ACK to Dash to */
		tick0 = xTaskGetTickCount();
		LockedMessageCheck(&dv, tick0);

		/* In case of UnLocked message */
		tick0 = xTaskGetTickCount();
		UnlockedMessageCheck(&dv, tick0);

		/* If PC doesn't communicate properly with CAN2USB to lock the mission
		 * CAN2USB indicates error to Dash, else lock mission */
		MissionACKCheck(&dv);

		/* Update Mission Handler, only if P23 has locked mission */
		if(dv.DV_Status->mission_locked == LOCKED)
			dv.DV_Status->PC_Locked_Mission = dv.DV_Status->dv_says_mission;

		/* Reset DV mission */
		dv.DV_Status->dv_says_mission = NOT_SELECTED;

		osDelay(5);
	}
}


/* Task that mainly handles what the CANOpen Communication with EPOS4 will involve
 * process PC status health message and control-target message */
void Target_Handler()
{
	uint32_t byte_changed;
	for(;;)
	{
		/* This task is notified only when DV PC sends target commands
		 * OR an error has occurred and we must stop the steering actuator
		 * or the torque/brake commands */
		xTaskNotifyWait(0, 0xFFFFFFFF, &byte_changed, portMAX_DELAY);

		/* Target Message as given from DV Control algorithms */
		if(dv.DV_Status->dv_health == DV_DRIVING)
		{
			Target_Message(&dv);
		}
		else
		{
			/* In case of MISSION_ENDED/PC_Error, we bypass PC commands to make sure Standstill */
			dv.DV_Dynamics->Motor_torque_target 		  = 0;
			dv.DV_Dynamics->Motor_torque_command_bytes[0] = 0;
			dv.DV_Dynamics->Motor_torque_command_bytes[1] = 0;
			dv.DV_Dynamics->Brake_hydr_target 			  = 0;
			Target_Message(&dv);
		}

		/* Enable/not MAXON motors */
		if(byte_changed == 0x03)
		{
			/* EP0S4 is enabled only if mission was selected
			 * & the AS status is AS_READY: Instructions to P23 are given
			 * if we move to AS_DRIVING */
			if(dv.DV_Status->AS_state == AS_READY
			&& (!epos4.motor_enable))
			{
				epos4.motor_enable = true;

				/* When EP0S4 is ready to be enabled, zero-out the steering shaft */
//				dv.DV_Dynamics->steer_target_in_mm = STEERING_ZERO;
			}
		}

		osDelay(5);
	}
}

/* USER CODE END Application */

