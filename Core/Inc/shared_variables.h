/*
 * shared_variables.h
 *
 *  Created on: May 9, 2023
 *      Author: gamin
 */

#ifndef INC_SHARED_VARIABLES_H_
#define INC_SHARED_VARIABLES_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"

/* Peripheral Includes */
#include "adc.h"
#include "tim.h"
#include "can.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* CAN2USB Includes */
#include "CAN_Open_Functions.h"
#include "DV.h"
#include "PID.h"

/* Debugging */
#define DEBUGGING 	1

typedef enum{

	EPOS4_EMERGENCY = 1<<0,
	DV_EMERGENCY    = 1<<1,

}Error_t;

/* canOpen Node PHY settings */
extern canOpen CO;
extern EPOS4 epos4;

/* DV basic structure */
extern DV 			  dv;
extern DV_Dynamics    dv_dynamics;
extern DV_Kinematics  dv_kinematics;
extern DV_Status 	  dv_status;

/* PID Controller for Steering motor & PID settings */
extern PIDController pid;
extern float Kp;
extern float Ki;
extern float Kd;
extern float tau;
extern float Ts;
extern float limMin;
extern float limMax;
extern float limMinInt;
extern float limMaxInt;

/* EP0S4 Task parameters */
extern TaskHandle_t Target_Handle;
extern BaseType_t xHigherPriorityTarget;

/* CAN2USB USB Transactions Task parameters */
extern TaskHandle_t CAN2USB_Tx_Handle;
extern BaseType_t xHigherPriorityCAN2USB;

/* Mission Handler Task parameters */
extern TaskHandle_t MissionSelect_Handle;
extern BaseType_t xHigherPriorityMission;

/* Target Variables */
extern volatile float target_position;

/* RX buffer for USB Transactions */
extern uint8_t buffer[64];

/* CAN2USB TX Buffer */
extern uint8_t RxData[12];

/* CAN settings */
extern CAN_RxHeaderTypeDef RxHeader1,RxHeader2;
extern CAN_TxHeaderTypeDef TxHeader1,TxHeader2;
extern uint32_t 		   canMailbox;

/* DEBUG VARIABLES */
extern uint8_t dummy_flag;

#endif /* INC_SHARED_VARIABLES_H_ */
