/*
 * DV.h
 *
 * Contains critical parameters for DV data_logging
 * & transmitting DV_PC necessary sensor variables
 * and receiving estimated data from PC.
 *
 */

#ifndef INC_DV_H_
#define INC_DV_H_

/* Includes */
#include "main.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "semphr.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "math.h"
#include <string.h>
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

#define PC_WAIT_TIME	100000

/* CAN defines */
#define CAN2USB_ID1 		0x302	// CAN ID's
#define CAN2USB_ID2			0x310
#define CAN2USB_ID3			0x275

/* DV defines */
#define DL_ID1				0x500
#define DL_ID2				0x501
#define DL_ID3				0x502

// MAX TORQUE & BRAKE
#define TORQUE_MAX	240
#define BRAKE_MAX	10

extern uint8_t debug_flag;
extern char    RxSp[200];
extern uint8_t RxData[12];

/* CAN2USB USB Transactions Semaphore */
extern SemaphoreHandle_t xSemaphoreCAN2USB;
extern BaseType_t xHigherPriorityCAN2USBTaskWoken;

/* Mission Handler Task parameters */
extern TaskHandle_t MissionSelect_Handle;
extern BaseType_t xHigherPriorityMission;

/* Error Handler Task parameters */
extern TaskHandle_t CAN2USB_Error_Handle;
extern BaseType_t xHigherPriorityError;

/* ASB STATE */
typedef enum{

	ASB_NOT_TALKING    = 0x00,
	INIT_CHECK_STARTED = 1<<0,
	SDC_CLOSED		   = 1<<1,
	INIT_CHECK_ENDED   = 1<<2,
	HEARTBEAT_MSG	   = 1<<3,

}ASB_Status_t;


/* EBS STATE */
typedef enum{

	NO_EBS_STATE	= 0x00,
	EBS_UNAVAILABLE = 0x01,
	EBS_ARMED		= 0x02,
	EBS_ACTIVATED	= 0x03,

}EBS_State_t;


/* ServiceBrake STATE */
typedef enum{

	SERVICE_BRAKE_DISENGAGED = 0x01,
	SERVICE_BRAKE_ENGAGED	 = 0x02,
	SERVICE_BRAKE_AVAILABLE	 = 0x03,

}ServiceBrake_State_t;


/* AMI STATE FLAGS */
typedef enum{

	NOT_SELECTED = 0x00,
	ACCELERATION = 0x01,
	SKIDPAD		 = 0x02,
	AUTOCROSS	 = 0x04,
	TRACKDRIVE 	 = 0x08,
	EBS_TEST	 = 0x10,
	INSPECTION	 = 0x20,
	MANUAL		 = 0x40,

}Mission_t;

/* Locking the mission */
typedef enum{

	UNLOCKED = 0,
	LOCKED,

}Locked_t;


/* Hardware condition of DV nodes */
typedef enum{

	HARDWARE_OK		   = 0x00,
	VN_200_ERROR       = 1<<0,
	VN_300_ERROR       = 1<<1,
	CAMERA_LEFT_ERROR  = 1<<2,
	CAMERA_RIGHT_ERROR = 1<<3,

//	INS_MODE_0 		= (0<<4 && 0<<5),
	INS_MODE_2_ERROR   = 1<<4,
	INS_MODE_1_ERROR   = 1<<5,
//	INS_MODE_3 		= (1<<4 && 1<<5),
	STEERING_OFF	   = 1<<6,

}HardwareStatus_t;


/* Software condition of DV nodes */
typedef enum{

	SOFTWARE_OK				   = 0x00,
	CLOCK_ERROR                = 1<<0,
	CAMERA_INFERENCE_ERROR     = 1<<1,
	VELOCITY_ESTIMATION_ERROR  = 1<<2,
	SLAM_ERROR         		   = 1<<3,
	MPC_CONTROLLER_ERROR	   = 1<<4,
	PATH_PLANNING_ERROR		   = 1<<5,
	PI_PP_CONTROLLER_ERROR     = 1<<6,

}SoftwareStatus_t;


/* AS Status flags */
typedef enum{

	NO_AS_STATE	 = 0x00,
	AS_OFF       = 0x01,
	AS_READY     = 0x02,
	AS_DRIVING   = 0x04,
	AS_FINISHED  = 0x08,
	AS_EMERGENCY = 0x10,

}AS_Status_t;


/* DV status flags */
typedef enum{

	LV_ON 		 	 = 0x00,
	MISSION_SELECTED = 0x01,
	DV_READY 		 = 0x02,
	DV_DRIVING		 = 0x04,
	MISSION_FINISHED = 0x08,
	PC_ERROR		 = 0x0F,

}DVHealth_t;


typedef struct DV_Dynamics
{
	/* Speed as measured from PC MPC: Used for DataLogger */
	uint8_t Speed_actual;
	uint8_t Speed_target;

	/* Steering Target & Actual measurements from ADC */
	int8_t Steering_target;

	/* Brake Pressure Target */
	uint8_t Brake_hydr_target;

	/* Motor Torque commands */
	float Motor_torque_target;
	uint8_t Motor_torque_command_bytes[2];

	/* Target translated */
	float steer_target_in_deg, steer_target_in_mm;

} DV_Dynamics;


/* DV_Kinematics variables needed */
typedef struct DV_Kinematics
{
	float acc_long;
	float acc_lat;
	float yaw_rate;

}DV_Kinematics;


typedef struct DV_Status
{
	/* Contains Autonomous System status & Emergency Brake System status */
	AS_Status_t 	     AS_state;
	EBS_State_t  	 	 EBS_state;
	ServiceBrake_State_t ServBrake_state;

	/* Mission & Contains Locked/not */
	Mission_t mission_selected;
	Locked_t  mission_locked;

	Mission_t dv_says_mission;
	Locked_t  dv_says_lock;

	Mission_t PC_Locked_Mission;

	/* DV Flags */
	uint8_t    SteeringState;
	DVHealth_t dv_health;

	/* Mainly needed for DataLogger */
	uint8_t  Lap_counter;
	uint8_t  Cones_count_actual;
	uint16_t Cones_count_all;

	/* Hardware Interface & PC Temperature */
	HardwareStatus_t hardware_flags;
	SoftwareStatus_t software_flags;
	uint8_t 		 pc_temperature;	// in *C

	/* ASB status */
	ASB_Status_t     ASB_status;
	uint8_t 		 sd_is_open;

} DV_Status;


typedef struct DV
{
	/* DV Characteristics */
	DV_Kinematics *DV_Kinematics;
	DV_Dynamics   *DV_Dynamics;
	DV_Status 	  *DV_Status;

	/* CAN Reception for USB Buffer TX */
	CAN_HandleTypeDef*   hcan;
	CAN_RxHeaderTypeDef* RxHeader;
	CAN_TxHeaderTypeDef* TxHeader;
	uint8_t 			 can_rx[8];
	uint8_t 			 can_tx[8];
	uint32_t*			 canMailbox;

	uint8_t 		     log_data1[8];
	uint8_t 		     log_data3[8];

	uint32_t 			 rcv_tick;
	uint32_t 			 rcv_time;
	bool				 rcv_flag;

	/* Shows the message that PC has sent to USB */
	uint8_t full_mission;
	uint8_t	DV_Identifier;

	float steering_linear_mm;
	float steering_deg;

	/* Actual Readings of sensors received through Autonomous CAN from VCU:
	 * - EMRAX torque
	 * - EBS pressure as measured from ASB
	 * - Brake Pressure
	 * - HALL speed measurements (rpm/s) */
	float Motor_torque_actual;
	float ServBrake_pressure;					// must be added in AutoCAN_RX();
	uint8_t brake_pressure_front;
	uint8_t brake_pressure_rear;
	float hall_front_right;
	float hall_front_left;
	float hall_rear_right;
	float hall_rear_left;

} DV;

/* Initialization */
void DV_Init(DV *dv, DV_Dynamics *dynamics, DV_Kinematics *kinematics, DV_Status *status,
			 CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxHeader,
			 CAN_TxHeaderTypeDef *TxHeader, uint32_t *canMailbox);

/* Transmit Autonomous - CAN frames for setting target / mission commands to P23*/
uint8_t Mission_ACK(DV *dv);
uint8_t Target_Message(DV *dv);
uint8_t GNSS_Tx(DV *dv);

/* DataLogger Functionality: Periodic transmission of messages */
uint8_t DataLogger_TX(DV *dv);

/* Mission Transmission to USB */
void MissionACKCheck(DV *dv);
void LockedMessageCheck(DV *dv, uint32_t tick0);
void UnlockedMessageCheck(DV *dv, uint32_t tick0);
void SendMissionToPC(DV *dv);

/* CAN PHY communication */
uint8_t AutoCAN_Tx(DV *dv, uint32_t id, uint8_t dlc, uint8_t* data);
uint8_t AutoCAN_Rx(DV *dv);

#endif /* INC_DV_H_ */
