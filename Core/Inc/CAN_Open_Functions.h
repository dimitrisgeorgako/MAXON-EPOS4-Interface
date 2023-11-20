/*
 * CAN_Open_Functions.h
 *
 *  Created on: Mar 2, 2023
 *      Author: gamin
 */

#ifndef INC_CAN_OPEN_FUNCTIONS_H_
#define INC_CAN_OPEN_FUNCTIONS_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "usbd_cdc_if.h"

#include "cmsis_os.h"
#include "event_groups.h"

#include "PID.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"

/* CANOpen defines */
#define TPDO_NUM	4
#define RPDO_NUM	4

/* 	EPOS4 RPDO List:
 *
 * 		RPDO0 --> 0x182: Mode of Operation | StatusWord
 *		RPDO1 --> 0X282: Actual_Velocity   | Target_Velocity
 *
 */

#define INITIAL_TPDO	0x200
#define INCR_TPDO		0x100

#define INITIAL_RPDO	0x180
#define INCR_RPDO		0x100

#define HEARTBEAT 		0x702
#define EMERGENCY		0x082

/* EPOS4 Defines */
#define MODE			0x09

/* Error Handler */
extern TaskHandle_t CAN2USB_Error_Handle;

typedef enum{

	EPOS4_OK = 0,
	CAN_BUS_OFF,
	CAN_RX_QUEUE_OVERFLOW,
	CAN_TX_QUEUE_OVERFLOW,
	CAN_PASSIVE_MODE_ERROR,
	CAN_HEARTBEAT_ERROR,
	CAN_OVERRUN_ERROR,
	MISSING_MAIN_SENSOR_ERROR,
	LOW_LV_SUPPLY_ERROR,
	THERMAL_OVERLOAD_ERROR,
	OVERVOLTAGE_ERROR,
	OVERCURRENT_ERROR,
	POWER_STAGE_PROTECTION_ERROR,
	GENERIC_ERROR,

}EPOS4_ErrorMessage_t;

typedef enum
{
	NO_BC  = 0,
	BC,
	BOOT_UP,

}Broadcast_t;


typedef enum
{
	NO_ERROR = 0,
	BOOT_UP_ERROR,
	SYNC_ERROR,
	BC_ERROR,
	CAN_ERROR,

}CO_Error_t;


typedef enum
{
	NMT       = 0x000,
	FAILSAFE  = 0x001,
	SYNC 	  = 0x080,
	TIMESTAMP = 0x100,

}CO_CMD_t;


typedef enum
{
	/* NMT_Master TX states */
	NMT_OPERATIONAL 		= 0x01,
	NMT_STOPPED 			= 0x02,
	NMT_PRE_OPERATIONAL 	= 0x80,
	NMT_RESET_NODE 			= 0x81,
	NMT_RESET_COMMUNICATION = 0x82,

	/* NMT_Slaves verification of their state */
	NO_STATE 			= 0xAA,
	HB_BOOT_UP 			= 0x00,
	HB_STOPPED 			= 0x04,
	HB_OPERATIONAL 		= 0x05,
	HB_PRE_OPERATIONAL  = 0x7F,

}CO_NMT_State_t;


typedef struct canOpen
{
	/* CAN Structure */
	CAN_HandleTypeDef*   hcan;
	CAN_RxHeaderTypeDef* RxHeader;
	CAN_TxHeaderTypeDef* TxHeader;
	uint8_t			 	 RxData[8];
	uint8_t			 	 TxData[8];
	uint32_t*			 canMailbox;

	/* Master - Slave's Node ID */
	uint8_t nodeId;
	uint8_t Slave_nodeId;
	CO_Error_t NMT_Master_error;
	CO_NMT_State_t Slave_State;

	/* Communication object IDs, depending on nodeID */
	uint32_t TPDO[TPDO_NUM];
	uint32_t RPDO[RPDO_NUM];

	/* CANOpen Flags */
	bool RPDO_rcv_flag[RPDO_NUM];
	bool CO_Emergency;
	bool BOOT_UP_flag;

	/* Debugging purposes */
	uint8_t flag_NMT;
	uint8_t tpdo_number;
	CO_NMT_State_t debug_state;

}canOpen;


typedef struct EPOS4
{
	/* Motor Situation flags */
	bool motor_enable;
	bool motor_quick_stop;

	/* EPOS4 Control */
	uint8_t EPOS4_OFF[8];
	uint8_t SHUTDOWN[8];
	uint8_t SWITCH_ON[8];
	uint8_t QUICK_STOP[8];

	/* EPOS4 status & errors */
	EPOS4_ErrorMessage_t Ep0s4_Error;
	uint16_t ErrorCode;
	uint8_t ErrorRegister;
	uint8_t data_emergency[8];

	uint8_t mode_of_op;

	/* Position and velocity, as demanded from NMT master */
	uint8_t target_data[8];
	int32_t previous_target;
	int32_t target_velocity;
	int32_t delta_target_position;

	/* Velocity, as measured from EPOS4 */
	int32_t actual_velocity;
	int32_t demand_velocity;
	float   actual_position;
	int32_t demand_position;

	/* EPOS4 Internal State Machine */
	uint8_t ready_to_switch_on;
	uint8_t switched_on;
	uint8_t operation_enabled;
	uint8_t fault;
	uint8_t voltage_enabled;
	uint8_t quick_stop;
	uint8_t switch_on_disabled;
	uint8_t warning;
	uint8_t remote;
	uint8_t target_reached;

	/* Power limitation */
	uint16_t ep0s4_temp;
	uint16_t i2t_level_moter, i2t_level_power_stage;

}EPOS4;


/* EPOS4 Initialization */
void EPOS4_Init(EPOS4 *EPOS4, canOpen *canOpen, CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef* RxHeader,
		CAN_TxHeaderTypeDef* TxHeader, uint32_t* canMailbox, uint8_t MasterId);

/* Ensures proper functionality of EPOS4 Operating states */
uint8_t EPOS4_app_process(canOpen *canOpen, EPOS4 *EPOS4);

/* Function that can set EPOS4 with the velocity target requested, when change is applied */
uint8_t EPOS4_set_speed(canOpen *canOpen, EPOS4 *EPOS4);

/* Function that can set EPOS4 with the position target requested, when change is applied */
uint8_t EPOS4_set_position(canOpen *canOpen, EPOS4 *EPOS4);

/* EPOS4 RPDO Configuration and BLDC Control */
void EPOS4_StatusWord(EPOS4 *EPOS4, uint8_t *data);
void EPOS4_VelocityReadings(EPOS4 *EPOS4, uint8_t *data);
void EPOS4_PositionReadings(EPOS4 *EPOS4, uint8_t *data);
void EPOS4_i2tReadings(EPOS4 *EPOS4, uint8_t *data);

/* Settings of CanOpen Node and Boot-Up HeartBeat */
void canOpen_Init(canOpen *canOpen, CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef* RxHeader,
				  CAN_TxHeaderTypeDef* TxHeader, uint32_t* canMailbox, uint8_t nodeId);

/* Receive update from Slave nodes - Node Guarding from NMT & Emergency messages */
uint8_t CO_EMERGENCY_Monitor(canOpen *canOpen, EPOS4 *EPOS4);
void 	EP0S4_ErrorHandling(EPOS4 *EPOS4);
uint8_t CO_HEARTBEAT_Monitor(canOpen *canOpen);

/* RPDO reception for EPOS4 */
uint8_t CO_RPDO_Read(canOpen *canOpen, EPOS4 *EPOS4);

/* Transmit TPDO & NMT commands */
uint8_t send_NMT_command(canOpen *canOpen, CO_NMT_State_t NMT_State_Int, Broadcast_t broadcast , uint8_t addressed_node);
uint8_t send_TPDO(canOpen *canOpen, uint8_t *data, int TPDO_num);
uint8_t send_SYNC(canOpen *canOpen);

/* CANOpen PHY communication Interface */
uint8_t CO_Send(canOpen *canOpen);
uint8_t CO_Receive(canOpen *canOpen, EPOS4 *EPOS4);
void 	erase_Txdata(canOpen *canOpen);

#endif /* INC_CAN_OPEN_FUNCTIONS_H_ */
