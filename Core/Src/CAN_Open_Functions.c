/*
 * CAN_Open_Functions.c
 *
 *  Created on: Mar 2, 2023
 *      Author: gamin
 */

#include "CAN_Open_Functions.h"

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

void canOpen_Init(canOpen *canOpen, CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef* RxHeader,
		CAN_TxHeaderTypeDef* TxHeader, uint32_t* canMailbox, uint8_t MasterId)
{
	/* CANOpen PHY Implementation */
	canOpen->hcan 		= hcan;
	canOpen->RxHeader 	= RxHeader;
	canOpen->TxHeader 	= TxHeader;
	canOpen->canMailbox = canMailbox;

	for(uint8_t i = 0; i < 8; i++)
	{
		canOpen->RxData[i] = 0;
		canOpen->TxData[i] = 0;
	}

	/* Master - Slave's Node ID */
	canOpen->nodeId 	  = MasterId;
	canOpen->Slave_nodeId = 2;
	canOpen->Slave_State  = NO_STATE;

	/* PDO COB-ID settingS */
	/* TPDO - EPOS4 */
	canOpen->TPDO[0] = INITIAL_TPDO + canOpen->Slave_nodeId;
	for(uint8_t i = 1; i < TPDO_NUM; i++)
		canOpen->TPDO[i] = canOpen->TPDO[i-1] + INCR_TPDO;
	/* RPDO - EPOS4 */
	canOpen->RPDO[0] = INITIAL_RPDO + canOpen->Slave_nodeId;
	for(uint8_t i = 1; i < RPDO_NUM; i++)
		canOpen->RPDO[i] =  canOpen->RPDO[i-1] + INCR_RPDO;

	/* RPDO Flags */
	for(uint8_t i = 0; i < RPDO_NUM; i++)
		canOpen->RPDO_rcv_flag[i] = false;

	/* Boot Up flag */
	canOpen->BOOT_UP_flag = false;

	/* NMT Master produces no-error and Slave's are not in Emergency Mode */
	canOpen->CO_Emergency 	  = false;
	canOpen->NMT_Master_error = NO_ERROR;

	/* Debugging */
	canOpen->flag_NMT  = false;
	canOpen->debug_state = NMT_OPERATIONAL;
	canOpen->tpdo_number = 0;

}


/* EPOS4 Initialization */
void EPOS4_Init(EPOS4 *EPOS4, canOpen *canOpen, CAN_HandleTypeDef* hcan, CAN_RxHeaderTypeDef* RxHeader,
		CAN_TxHeaderTypeDef* TxHeader, uint32_t* canMailbox, uint8_t MasterId)
{
	/* Configure steering PID controller */
	PIDController_Init(&pid, Kp, Ki, Kd, tau, limMin, limMax, limMinInt, limMaxInt, Ts);

	/* CANOpen Interface and configuration */
	canOpen_Init(canOpen, hcan, RxHeader, TxHeader, canMailbox, MasterId);

	/* BLDC Motor situation flags */
	EPOS4->motor_enable  = false;
	EPOS4->motor_quick_stop = false;

	/* EPOS4 command Initialization */
	EPOS4->EPOS4_OFF[0]  = 0X00;

	EPOS4->SHUTDOWN[0]   = 0x06;

	EPOS4->SWITCH_ON[0]  = 0x0F;
	EPOS4->SWITCH_ON[2]  = MODE;

	EPOS4->QUICK_STOP[0] = 0x0B;

	/* EPOS4 EMCY messages */
	EPOS4->ErrorCode		 = 0x0000;
	EPOS4->ErrorRegister	 = 0x00;
	EPOS4->data_emergency[0] = 0x80;	// Reset Fault of EPOS4

	for(uint8_t i = 0; i < sizeof(EPOS4->target_data); i++)
		EPOS4->target_data[i] = 0;

}

/* Ensures proper functionality of EPOS4 Operating states */
uint8_t EPOS4_app_process(canOpen *canOpen, EPOS4 *EPOS4)
{
	/* Make sure EP0S4 is in operational mode */
	if(canOpen->Slave_State == HB_OPERATIONAL)
	{
		/* Motor enable management:
		 * Send Shutdown message before switching on EPOS4 */
		if((!EPOS4->ready_to_switch_on) && (EPOS4->motor_enable))
			send_TPDO(canOpen, EPOS4->SHUTDOWN, 0);

		 /* Enable operation of EPOS4 */
		if(EPOS4->ready_to_switch_on && (!EPOS4->operation_enabled) && (EPOS4->motor_enable))
			send_TPDO(canOpen, EPOS4->SWITCH_ON, 0);

		 /* Stop EPOS4 if enabled and must on the go stop EPOS4 controller
		  * - First zero velocity, then OFF command */
		if((EPOS4->ready_to_switch_on) && (EPOS4->operation_enabled) && (!EPOS4->motor_enable))
		{
			if((uint32_t)(EPOS4->demand_velocity) != 0)
			{
				EPOS4->target_velocity = 0;
				EPOS4_set_speed(canOpen, EPOS4);
			}

			send_TPDO(canOpen, EPOS4->EPOS4_OFF, 0);
		}
	}
	else if(canOpen->Slave_State == NO_STATE)
		send_NMT_command(canOpen, NMT_RESET_NODE, NO_BC , canOpen->Slave_nodeId);

	/* Quick Stop command */
	if(EPOS4->motor_quick_stop)
		send_TPDO(canOpen, EPOS4->QUICK_STOP, 0);

	return HAL_OK;
}


/* Function that can set EPOS4 with the velocity target requested, when change is applied */
uint8_t EPOS4_set_speed(canOpen *canOpen, EPOS4 *EPOS4)
{
	if((EPOS4->operation_enabled)
			&& (EPOS4->target_velocity != EPOS4->previous_target)
			&& (canOpen->Slave_State == HB_OPERATIONAL))
	{
		EPOS4->previous_target = EPOS4->target_velocity;

		uint32_t unsignedVelocity = (uint32_t)(EPOS4->target_velocity);

		/* Set demanded velocity target */
		EPOS4->target_data[0] = (unsignedVelocity & 0xFF);
		EPOS4->target_data[1] = (unsignedVelocity >> 8) & 0xFF;
		EPOS4->target_data[2] = (unsignedVelocity >> 16) & 0xFF;
		EPOS4->target_data[3] = (unsignedVelocity >> 24) & 0xFF;

		if(send_TPDO(canOpen, EPOS4->target_data, 1) != HAL_OK)
			return HAL_ERROR;

		/* Apply control to EPOS4 to execute command */
		EPOS4->target_data[0] = 0x0F;
		EPOS4->target_data[1] = 0;
		EPOS4->target_data[2] = 0;
		EPOS4->target_data[3] = 0;

		return send_TPDO(canOpen, EPOS4->target_data, 0);
	}

	return HAL_OK;
}

/* Function that can set EPOS4 with the position target requested, when change is applied */
uint8_t EPOS4_set_position(canOpen *canOpen, EPOS4 *EPOS4)
{
	if((EPOS4->operation_enabled) && (EPOS4->delta_target_position != 0) && (canOpen->Slave_State == HB_OPERATIONAL))
	{
		EPOS4->previous_target = EPOS4->delta_target_position;

		uint32_t unsignedPosition = (uint32_t)(EPOS4->delta_target_position);

		EPOS4->delta_target_position = 0;

		/* Set demanded position target */
		EPOS4->target_data[0] = (unsignedPosition & 0xFF);
		EPOS4->target_data[1] = (unsignedPosition >> 8) & 0xFF;
		EPOS4->target_data[2] = (unsignedPosition >> 16) & 0xFF;
		EPOS4->target_data[3] = (unsignedPosition >> 24) & 0xFF;

		if(send_TPDO(canOpen, EPOS4->target_data, 2) != HAL_OK)
			return HAL_ERROR;

		osDelay(2);

		/* Apply control to EPOS4 to execute command to relative control */
		EPOS4->target_data[0] = 0x7F;
		EPOS4->target_data[1] = 0;
		EPOS4->target_data[2] = 0;
		EPOS4->target_data[3] = 0;

		if(send_TPDO(canOpen, EPOS4->target_data, 0) != HAL_OK)
			return HAL_ERROR;

		osDelay(2);

		EPOS4->target_data[0] = 0x0F;
		EPOS4->target_data[1] = 0;
		EPOS4->target_data[2] = 0;
		EPOS4->target_data[3] = 0;

		return send_TPDO(canOpen, EPOS4->target_data, 0);
	}

	return HAL_OK;
}

/* EPOS4 Status */
void EPOS4_StatusWord(EPOS4 *EPOS4, uint8_t *data)
{
	/* StatusWord parameters */
	EPOS4->ready_to_switch_on = (data[0] & 0x01) >> 0;
	EPOS4->switched_on 		  = (data[0] & 0x02) >> 1;
	EPOS4->operation_enabled  = (data[0] & 0x04) >> 2;
	EPOS4->fault			  = (data[0] & 0x08) >> 3;
	EPOS4->voltage_enabled 	  = (data[0] & 0x10) >> 4;
	EPOS4->quick_stop 	 	  = (data[0] & 0x20) >> 5;
	EPOS4->switch_on_disabled = (data[0] & 0x40) >> 6;
	EPOS4->warning 			  = (data[0] & 0x80) >> 7;

	EPOS4->remote 			  = (data[1] & 0x02) >> 1;;
	EPOS4->target_reached 	  = (data[1] & 0x04) >> 2;;

	/* EPOS4 mode of operation */
	EPOS4->mode_of_op = data[2];
}


/* EPOS4 Velocity sensor readings */
void EPOS4_VelocityReadings(EPOS4 *EPOS4, uint8_t *data)
{
	/* Actual velocity message */
	EPOS4->actual_velocity  =
					(int32_t)(0.1*
					((data[0])
					|(data[1] << 8)
					|(data[2] << 16)
					|(data[3] << 24)));

	/* Demand velocity message */
	EPOS4->demand_velocity  =
					(int32_t)(0.1*
					((data[4])
					|(data[5] << 8)
					|(data[6] << 16)
					|(data[7] << 24)));
}


/* EPOS4 Position sensor readings */
void EPOS4_PositionReadings(EPOS4 *EPOS4, uint8_t *data)
{
//	char dummySp[200];

	/* Actual position message */
	EPOS4->actual_position =
					(int32_t)(
					((data[0])
					|(data[1] << 8)
					|(data[2] << 16)
					|(data[3] << 24)));

	/* Set feedback value to DV PC */
//	sprintf(dummySp, "%02x%02x%02x%02x%02x%02x%02x\n", 0x00, 0x05, 0x04,
//			data[0], data[1], data[2], data[3]);
//	CDC_Transmit_FS((uint8_t*)dummySp, strlen(dummySp));

	/* Demand position message */
	EPOS4->demand_position =
					(int32_t)(
					((data[4])
					|(data[5] << 8)
					|(data[6] << 16)
					|(data[7] << 24)));

}


void EPOS4_i2tReadings(EPOS4 *EPOS4, uint8_t *data)
{
	EPOS4->ep0s4_temp 		     = (uint16_t)(data[0] | (data[1] << 8));
	EPOS4->i2t_level_power_stage = (uint16_t)(data[2] | (data[3] << 8));
}


/* RPDO readings from EPOS4 */
uint8_t CO_RPDO_Read(canOpen *canOpen, EPOS4 *EPOS4)
{
	/* Check for EPOS4 TPDO messages */
	switch(canOpen->RxHeader->StdId)
	{
	case(0x182):
		canOpen->RPDO_rcv_flag[0] = true;
		EPOS4_StatusWord(EPOS4 ,canOpen->RxData);
		if(EPOS4->fault)
			send_TPDO(canOpen, EPOS4->data_emergency, 0);
		break;

	case(0x282):
		canOpen->RPDO_rcv_flag[1] = true;
		EPOS4_VelocityReadings(EPOS4, canOpen->RxData);
		break;

	case(0x382):
		canOpen->RPDO_rcv_flag[2] = true;
		EPOS4_PositionReadings(EPOS4, canOpen->RxData);
		break;

	case(0x482):
		canOpen->RPDO_rcv_flag[3] = true;
		EPOS4_i2tReadings(EPOS4, canOpen->RxData);
		break;
	}

	return HAL_OK;
}


/* Emergency message Monitoring */
uint8_t CO_EMERGENCY_Monitor(canOpen *canOpen, EPOS4 *EPOS4)
{
	/* EPOS4 has indicated an emergency error */
	canOpen->CO_Emergency  = true;
	EPOS4->ErrorCode 	   = (canOpen->RxData[0] | (canOpen->RxData[1] << 8));
	EPOS4->ErrorRegister   = canOpen->RxData[2];

	/* Distinguishes the EPOS4_Error that has occurred and will be used to transmit it via P23Telemetry */
	EP0S4_ErrorHandling(EPOS4);

	/* In case of an EMCY object message, notify CAN2USB Error Handler
	 * that disables EPOS4 in case of an EMCY frame */
//	xTaskNotify(CAN2USB_Error_Handle, EPOS4->ErrorCode, eSetValueWithOverwrite);

	/* Only in Operational, a ResetFault TPDO can be sent */
	if(canOpen->Slave_State == HB_OPERATIONAL)
	{
		return send_TPDO(canOpen, EPOS4->data_emergency, 0);
	}
	/* Else, reset communication for EPOS4 */
	else
	{
		if(send_NMT_command(canOpen, NMT_OPERATIONAL, NO_BC, canOpen->Slave_nodeId) != HAL_OK)
			return HAL_ERROR;
		return send_TPDO(canOpen, EPOS4->data_emergency, 0);
	}

	return HAL_OK;
}


void EP0S4_ErrorHandling(EPOS4 *EPOS4)
{
	switch(EPOS4->ErrorCode)
	{
	case(0x2310):
			EPOS4->Ep0s4_Error = OVERCURRENT_ERROR;
			break;

	case(0x2320):
			EPOS4->Ep0s4_Error = POWER_STAGE_PROTECTION_ERROR;
			break;

	case(0x3210):
			EPOS4->Ep0s4_Error = OVERVOLTAGE_ERROR;
			break;

	case(0x4210):
			EPOS4->Ep0s4_Error = THERMAL_OVERLOAD_ERROR;
			break;

	case(0x5113):
			EPOS4->Ep0s4_Error = LOW_LV_SUPPLY_ERROR;
			break;

	case(0x7390):
			EPOS4->Ep0s4_Error = MISSING_MAIN_SENSOR_ERROR;
			break;

	case(0x8111):
			EPOS4->Ep0s4_Error = CAN_OVERRUN_ERROR;
			break;

	case(0x8120):
			EPOS4->Ep0s4_Error = CAN_PASSIVE_MODE_ERROR;
			break;

	case(0x8130):
			EPOS4->Ep0s4_Error = CAN_HEARTBEAT_ERROR;
			break;

	case(0x81FD):
		    EPOS4->Ep0s4_Error = CAN_BUS_OFF;
			break;

	case(0x81FE):
			EPOS4->Ep0s4_Error = CAN_RX_QUEUE_OVERFLOW;
			break;

	case(0x81FF):
			EPOS4->Ep0s4_Error = CAN_TX_QUEUE_OVERFLOW;
			break;

	case(0x1000):
			EPOS4->Ep0s4_Error = GENERIC_ERROR;
			break;
	}
}


/* CANOpen PHY NMT monitoring/node guarding and TPDO commands */
uint8_t CO_HEARTBEAT_Monitor(canOpen *canOpen)
{
	switch(canOpen->RxData[0])
	{
	/* Pre-Operational state of EPOS4 */
	case HB_PRE_OPERATIONAL:
		canOpen->Slave_State = HB_PRE_OPERATIONAL;
		if((send_NMT_command(canOpen, NMT_OPERATIONAL, NO_BC, canOpen->Slave_nodeId) != HAL_OK))
			return HAL_ERROR;
		break;

	/* Operational state of EPOS4 */
	case HB_OPERATIONAL:
		canOpen->Slave_State = HB_OPERATIONAL;
		break;

	/* Stopped state of EPOS4 */
	case HB_STOPPED:
		canOpen->Slave_State = HB_STOPPED;
		break;

	/* EPOS4 has booted up */
	case HB_BOOT_UP:
		canOpen->BOOT_UP_flag = true;
		canOpen->Slave_State = HB_BOOT_UP;
		break;
	}

#if(DEBUGGING)
	if(canOpen->flag_NMT)
	{
		canOpen->flag_NMT = false;
		return send_NMT_command(canOpen, canOpen->debug_state, NO_BC, canOpen->Slave_nodeId);
	}
#endif

	return HAL_OK;
}


/* Transmit NMT commands */
uint8_t send_NMT_command(canOpen *canOpen, CO_NMT_State_t NMT_State_Int, Broadcast_t broadcast , uint8_t addressed_node)
{
	if((NMT_State_Int != (NMT_OPERATIONAL | NMT_STOPPED | NMT_PRE_OPERATIONAL | NMT_RESET_NODE | NMT_RESET_COMMUNICATION)) && (broadcast != BOOT_UP))
		canOpen->NMT_Master_error = CAN_ERROR;

	canOpen->TxHeader->DLC = 2;
	canOpen->TxHeader->StdId = NMT;
	canOpen->TxHeader->IDE = CAN_ID_STD;
	canOpen->TxHeader->RTR = CAN_RTR_DATA;
	canOpen->TxHeader->TransmitGlobalTime  = DISABLE;

	erase_Txdata(canOpen);

	/* Intentional NMT state that all slave nodes should proceed */
	canOpen->TxData[0] = NMT_State_Int;

	if(broadcast == NO_BC)
		canOpen->TxData[1] = addressed_node;	// Slave node ID requested to make a NMT state change

	else if(broadcast == BC)
		canOpen->TxData[1] = BC;

	else if(broadcast == BOOT_UP)
	{
		canOpen->TxHeader->StdId = 0x700 + canOpen->nodeId;
		canOpen->TxData[0] = HB_BOOT_UP;
	}

	else
		canOpen->NMT_Master_error = BC_ERROR;

	return CO_Send(canOpen);
}


/* Transmit TPDO's via CANOpen */
uint8_t send_TPDO(canOpen *canOpen, uint8_t *data, int TPDO_num)
{
	erase_Txdata(canOpen);

	canOpen->TxHeader->DLC = 8;
	canOpen->TxHeader->StdId = canOpen->TPDO[TPDO_num];
	canOpen->TxHeader->IDE = CAN_ID_STD;
	canOpen->TxHeader->RTR = CAN_RTR_DATA;
	canOpen->TxHeader->TransmitGlobalTime  = DISABLE;

	for(uint8_t i = 0; i < sizeof(data); i++)
		canOpen->TxData[i] = data[i];

	return CO_Send(canOpen);
}


/* Transmit SYNC frame */
uint8_t send_SYNC(canOpen *canOpen)
{
	canOpen->TxHeader->DLC = 8;
	canOpen->TxHeader->StdId = SYNC;
	canOpen->TxHeader->IDE = CAN_ID_STD;
	canOpen->TxHeader->RTR = CAN_RTR_DATA;
	canOpen->TxHeader->TransmitGlobalTime  = DISABLE;

	erase_Txdata(canOpen);

	return CO_Send(canOpen);
}


/* CANOpen PHY communication */
uint8_t CO_Receive(canOpen *canOpen, EPOS4 *EPOS4)
{
	/* Receive via CANOpen PHY and check whether there is NMT message/RPDO request */
	if(HAL_CAN_GetRxMessage(canOpen->hcan, CAN_RX_FIFO0, canOpen->RxHeader, canOpen->RxData) != HAL_OK)
	{
		canOpen->NMT_Master_error = CAN_ERROR;
		return HAL_ERROR;
	}

	/* RPDO reading from EPOS4 */
	if(canOpen->Slave_State == HB_OPERATIONAL)
		CO_RPDO_Read(canOpen, EPOS4);

	/* Check for Heart-beat responses or BOOT_UP messages from EPOS4 */
	switch(canOpen->RxHeader->StdId)
	{
	case(HEARTBEAT):
		return CO_HEARTBEAT_Monitor(canOpen);

	case(EMERGENCY):
		return CO_EMERGENCY_Monitor(canOpen, EPOS4);
	}

	return HAL_OK;
}

/* CAN Send frames */
uint8_t CO_Send(canOpen *canOpen)
{
	if (HAL_CAN_AddTxMessage(canOpen->hcan, canOpen->TxHeader, canOpen->TxData, canOpen->canMailbox) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}


/* Erase TX buffer for new CO commands */
void erase_Txdata(canOpen *canOpen)
{
	for(uint8_t i = 0; i < 8; i++)
		canOpen->TxData[i] = 0;
}
