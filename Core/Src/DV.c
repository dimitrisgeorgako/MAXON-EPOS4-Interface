/*
 * DV.c
 *
 */
#include "DV.h"

uint32_t tickCAN, timeCAN;

void DV_Init(DV *dv, DV_Dynamics *dynamics, DV_Kinematics *kinematics, DV_Status *status,
			 CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *RxHeader,
			 CAN_TxHeaderTypeDef *TxHeader, uint32_t *canMailbox)
{
	/* Autonomous CAN Initialization */
	dv->hcan	 = hcan;
	dv->RxHeader = RxHeader;
	dv->TxHeader = TxHeader;
	dv->canMailbox = canMailbox;
	for(uint8_t i = 0; i < 8; ++i)
	{
		dv -> can_rx[i] = 0;
		dv -> can_tx[i] = 0;
	}

	/* DV_Status */
	dv->DV_Status->AS_state  = NO_AS_STATE;
	dv->DV_Status->EBS_state = NO_EBS_STATE;

	/* Mission & Contains Locked/not */
	dv->DV_Status->mission_selected = MANUAL;
	dv->DV_Status->mission_locked	= UNLOCKED;
	dv->DV_Status->dv_says_mission  = MANUAL;

	/* DV Flags */
	dv->DV_Status->dv_health = LV_ON;

	/* DV object */
	dv -> DV_Dynamics 	= dynamics;
	dv -> DV_Kinematics = kinematics;
	dv -> DV_Status	  	= status;

	dv -> DV_Identifier = 0x00;
	dv -> hcan	   	  	= hcan;
	dv -> RxHeader 	  	= RxHeader;
	dv -> TxHeader 	  	= TxHeader;
	dv -> canMailbox 	= canMailbox;

	/* ASB status & SDC condition */
	dv->DV_Status->ASB_status = ASB_NOT_TALKING;
	dv->DV_Status->sd_is_open = 1;

	/* Start USB PHY communication */
	HAL_GPIO_WritePin(GPIOC, USB_ENABLE_Pin, GPIO_PIN_SET);

}


/* Send via Autonomous CAN bus data that DV is calculating during MPC running */
uint8_t Target_Message(DV *dv)
{
	/* DV Status */
	dv->can_tx[0] = (uint8_t)((dv->DV_Status->dv_health) | (dv->DV_Status->Lap_counter << 4));

	/* Motor Torque target */
	dv->can_tx[1] = dv->DV_Dynamics->Motor_torque_command_bytes[0];
	dv->can_tx[2] = dv->DV_Dynamics->Motor_torque_command_bytes[1];

	/* Steering Angle target */
	dv->can_tx[3] = (dv->DV_Dynamics->Steering_target);

	/* Brake Pressure target */
	dv->can_tx[4] = (uint8_t)(dv->DV_Dynamics->Brake_hydr_target);

	/* Speed as calculated from DV */
	dv->can_tx[5] = (uint8_t)(dv->DV_Dynamics->Speed_actual);
	dv->can_tx[6] = (uint8_t)(dv->DV_Dynamics->Speed_target);

	dv->can_tx[7] = 0;

	if(AutoCAN_Tx(dv, CAN2USB_ID1, 8, dv->can_tx) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}


/* Send Autonomous CAN data that DV is calculating with their sensors */
uint8_t GNSS_Tx(DV *dv)
{
	dv->can_tx[0] = (uint8_t)(dv->DV_Kinematics -> yaw_rate) >> 8;
	dv->can_tx[1] = (uint8_t)(dv->DV_Kinematics -> yaw_rate);
	dv->can_tx[2] = (uint8_t)(dv->DV_Kinematics -> acc_long) >> 8;
	dv->can_tx[3] = (uint8_t)(dv->DV_Kinematics -> acc_long);
	dv->can_tx[4] = (uint8_t)(dv->DV_Kinematics -> acc_lat)  >> 8;
	dv->can_tx[5] = (uint8_t)(dv->DV_Kinematics -> acc_lat);
	dv->can_tx[6] = 0;
	dv->can_tx[7] = 0;

	if(AutoCAN_Tx(dv, DL_ID2, 6, dv->can_tx) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}


/* Send every 250msec to the FSG Data Logger the DV data */
uint8_t DataLogger_TX(DV *dv)
{
	/* Targets to DL */
	dv->log_data1[0] = dv->DV_Dynamics->Speed_actual;
	dv->log_data1[1] = dv->DV_Dynamics->Speed_target;
	dv->log_data1[2] = ((int8_t)(dv->steering_deg)*0.5);
	dv->log_data1[3] = (dv->DV_Dynamics->Steering_target)*0.5;
	dv->log_data1[4] = (dv->brake_pressure_front);
	dv->log_data1[5] = dv->DV_Dynamics->Brake_hydr_target;
	dv->log_data1[6] = (int8_t)(dv->Motor_torque_actual);
	dv->log_data1[7] = (int8_t)(dv->DV_Dynamics->Motor_torque_target);

	if(AutoCAN_Tx(dv, DL_ID1, 8, dv->log_data1) != HAL_OK)
		return HAL_ERROR;
	osDelay(25);

	/* Sensors to DL */
	if(GNSS_Tx(dv) != HAL_OK)
		return HAL_ERROR;
	osDelay(25);

	/* Status to DL */
	switch(dv->DV_Status->AS_state)
	{
	case(NO_AS_STATE):
		break;

	case(AS_OFF):
		dv->log_data3[0] = 1;
		break;

	case(AS_READY):
		dv->log_data3[0] = 2;
		break;

	case(AS_DRIVING):
		dv->log_data3[0] = 3;
		break;

	case(AS_EMERGENCY):
		dv->log_data3[0] = 4;
		break;

	case(AS_FINISHED):
		dv->log_data3[0] = 5;
		break;
	}

	switch(dv->DV_Status->mission_selected)
	{
	case(NOT_SELECTED):
		break;

	case(MANUAL):
		break;

	case(ACCELERATION):
		dv->log_data3[0] |= (1<<5);
		break;

	case(SKIDPAD):
		dv->log_data3[0] |= (2<<5);
		break;

	case(TRACKDRIVE):
		dv->log_data3[0] |= (3<<5);
		break;

	case(EBS_TEST):
		dv->log_data3[0] |= (4<<5);
		break;

	case(INSPECTION):
		dv->log_data3[0] |= (5<<5);
		break;

	case(AUTOCROSS):
		dv->log_data3[0] |= (6<<5);
		break;
	}

	dv->log_data3[0] |= (dv->DV_Status->EBS_state << 3);
	dv->log_data3[1] = (dv->DV_Status->SteeringState | ((dv->DV_Status->ServBrake_state) << 1) | ((dv->DV_Status->Lap_counter) << 3) | ((dv->DV_Status->Cones_count_actual) << 7));
	dv->log_data3[2] = ((dv->DV_Status->Cones_count_actual >> 1) | ((dv->DV_Status->Cones_count_all) << 8));
	dv->log_data3[3] = dv->DV_Status->Cones_count_all >> 1;
	dv->log_data3[4] = dv->DV_Status->Cones_count_all << 8;

	if(AutoCAN_Tx(dv, DL_ID3, 5, dv->log_data3) != HAL_OK)
		return HAL_ERROR;
	osDelay(25);

	return HAL_OK;
}


/* CAN PHY Transaction communication */
uint8_t AutoCAN_Tx(DV *dv, uint32_t id, uint8_t dlc, uint8_t* data)
{
	dv->TxHeader->DLC   = dlc;
	dv->TxHeader->StdId = id;
	dv->TxHeader->IDE 	= CAN_ID_STD;
	dv->TxHeader->RTR 	= CAN_RTR_DATA;
	dv->TxHeader->TransmitGlobalTime  = DISABLE;

	for(uint8_t i = 0; i < sizeof(data); i++)
	{
		dv->can_tx[i] = data[i];
	}

	if (HAL_CAN_AddTxMessage(dv->hcan, dv->TxHeader, dv->can_tx, dv->canMailbox) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}


/* Receive CAN packets from Autonomous CAN Interface */
uint8_t AutoCAN_Rx(DV *dv)
{
	if(HAL_CAN_GetRxMessage(dv->hcan, CAN_RX_FIFO1, dv->RxHeader, dv->can_rx) != HAL_OK)
		return HAL_ERROR;

	RxData[0] = dv->RxHeader->StdId >> 8;
	RxData[1] = dv->RxHeader->StdId;
	RxData[2] = dv->RxHeader->DLC;

	switch(dv->RxHeader->StdId)
	{
		case(0x306):
			dv->DV_Status->AS_state = dv->can_rx[0];	// From VCU: AS_Status (Total: 1 byte)

			for(uint8_t i = 0; i < dv->RxHeader->DLC; ++i)
				RxData[i+3] = dv->can_rx[i];

			sprintf(RxSp, "%02x%02x%02x%02x\n", RxData[0], RxData[1], RxData[2], RxData[3]);

			xSemaphoreGiveFromISR(xSemaphoreCAN2USB, &xHigherPriorityCAN2USBTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityCAN2USBTaskWoken);
			return HAL_OK;

		case(0x300):	// Receive from VCU:  BrakePressure | ActualTorque (Total: 4 bytes)
			dv->brake_pressure_front = dv->can_rx[0];
			dv->brake_pressure_rear  = dv->can_rx[1];
			dv->Motor_torque_actual  = (float)((dv->can_rx[2] << 8) | dv->can_rx[3]);

			timeCAN = xTaskGetTickCount() - tickCAN;
			tickCAN = xTaskGetTickCount();

			for(uint8_t i = 0; i < dv->RxHeader->DLC; ++i)
				RxData[i+3] = dv->can_rx[i];

			RxData[2] = 4;

			sprintf(RxSp, "%02x%02x%02x%02x%02x%02x%02x\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6]);

			xSemaphoreGiveFromISR(xSemaphoreCAN2USB, &xHigherPriorityCAN2USBTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityCAN2USBTaskWoken);
			return HAL_OK;

		case(0x301):	// Receive from VCU: Hall sensor bytes from ADC's
			dv->hall_front_left  = (float)(dv->can_rx[1] | (dv->can_rx[0] << 8));
			dv->hall_front_right = (float)(dv->can_rx[3] | (dv->can_rx[2] << 8));
			dv->hall_rear_left 	 = (float)(dv->can_rx[5] | (dv->can_rx[4] << 8));
			dv->hall_rear_right  = (float)(dv->can_rx[7] | (dv->can_rx[6] << 8));

			for(uint8_t i = 0; i < dv->RxHeader->DLC; ++i)
				RxData[i+3] = dv->can_rx[i];

			sprintf(RxSp, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7], RxData[8], RxData[9], RxData[10]);

			xSemaphoreGiveFromISR(xSemaphoreCAN2USB, &xHigherPriorityCAN2USBTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityCAN2USBTaskWoken);
			return HAL_OK;

		case(0x304):	// Dashboard-ID: We need AMI State - Along with Mission_Locked flag
			dv->full_mission 				= (dv->can_rx[0]);
			dv->DV_Status->mission_selected = (dv->can_rx[0] & 0x7F);
			dv->DV_Status->mission_locked   = (dv->can_rx[0] & 0x80) >> 7;

			xTaskNotifyFromISR(MissionSelect_Handle, dv->DV_Status->mission_locked, eSetValueWithOverwrite, &xHigherPriorityMission);
		    portYIELD_FROM_ISR(xHigherPriorityMission);
			return HAL_OK;

		case(0x305):	// Read EBS_State, Service_Brake_State from ASB Board
			dv->DV_Status->EBS_state 	   = dv->can_rx[0];
			dv->DV_Status->ServBrake_state = dv->can_rx[1];
			return HAL_OK;

		case(0x303):	// ASB status condition
			dv->DV_Status->ASB_status = (dv->can_rx[0] & 0x0F);
			dv->DV_Status->sd_is_open = ((dv->can_rx[0] & 0x10) >> 4);
			return HAL_OK;
	}

	return HAL_OK;
}


/* In case of Locked message, send ACK to PC in order
 * to properly lock the mission. If everything checks OK,
 * send ACK to Dash to */
void LockedMessageCheck(DV *dv, uint32_t tick0)
{
	while((xTaskGetTickCount() - tick0 < PC_WAIT_TIME) &&
			((dv->DV_Status->dv_says_mission != dv->DV_Status->mission_selected)))
	{
		// While the mission that the PC read is different from the mission P23 selected
		HAL_GPIO_TogglePin(GPIOB, USB2CAN_IND_Pin);

		SendMissionToPC(dv);
		osDelay(100);
	}
}


/* In case of Unlocked message */
void UnlockedMessageCheck(DV *dv, uint32_t tick0)
{
	while((xTaskGetTickCount() - tick0 < PC_WAIT_TIME)
			&& (dv->DV_Status->mission_selected == NOT_SELECTED)
			&& (dv->DV_Status->dv_says_lock     == LOCKED)
			&& (dv->DV_Status->mission_locked   == UNLOCKED))
	{
		/* If mission is not selected, unlocked, but the PC is locked.
		 * Generally we do not want to enter is state since it is not
		 * preferable to change the mission of the PC. */
		HAL_GPIO_TogglePin(GPIOB, USB2CAN_IND_Pin);

		/* Reset UnLocked message to lock again */
		dv->DV_Status->PC_Locked_Mission = NOT_SELECTED;

		/* Receive feedback from PC */
		SendMissionToPC(dv);

		osDelay(100);
	}
}


/* If PC doesn't communicate properly with CAN2USB to lock the mission
 * CAN2USB indicates error to Dash, else lock mission */
void MissionACKCheck(DV *dv)
{
	if(dv->DV_Status->dv_says_mission != dv->DV_Status->mission_selected)
		dv->DV_Status->dv_says_mission = NOT_SELECTED;
	else
		Mission_ACK(dv);
}


/* Send to DashBoard the mission message for Acknowledgment, only if PC has answered properly */
uint8_t Mission_ACK(DV *dv)
{
	dv->can_tx[0] = (uint8_t)((dv->DV_Status->mission_selected)|(dv->DV_Status->mission_locked));

	if(AutoCAN_Tx(dv, CAN2USB_ID3, 8, dv->can_tx) != HAL_OK)
		return HAL_ERROR;

	return HAL_OK;
}


/* Send mission to USB */
void SendMissionToPC(DV *dv)
{
	char    RxSp[100];
	uint8_t RxData[3];

	RxData[0] = 0x03;
	RxData[1] = 0x04;
	RxData[2] = 1;
	RxData[3] = dv->full_mission;

	/* Transmit message over USB, concerning MissionSelection:
	 * Send same ID to DV PC */
	sprintf(RxSp, "%02x%02x%02x%02x\n", RxData[0], RxData[1], RxData[2], RxData[3]);
	CDC_Transmit_FS((uint8_t*)RxSp, strlen(RxSp));
	osDelay(1);
}


