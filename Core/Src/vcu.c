/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include "vcu.h"
#include "gopher_sense.h"
#include <stdlib.h>

// The HAL_CAN struct
CAN_HandleTypeDef* hcan;

uint32_t maxCurrent_mA = 0;
uint32_t preDriveTimer_ms = 0;
float desiredTorque_Nm = 0;
float torqueLimit_Nm = 0;

uint16_t apps1FaultTimer_ms = 0;
uint16_t apps2FaultTimer_ms = 0;
uint16_t brakeSensorFaultTimer_ms = 0;
uint16_t currentSensorFaultTimer_ms = 0;

uint16_t correlationTimer_ms = 0;

boolean appsBrakeLatched_state = 0;

boolean readyToDriveButtonPressed_state = 0;

VEHICLE_STATE_t vehicle_state = VEHICLE_STARTUP;

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(GCAN1, hcan, VCU_ID, BXTYPE_SLAVE);

	// TODO: Initialization code/functions
	// TODO: Validate inverter config

}

void main_loop() {
	update_RTD();
	process_sensors();
	process_inverter();
	update_outputs();
	update_cooling();
	update_display_fault_status();
	update_gcan_states(); // Should be after process_sensors

	// Turn off RGB
	HAL_GPIO_WritePin(STATUS_R_GPIO_Port, STATUS_R_Pin, SET);
	HAL_GPIO_WritePin(STATUS_G_GPIO_Port, STATUS_G_Pin, SET);
	HAL_GPIO_WritePin(STATUS_B_GPIO_Port, STATUS_B_Pin, SET);
}

/**
 * Services the CAN RX and TX hardware task
 */
void can_buffer_handling_loop()
{
	// Handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// An error has occurred
	}

	// Handle the transmission hardware for each CAN bus
	service_can_tx(hcan);
}

void update_gcan_states() {
	// Log BSPD sensor faults
	update_and_queue_param_u8(&bspdPedalPosition1Fault_state,
			HAL_GPIO_ReadPin(BSPD_APPS1_FAULT_GPIO_Port, BSPD_APPS1_FAULT_Pin) == BSPD_APPS1_FAULT);
	update_and_queue_param_u8(&bspdPedalPosition2Fault_state,
			HAL_GPIO_ReadPin(BSPD_APPS2_FAULT_GPIO_Port, BSPD_APPS2_FAULT_Pin) == BSPD_APPS2_FAULT);
	update_and_queue_param_u8(&bspdBrakePressureSensorFault_state,
			HAL_GPIO_ReadPin(BSPD_BRK_FAULT_GPIO_Port, BSPD_BRK_FAULT_Pin) == BSPD_BRAKE_FAULT);
	update_and_queue_param_u8(&bspdTractiveSystemCurrentSensorFault_state,
			HAL_GPIO_ReadPin(BSPD_TS_SNS_FAULT_GPIO_Port, BSPD_TS_SNS_FAULT_Pin) == BSPD_TS_SNS_FAULT);
	// Log BSPD current/braking fault
	update_and_queue_param_u8(&bspdTractiveSystemBrakingFault_state,
			HAL_GPIO_ReadPin(BSPD_TS_BRK_FAULT_GPIO_Port, BSPD_TS_BRK_FAULT_Pin) == BSPD_TS_BRK_FAULT);
	// VSC sensors faults
	update_and_queue_param_u8(&vcuPedalPosition1Fault_state, apps1FaultTimer_ms > INPUT_TRIP_DELAY_ms);
	update_and_queue_param_u8(&vcuPedalPosition2Fault_state, apps2FaultTimer_ms > INPUT_TRIP_DELAY_ms);
	update_and_queue_param_u8(&vcuBrakePressureSensorFault_state, brakeSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms);
	update_and_queue_param_u8(&vcuTractiveSystemCurrentSensorFault_state, currentSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms);
	// VSC safety checks
	update_and_queue_param_u8(&vcuPedalPositionCorrelationFault_state, correlationTimer_ms > CORRELATION_TRIP_DELAY_ms);
	update_and_queue_param_u8(&vcuPedalPositionBrakingFault_state, appsBrakeLatched_state);
	// Requested torque
	update_and_queue_param_u8(&vcuTorqueRequested_Nm, desiredTorque_Nm);
	// Cooling
	update_and_queue_param_u8(&coolantFanPower_percent, 100*HAL_GPIO_ReadPin(FAN_GPIO_Port, FAN_Pin));
	update_and_queue_param_u8(&coolantPumpPower_percent, 100*HAL_GPIO_ReadPin(PUMP_GPIO_Port, PUMP_Pin));
	update_and_queue_param_u8(&accumulatorFanPower_percent, 100*HAL_GPIO_ReadPin(ACC_FAN_GPIO_Port, ACC_FAN_Pin));
	// Vehicle state
	update_and_queue_param_u8(&vehicleState_state, vehicle_state);
	update_and_queue_param_u8(&readyToDriveButton_state, readyToDriveButtonPressed_state);

	update_and_queue_param_u8(&vcuGSenseStatus_state, HAL_GPIO_ReadPin(GSENSE_LED_GPIO_Port, GSENSE_LED_Pin));
}

void update_cooling() {
	// Pump should always be on at least a little bit, making sure that's the case (RESET turns the pin on)
	HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);

	// TODO: Ramp up cooling based on temperatures of the inverter and motor using PWM
	// TODO: Set pump and fan pins to PWM
	if(igbtATemp_C.data >= IGBT_TEMP_THRESH_C
			|| igbtBTemp_C.data >= IGBT_TEMP_THRESH_C
			|| igbtCTemp_C.data >= IGBT_TEMP_THRESH_C
			|| gateDriverBoardTemp_C.data >= GDB_TEMP_THRESH_C
			|| controlBoardTemp_C.data >= CTRL_BOARD_TEMP_THRESH_C
			|| motorTemp_C.data >= MOTOR_TEMP_THRESH_C){
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
	}
}

void process_sensors() {
	torqueLimit_Nm = MAX_CMD_TORQUE_Nm;

	// Input Validation Checks
	if(pedalPosition1_mm.data > APPS_MAX_POS_mm
			|| pedalPosition1_mm.data < APPS_MIN_POS_mm) {
		apps1FaultTimer_ms = (apps1FaultTimer_ms > INPUT_TRIP_DELAY_ms) ? INPUT_TRIP_DELAY_ms + 1 : apps1FaultTimer_ms + 1;
	} else {
		apps1FaultTimer_ms = 0;
	}
	if(pedalPosition2_mm.data > APPS_MAX_POS_mm
				|| pedalPosition2_mm.data < APPS_MIN_POS_mm) {
		apps2FaultTimer_ms = (apps2FaultTimer_ms > INPUT_TRIP_DELAY_ms) ? INPUT_TRIP_DELAY_ms + 1 : apps2FaultTimer_ms + 1;
	} else {
		apps2FaultTimer_ms = 0;
	}
	if(brakePressureFront_psi.data > BRAKE_PRESS_MAX_psi
			|| brakePressureFront_psi.data < BRAKE_PRESS_MIN_psi) {
		brakeSensorFaultTimer_ms = (brakeSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms) ? INPUT_TRIP_DELAY_ms + 1 : brakeSensorFaultTimer_ms + 1;
	} else {
		brakeSensorFaultTimer_ms = 0;
	}
	if(vcuTractiveSystemCurrent_A.data > TS_CURRENT_MAX_A
			|| vcuTractiveSystemCurrent_A.data < TS_CURRENT_MIN_A) {
		currentSensorFaultTimer_ms = (currentSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms) ? INPUT_TRIP_DELAY_ms + 1 : currentSensorFaultTimer_ms + 1;
	} else {
		currentSensorFaultTimer_ms = 0;
	}
	if(apps1FaultTimer_ms > INPUT_TRIP_DELAY_ms
			|| apps2FaultTimer_ms > INPUT_TRIP_DELAY_ms
			|| brakeSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms
			|| currentSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms) {
		torqueLimit_Nm = 0;
	}

	// Correlation Check
	if(pedalPosition1_mm.data - pedalPosition2_mm.data > APPS_CORRELATION_THRESH_mm
			|| pedalPosition2_mm.data - pedalPosition1_mm.data > APPS_CORRELATION_THRESH_mm) {
		correlationTimer_ms = (correlationTimer_ms > CORRELATION_TRIP_DELAY_ms) ? CORRELATION_TRIP_DELAY_ms + 1 : currentSensorFaultTimer_ms + 1;
	} else {
		correlationTimer_ms = 0;
	}
	if(correlationTimer_ms > CORRELATION_TRIP_DELAY_ms) {
		torqueLimit_Nm = 0;
	}

	// APPS/Braking Check
	if((brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi
			&& pedalPosition1_mm.data > APPS_BRAKE_APPS1_THRESH_mm)) {
		appsBrakeLatched_state = TRUE;
	} else if (pedalPosition1_mm.data <= APPS_BRAKE_RESET_THRESH_mm) {
		appsBrakeLatched_state = FALSE;
	}

	if(appsBrakeLatched_state) {
		torqueLimit_Nm = 0;
	}

	if(bspdTractiveSystemBrakingFault_state.data) {
		float tractiveSystemBrakingLimit_Nm = 0;
		float accumulatorVoltage_V = seg1Voltage_V.data +
							seg2Voltage_V.data +
							seg3Voltage_V.data +
							seg4Voltage_V.data +
							seg5Voltage_V.data +
							seg6Voltage_V.data +
							seg7Voltage_V.data;
		if(motorSpeed_rpm.data != 0) {
			// Calculate max torque from speed and voltage, using angular velocity (rad/s)
			tractiveSystemBrakingLimit_Nm = (BRAKE_TS_CURRENT_THRESH_A * accumulatorVoltage_V)
					/ ((motorSpeed_rpm.data * MATH_TAU) / SECONDS_PER_MIN);
		}
		// If the tractive system braking limit is less (more restrictive),
		// then set the torque limit to that amount
		if(tractiveSystemBrakingLimit_Nm < torqueLimit_Nm) {
			update_and_queue_param_u8(&vcuBrakingClampingCurrent_state, TRUE);
			torqueLimit_Nm = tractiveSystemBrakingLimit_Nm;
		} else {
			update_and_queue_param_u8(&vcuBrakingClampingCurrent_state, FALSE);
		}
	}

	desiredTorque_Nm = ((pedalPosition1_mm.data-APPS_MIN_POS_mm)/APPS_TOTAL_TRAVEL_mm)*MAX_CMD_TORQUE_Nm;

	if(pedalPosition1_mm.data < APPS_MIN_POS_mm) {
		desiredTorque_Nm = 0;
	}

	if(desiredTorque_Nm > torqueLimit_Nm) {
		desiredTorque_Nm = torqueLimit_Nm;
	}
}

void update_display_fault_status() {
	int status = NONE;
	if(amsFault_state.data) status = AMS_FAULT;
	else if(bmsNumActiveAlerts_state.data) status = BMS_FAULT;
	else if(vcuPedalPositionBrakingFault_state.data) status = RELEASE_PEDAL;
	else if(bspdTractiveSystemBrakingFault_state.data || vcuBrakingClampingCurrent_state.data) status = BREAKING_FAULT;
	else if(vcuPedalPositionCorrelationFault_state.data) status = APPS_FAULT;
	else if(bspdFault_state.data
			|| bspdBrakePressureSensorFault_state.data
			|| bspdPedalPosition1Fault_state.data
			|| bspdPedalPosition2Fault_state.data
			|| bspdTractiveSystemCurrentSensorFault_state.data
			) status = BSPD_FAULT;
	else if(vcuBrakePressureSensorFault_state.data
			|| vcuPedalPosition2Fault_state.data
			|| vcuTractiveSystemCurrentSensorFault_state.data
			) status = VCU_FAULT;

	update_and_queue_param_u8(&displayFaultStatus_state, status);
}


void process_inverter() {
	if(HAL_GetTick() - inverterState_state.info.last_rx > INVERTER_TIMEOUT_ms) {
		vehicle_state = VEHICLE_STARTUP;
	}
	// Only do the state-change logic if we've actually talked with the inverter
	if(inverterState_state.info.last_rx != 0) {
		if(vehicle_state == VEHICLE_STARTUP && ((HAL_GetTick() - inverterState_state.info.last_rx) < INVERTER_TIMEOUT_ms)) {
			vehicle_state = VEHICLE_LOCKOUT;
		}
		if(invStatesByte4_state.data & 0x01) {
			// TODO: ADD AN ERROR CODE IF WE ARE IN SPEED MODE
		}
		// If we are in the driving state and the inverter is disabled,
		// go to the standby state.  This has to be after the startup stuff
		// so that this case will only be hit if the inverter is not in lockout
		if((vehicle_state == VEHICLE_DRIVING || vehicle_state == VEHICLE_PREDRIVE)
			&& !(invStatesByte6_state.data & INVERTER_ENABLE)) {
			vehicle_state = VEHICLE_STANDBY;
		}

		if((invStatesByte6_state.data & INVERTER_LOCKOUT)) {
			// We're in lockout; reset faults
			// TODO: Fix with send_param
			invParameterAddress_state.data = PARAM_CMD_FAULT_CLEAR;
			invParameterRW_state.data = PARAM_CMD_WRITE;
			invParameterReserved1_state.data = PARAM_CMD_RESERVED1;
			invParameterData_state.data = PARAM_FAULT_CLEAR_DATA;
			invParameterReserved2_state.data = PARAM_CMD_RESERVED2;
			send_parameter(&(invParameterAddress_state.info));
			send_parameter(&(invParameterRW_state.info));
			send_parameter(&(invParameterReserved1_state.info));
			send_parameter(&(invParameterData_state.info));
			send_parameter(&(invParameterReserved2_state.info));
			service_can_tx(hcan);
			vehicle_state = VEHICLE_LOCKOUT;
		} else {
			// We've exited lockout
			vehicle_state = VEHICLE_STANDBY;
		}
	}

	if(vehicle_state == VEHICLE_DRIVING) {
		// Send a torque command
		torqueCmd_Nm.data = desiredTorque_Nm;
		speedCmd_rpm.data = 0;
		cmdDir_state.data = (U8)(MOTOR_DIRECTION > 0);
		invCmdFlags_state.data = INVERTER_ENABLE;
		torqueCmdLim_Nm.data = torqueLimit_Nm;
		send_parameter(&(torqueCmd_Nm.info));
		send_parameter(&(speedCmd_rpm.info));
		send_parameter(&(cmdDir_state.info));
		send_parameter(&(invCmdFlags_state.info));
		send_parameter(&(torqueCmdLim_Nm.info));
		service_can_tx(hcan);
	} else {
		// Tell the inverter we don't want the motor to spin
		// This will also attempt to exit lockout
		torqueCmd_Nm.data = desiredTorque_Nm;
		speedCmd_rpm.data = 0;
		cmdDir_state.data = (U8)(MOTOR_DIRECTION > 0);
		invCmdFlags_state.data = INVERTER_DISABLE;
		torqueCmdLim_Nm.data = torqueLimit_Nm;
		send_parameter(&(torqueCmd_Nm.info));
		send_parameter(&(speedCmd_rpm.info));
		send_parameter(&(cmdDir_state.info));
		send_parameter(&(invCmdFlags_state.info));
		send_parameter(&(torqueCmdLim_Nm.info));
		service_can_tx(hcan);
	}
}

void handle_CAN() {
	// TODO: Write
}

void check_ready_to_drive() {
	if(brakePressureFront_psi.data > PREDRIVE_BRAKE_THRESH_psi
			&& brakePressureRear_psi.data > PREDRIVE_BRAKE_THRESH_psi
			&& readyToDriveButton_state.data == PREDRIVE_BUTTON_PRESSED
			&& vehicle_state == VEHICLE_STANDBY) {
		vehicle_state = VEHICLE_PREDRIVE;
	}
}

void update_outputs() {
	if(vehicle_state == VEHICLE_PREDRIVE) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, BUZZER_ON);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, BUZZER_OFF);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, FALSE);
	}

	if(brakePressureFront_psi.data > BRAKE_LIGHT_THRESH_psi
			|| brakePressureRear_psi.data > BRAKE_LIGHT_THRESH_psi) {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, BRAKE_LIGHT_ON);
		update_and_queue_param_u8(&brakeLightOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, BRAKE_LIGHT_OFF);
		update_and_queue_param_u8(&brakeLightOn_state, FALSE);
	}
	return;
}

void update_RTD() {
	// Update the predrive timer
	if(vehicle_state == VEHICLE_PREDRIVE) {
		if(++preDriveTimer_ms > PREDRIVE_TIME_ms) {
			vehicle_state = VEHICLE_DRIVING;
		}
	}
	readyToDriveButtonPressed_state = HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin) == RTD_BUTTON_PUSHED;
	// Forgot to assign a pin to the RTD button, using GPIO_1 for it
	// HAL_GPIO_ReadPin is a pull up pin, so it will return 0 if the button is pressed
	if (readyToDriveButtonPressed_state
			&& vehicle_state == VEHICLE_STANDBY
			&& brakePressureFront_psi.data > PREDRIVE_BRAKE_THRESH_psi
			&& brakePressureRear_psi.data > PREDRIVE_BRAKE_THRESH_psi) {
		// Button is pressed, set state to VEHICLE_PREDRIVE
		vehicle_state = VEHICLE_PREDRIVE;
		preDriveTimer_ms = 0;
	}
}
