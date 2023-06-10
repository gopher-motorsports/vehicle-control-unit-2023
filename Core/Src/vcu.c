/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include "inverter.h"
#include "vcu.h"
#include "main.h"
#include "GopherCAN.h"
#include "gopher_sense.h"
#include <stdlib.h>

// The HAL_CAN struct
CAN_HandleTypeDef* hcan;

uint32_t maxCurrent_mA = 0;
uint32_t predriveTimer_ms = 0;
uint32_t desiredTorque_Nm = 0;
uint32_t torqueLimit_Nm = 0;

uint16_t apps1FaultTimer_ms = 0;
uint16_t apps2FaultTimer_ms = 0;
uint16_t brakeSensorFaultTimer_ms = 0;
uint16_t currentSensorFaultTimer_ms = 0;

uint16_t correlationTimer_ms = 0;

bool appsBrakeLatched_state = 0;

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(GCAN1, hcan, VCU_ID, BXTYPE_MASTER);

	// TODO: Initialization code/functions
	// TODO: Validate inverter config

}

void main_loop() {
	check_ready_to_drive();
	run_safety_checks();
	process_inverter();
	update_outputs();
	update_cooling();
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

void run_safety_checks() {
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
		appsBrakeLatched_state = true;
	} else if (pedalPosition1_mm.data <= APPS_BRAKE_RESET_THRESH_mm) {
		appsBrakeLatched_state = false;
	}

	if(appsBrakeLatched_state) {
		torqueLimit_Nm = 0;
	}
}


void process_inverter() {
	if(HAL_GetTick() - inverterState_state.info.last_rx > INVERTER_TIMEOUT_ms) {
		vehicle_state = VEHICLE_STARTUP;
	}
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

	if(vehicle_state == VEHICLE_DRIVING) {
		// Send a torque command
		update_and_queue_param_float(&torqueCmd_Nm, desiredTorque_Nm);
		update_and_queue_param_float(&speedCmd_rpm, 0);
		update_and_queue_param_u8(&cmdDir_state, (U8)(MOTOR_DIRECTION > 0));
		update_and_queue_param_u8(&invCmdFlags_state, INVERTER_ENABLE);
		update_and_queue_param_float(&torqueCmdLim_Nm, MAX_CMD_TORQUE_Nm);
		service_can_tx(hcan);
	} else {
		// Tell the inverter we don't want the motor to spin
		// This will also exit lockout
		update_and_queue_param_float(&torqueCmd_Nm, 0);
		update_and_queue_param_float(&speedCmd_rpm, 0);
		update_and_queue_param_u8(&cmdDir_state, (U8)(MOTOR_DIRECTION > 0));
		update_and_queue_param_u8(&invCmdFlags_state, INVERTER_DISABLE);
		update_and_queue_param_float(&torqueCmdLim_Nm, MAX_CMD_TORQUE_Nm);
		service_can_tx(hcan);
	}

	if(invStatesByte6_state.data & INVERTER_LOCKOUT) {
		// We've exited lockout
		vehicle_state = VEHICLE_STANDBY;
	} else {
		// We're in lockout; reset faults
		update_and_queue_param_u16(&invParameterAddress_state, PARAM_CMD_FAULT_CLEAR);
		update_and_queue_param_u8(&invParameterRW_state, PARAM_CMD_WRITE);
		update_and_queue_param_u8(&invParameterReserved1_state, PARAM_CMD_RESERVED1);
		update_and_queue_param_u16(&invParameterData_state, PARAM_FAULT_CLEAR_DATA);
		update_and_queue_param_u16(&invParameterReserved2_state, PARAM_CMD_RESERVED2);
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
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	}

	if(brakePressureFront_psi.data > BRAKE_LIGHT_THRESH_psi
			|| brakePressureRear_psi.data > BRAKE_LIGHT_THRESH_psi) {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, GPIO_PIN_RESET);
	}
	return;
}

void check_RTD_button() {
	// Forgot to assign a pin to the RTD button, using GPIO_1 for it
	// HAL_GPIO_ReadPin is a pull up pin, so it will return 0 if the button is pressed
	if (!HAL_GPIO_ReadPin(GPIO1_GPIO_Port, GPIO1_Pin)
			&& vehicle_state == VEHICLE_STANDBY
			&& brakePressureFront_psi.data > PREDRIVE_BRAKE_THRESH_psi
			&& brakePressureRear_psi.data > PREDRIVE_BRAKE_THRESH_psi) {
		// Button is pressed, set state to VEHICLE_PREDRIVE
		vehicle_state = VEHICLE_PREDRIVE;
	}
}
