/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include "vcu.h"
#include "main.h"
#include "GopherCAN.h"
#include "inverter_can.h"
#include "inverter_can.c"

// The HAL_CAN struct
CAN_HandleTypeDef* hcan;


// Use this to define what module this board will be
#define THIS_MODULE_ID VCU_ID


void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	// Initialization code goes here
	// TODO: Initialization code/functions
	// TODO: Validate inverter config

}

DRIVE_STATE_t* main_loop(DRIVE_STATE_t* curr_state) {
	control_cooling();
	handle_CAN();
	handle_inputs();
	brake_light();
	handle_inv(curr_state);
	handle_buzzer(curr_state);
	// Main function
	check_inv_lockout(curr_state);
	check_RTD_button(curr_state);

	return curr_state;

//  Old function setup, keeping them here just in case
//	update_inverter_state();
//	update_tractive_system_state();
//	update_cooling();
//	run_safety_checks();
//	update_brake_light();
//	send_torque_command();
//	update_driver_display();
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
	service_can_tx_hardware(hcan);
}

void control_cooling() {
	// TODO be smart about when turning on pump and fan
	HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);

	// TODO: Dynamic cooling system management
}

void handle_CAN() {

}

void handle_inputs() {

}

void brake_light() {
//	if(HAL_GPIO_ReadPin(BRK_PRESS_GPIO_Port, BRK_PRESS_Pin) == GPIO_PIN_Set) {
//		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, GPIO_PIN_RESET);
//	}
}

void handle_inv(DRIVE_STATE_t *curr_state) {
	U8 msg_data[8];
	U16 torque_req = 0;

	//update_inv_state();

	// find the state of the inverter based on the status of the ECU
//	if (inv_states.inv_enable_lockout)
//	{
//		inv_ctrl_state = INV_LOCKOUT;
//	}
//	else if (!inv_states.inv_en_state)
//	{
//		// not locked out, not enabled
//		inv_ctrl_state = INV_DISABLED;
//
//		// disable the state
//		*curr_state = NOT_READY_STATE;
//	}
//	else
//	{
//		inv_ctrl_state = INV_ENABLED;
//	}

	// if the inverter is locked out, we must first disable and enable the motor
	switch (*curr_state) {
		case INV_LOCKOUT:
			// send the disable then enable command with some delay
			CAN_cmd.inverter_en = 0;
			build_cmd_msg(msg_data, &CAN_cmd);
			send_can_message(COMMAND_MSG_ID, 8, msg_data);

			// also clear all faults when first starting up
			rw_cmd.param_addr = FAULT_CLEAR;
			rw_cmd.read_or_write = WRITE_CMD;
			rw_cmd.data = 0;
			build_param_cmd_msg(msg_data, &rw_cmd);
			send_can_message(PARAM_RW_CMD_ID, 8, msg_data);
			break;

		case INV_DISABLED:
			// try to enable the motor
			if (*curr_state == INV_READY)
			{
				CAN_cmd.inverter_en = 1;
				build_cmd_msg(msg_data, &CAN_cmd);
				send_can_message(COMMAND_MSG_ID, 8, msg_data);
			}
			break;

		case INV_ENABLED:
			// actually run the motor

			// calculate the torque we want from the motor. Right now we are linear
			if (brakePressureRear_psi.data <= BRAKE_PRESSURE_BREAK_THRESH_psi)
			{
				// Old command was "float curr_pp = vcu_apps2.data;"
				float curr_pp =pedalPosition2_mm.data;
				if (curr_pp >= APPS2_MAX_TRAVEL_mm)
				{
					torque_req = MAX_TORQUE_Nm;
				}
				else if (curr_pp <= APPS2_MIN_TRAVEL_mm)
				{
					torque_req = 0;
				}
				else
				{
					// linearly interpolate
					// TODO
				}
			}

			CAN_cmd.torque_cmd = torque_req;
			build_cmd_msg(msg_data, &CAN_cmd);
			send_can_message(COMMAND_MSG_ID, 8, msg_data);
			break;

		default:
			// not sure how we are here
			inv_ctrl_state = INV_LOCKOUT;
			break;
		}
}

void handle_buzzer(DRIVE_STATE_t *curr_state) {
	// TODO: Check that the RTD button has been pressed
	if(*curr_state == BUZZING) {
		HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, GPIO_PIN_SET);
		// Not sure how many seconds a tick is
		osDelay(5000);
		HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, GPIO_PIN_RESET);
	}
}

void check_inv_lockout(DRIVE_STATE_t *curr_state) {
	if (*curr_state) {

	}
}

void check_RTD_button(DRIVE_STATE_t *curr_state) {
	// Forgot to assign a pin to the RTD button, using GPIO_1 for it
	GPIO_PinState rtd_pressed = HAL_GPIO_ReadPin(GPIO_1_GPIO_Port, GPIO_1_Pin);

	if(rtd_pressed == GPIO_PIN_SET) {

	}
}

//Old functions, keeping for now

/**
 * Manages the inverter state (lockout, configuration)
 */
void update_inverter_state() {
	// TODO: Validate inverter configuration?
	// TODO: Lockout mode state management
	// TODO: What else needs to be done?
}

/**
 * Updates the state of the Tractive System
 */
void update_tractive_system_state() {
	// TODO: Manage tractive system state
	// TODO: Move send_torque command into this function?
	// TODO: Check if brake is pressed for RTD functionality
}

/**
 * Updates the cooling system on the car
 */
void update_cooling() {
	// TODO: Dynamic cooling system management
}

/**
 * Responsible for sensor range checks, APPS correlation checks,
 * and several other checks related to safety
 */
void run_safety_checks() {
        //brake pressure / pedal pressure chek to see that both aren't being pushed
		for(;;)
		{
			int pressure_sensor = 0;
			int brake_pressure_sensor = 0;
			brake_pressure_sensor.data;
			if (brake_pressure_sensor >= 0){
				pressure_sensor.data;
				if (pressure_sensor >= 500) { //we still need to calculate this constant, 2000 was the highest psi value.
					turn_inverter_off();
					HAL_CAN_AddTxMessage(); //maybe tell the display this check failed or maybe flash led,
				}
				if (pressure_sensor < 500) {
					osDelay(0);
				}
			}
			else {
				osDelay(0);
			}

		}

		//APPS correlation check / plausibility check
		for(;;)
		{
			break_pressure_rear.data;
			pressure_sensor.data;
			int brake_pressure_rear = 0;
			int a = brake_pressure_rear;
			int pressure_sensor = 0;
			int b = pressure_sensor;
			int c = 1;
			//to make sure the the larger number is in the denominator
			if (a > b);
				c = ((a-b)/a) * 100;
				if (c >= 10);
					turn_off_inverter();
				if (c < 10);
					osDelay(0);

			if (b > a);
				c = ((b-a)/b) * 100;
				if (c >= 10);
					tunr_off_inverter();
				if (c < 10);
					turn_off_inverter();

			if (a = b);
				osDelay(0);


		}

		//AAPS short detection?
		for (;;)
		{
			int a = apps_signal;
			apps1.data;

			if (apps_signal < 0.264);
				send_torque_command = 0;
			if (apps_signal > 3.04);
				send_torque_command = 0;
			else (void);
				send_torque_command = 1;
		}


	    // BSE short check

		for (;;)
		{
			int a = bse_signal;
			bse1.data;

			if (bse_signal < 0.264);
				send_torque_command = 0; //use the boolean true statements for either
			if (bse_signal > 3.2);
				send_torque_command = 0;
			else (void);
				send_torque_comand = 1;
		}
		// Braking/Tractive System current check
		for (;;)
		{
			int brake_pressure_rear;
			if (brake_pressure_rear > 0); //change to account for error
				ts_sns_1 = 0;
			if (brake_pressure_rear < 0);
				ts_sns_1 = 1;

		}
		}
}

/**
 * Updates the state of the brake light based on the sensed brake pressure
 */
void update_brake_light() {
	// TODO: Control brake light output
	int brake_pressure = 0;
	HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, brake_pressure > 100 ? SET : RESET);
}

/**
 * Sends a torque command to the inverter
 */
void send_torque_command() {
	// TODO: CAN command with the inverter
}

void update_driver_display() {

}
