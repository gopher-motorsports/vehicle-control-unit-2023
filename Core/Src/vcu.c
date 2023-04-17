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

void main_loop() {
	control_cooling();
	handle_CAN();
	handle_inputs();
	brake_light();
	handle_inv();
	handle_buzzer();
	// Main function
	check_inv_lockout();
	check_RTD_button();
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
	// Pump should always be on at least a little bit, making sure that's the case (RESET turns the pin on)
	HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_RESET);

	// TODO: Ramp up cooling based on temperatures of the inverter and motor using PWM
	// TODO: Set pump and fan pins to PWM
	if(brakeTempFrontLeft_C.data >= BRAKE_TEMP_THRESH || brakeTempFrontRight_C.data >= BRAKE_TEMP_THRESH || motorTemp_C.data >= MOTOR_TEMP_THRESH) {
		HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
	}
}

void handle_CAN() {
	// TODO: Write
}

void handle_inputs() {
	// TODO: Write
}

void brake_light() {
	// If brakePressureRear_psi.data is over the brake pressure threshold, the brake is pressed
	// and we need to turn on the brake light
	if(brakePressureRear_psi.data > BRAKE_PRESS_THRESH) {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, GPIO_PIN_SET);
	// Else, the brake isn't pressed and we need to turn the brake light off
	} else {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, GPIO_PIN_RESET);
	}
	return;
}

void handle_inv() {

	// TODO: Edit the logic here

	U8 msg_data[8];
	U16 torque_req = 0;

	//There's 4 states that the inverter could be in at any given time:
	// -INV_LCKOUT
	// -INV_FAULT
	// -INV_READY
	// -BUZZING

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
	switch (curr_state) {
		//TODO: Edit this to match current params
		case INV_LOCKOUT_STATE:
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

		case INV_FAULT_STATE:
			// try to enable the motor
			if (curr_state == INV_READY_STATE)
			{
				CAN_cmd.inverter_en = 1;
				build_cmd_msg(msg_data, &CAN_cmd);
				send_can_message(COMMAND_MSG_ID, 8, msg_data);
			}
			break;

		case INV_READY_STATE:
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
					// TODO: linearly interpolate
				}
			}

			CAN_cmd.torque_cmd = torque_req;
			build_cmd_msg(msg_data, &CAN_cmd);
			send_can_message(COMMAND_MSG_ID, 8, msg_data);
			break;

		default:
			// not sure how we are here
			inv_ctrl_state = INV_LOCKOUT_STATE;
			break;
		}
}

void handle_buzzer() {
	if(curr_state == BUZZING_STATE) {
		HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, GPIO_PIN_SET);
		// TODO: Figure out how many ticks five seconds is. 5000 is placeholder.
		osDelay(5000);
		HAL_GPIO_WritePin(RTD_BUZZER_GPIO_Port, RTD_BUZZER_Pin, GPIO_PIN_RESET);
	}
}

void check_inv_lockout() {
	if (curr_state) {

	}
}

void check_RTD_button() {
	// Forgot to assign a pin to the RTD button, using GPIO_1 for it
	// HAL_GPIO_ReadPin is a pull up pin, so it will return 0 if the button is pressed
	if (!HAL_GPIO_ReadPin(GPIO_1_GPIO_Port, GPIO_1_Pin)) {
		//Button is pressed, set state to BUZZING_STATE
		curr_state = BUZZING_STATE;
	}
}
