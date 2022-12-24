/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include "vcu.h"
#include "GopherCAN.h"
#include "inverter_can.h"
#include "cmsis_os.h"

// The HAL_CAN struct
CAN_HandleTypeDef* hcan;


// Use this to define what module this board will be
#define THIS_MODULE_ID VCU_ID

//INTERNAL_STATES_t inv_states = {0};
//CMD_CAN_t CAN_cmd =
//{
//		.torque_cmd = 0,
//		.spd_cmd = 0,
//		.dir_cmd = 0,
//		.inverter_en = 0,
//		.discharge_en = 0,
//		.speed_en = 0,
//		.torque_lim = MAX_TORQUE_Nm,
//};
//INV_RW_CMD_CAN_t rw_cmd = {0};
//INV_CTRL_STATE_t inv_ctrl_state = INV_LOCKOUT;
//extern CAN_HandleTypeDef hcan2;

void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	// Initialization code goes here
	// TODO: Initialization code/functions
	// TODO: Validate inverter config
}

void main_loop() {
	update_inverter_state();
	update_tractive_system_state();
	update_cooling();
	run_safety_checks();
	update_brake_light();
	send_torque_command();
	update_driver_display();
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


/**
 * Manages the inverter state (lockout, configuration)
 */
void update_inverter_state() {
	//Updating inverter state
	DRIVE_STATE_t curr_state = 0;
	handle_inverter(&curr_state);

	// TODO: Validate inverter configuration?
	// TODO: Lockout mode state management
	// TODO: What else needs to be done?
}

/**
 *
 */
void update_tractive_system_state() {
	// TODO: Manage tractive system state
	// TODO: Move send_torque command into this function?
	// TODO: Check if brake is pressed for RTD functionality
}

/**
 *
 */
void update_cooling() {
	// TODO: Cooling system management

}

/**
 *
 */
void run_safety_checks() {
	// TODO: APPS correlation check
	// TODO: Braking/APPS check
	// TODO: Sensor input checks (use pull-ups to validate BSPD functionality)
	// TODO: Clamp current draw if it is out of range for hard braking to prevent BSPD faults
}

/**
 *
 */
void update_brake_light() {
	// TODO: Control brake light output
}

/**
 *
 */
void send_torque_command() {
	// TODO: CAN command with the inverter
}

/**
 *
 */
void update_driver_display() {
	//TODO: CAN communication with AEM CD-5 display
}
