/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include "vcu.h"
#include "GopherCAN.h"

// The HAL_CAN struct
CAN_HandleTypeDef* hcan;


// Use this to define what module this board will be
#define THIS_MODULE_ID VCU_ID


void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	// Initialization code goes here
	// TODO: Initialization code/functions
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
