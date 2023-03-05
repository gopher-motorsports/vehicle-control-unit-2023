/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_VCU_H_
#define INC_VCU_H_

#include "stm32f4xx_hal.h"

typedef enum
{
	// Made it INV_LCKOUT because of conflict with INV_CTRL_STATE_t in inverter_can.h
	INV_LCKOUT = 0,
	INV_FAULT = 1,
	INV_READY = 2,
	BUZZING = 3
} DRIVE_STATE_t;

void init(CAN_HandleTypeDef* hcan_ptr);
void main_loop();
void can_buffer_handling_loop();

void control_cooling();
void handle_CAN();
void handle_inputs();
void brake_light();
void handle_inv(DRIVE_STATE_t *curr_state);
void handle_buzzer(DRIVE_STATE_t *curr_state);
void check_inv_lockout(DRIVE_STATE_t *curr_state);
void check_RTD_button(DRIVE_STATE_t *curr_state);

//OLD FUNCTIONS, not deleting for now in case we want to reference
void update_inverter_state();
void update_tractive_system_state();
void update_cooling();
void run_safety_checks();
void update_brake_light();
void send_torque_command();
void update_driver_display();

#endif /* INC_VCU_H_ */
