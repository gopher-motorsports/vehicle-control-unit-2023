/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_VCU_H_
#define INC_VCU_H_

// Brake pressure threshold is set at 0.75 so there's room for error
#define BRAKE_PRESS_THRESH 0.75
// Temperature thresholds still not entirely decided yet, 100 is a placeholder
#define BRAKE_TEMP_THRESH 100
#define MOTOR_TEMP_THRESH 100

#include "stm32f4xx_hal.h"

typedef enum
{
	INV_LOCKOUT_STATE = 0,
	INV_FAULT_STATE = 1,
	INV_READY_STATE = 2,
	BUZZING_STATE = 3
} DRIVE_STATE_t;

extern DRIVE_STATE_t curr_state;

void init(CAN_HandleTypeDef* hcan_ptr);
void main_loop();
void can_buffer_handling_loop();

void control_cooling();
void handle_CAN();
void handle_inputs();
void brake_light();
void handle_inv();
void handle_buzzer();
void check_inv_lockout();
void check_RTD_button();

//OLD FUNCTIONS, not deleting for now in case we want to reference
void update_inverter_state();
void update_tractive_system_state();
void update_cooling();
void run_safety_checks();
void update_brake_light();
void send_torque_command();
void update_driver_display();

#endif /* INC_VCU_H_ */
