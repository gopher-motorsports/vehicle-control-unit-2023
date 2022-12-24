/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_VCU_H_
#define INC_VCU_H_

#include "stm32f4xx_hal.h"

void init(CAN_HandleTypeDef* hcan_ptr);
void main_loop();
void can_buffer_handling_loop();

void update_inverter_state();
void update_tractive_system_state();
void update_cooling();
void run_safety_checks();
void update_brake_light();
void send_torque_command();
void update_driver_display();

typedef enum
{
	NOT_READY_STATE = 0,
	READY_STATE = 1
} DRIVE_STATE_t;

#endif /* INC_VCU_H_ */
