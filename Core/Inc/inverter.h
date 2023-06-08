/*
 * inverter_can.h
 *
 *  Created on: Jun 16, 2022
 *      Author: Cal
 */

#ifndef INVERTER_CAN_H
#define INVERTER_CAN_H

#include "base_types.h"
//#include "main_task.h"
#include "stdbool.h"
#include "stm32f4xx_it.h"

#include "main.h"

#define INVERTER_TASK_DELAY 1
#define MAX_TORQUE_Nm 140

typedef enum
{
	INV_LOCKOUT   = 0,
	INV_DISABLED  = 1,
	INV_ENABLED   = 2
} INV_CTRL_STATE_t;

typedef enum
{
	POWER_ON_STATE = 0,
	STOP_STATE = 1,
	OPEN_LOOP_STATE = 2,
	CLOSED_LOOP_STATE = 3,
	WAIT_STATE = 4,
	INTERNAL_STATE_5 = 5,
	INTERNAL_STATE_6 = 6,
	INTERNAL_STATE_7 = 7,
	IDLE_RUN_STATE = 8,
	IDLE_STOP_STATE = 9,
	INTERNAL_STATE_10 = 10,
	INTERNAL_STATE_11 = 11,
	INTERNAL_STATE_12 = 12,
} INV_STATE_t;

bool send_torque_command(void);

#endif /* INVERTER_CAN_H */
