/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_VCU_H_
#define INC_VCU_H_

// ===================================== VEHICLE PARAMETERS =====================================
#define SERIES_CELL_COUNT_ul   84
#define CELL_NOMINAL_VOLTAGE_V 3.6
#define NOMINAL_PACK_VOLTAGE_V ( SERIES_CELL_COUNT_ul * CELL_NOMINAL_VOLTAGE_V )
// ==============================================================================================

// ======================================= APPS PARAMETERS ======================================
#define APPS_MAX_POS_mm  45 // The position of the pedal at full travel
#define APPS_MIN_POS_mm  5  // The position of the pedal at rest
#define APPS_MARGIN_mm   2  // The margin on the top and bottom of the pedal for throttle mapping
// The total physical travel of the APPS
#define APPS_TOTAL_TRAVEL_mm ( APPS_MAX_POS_mm - APPS_MIN_POS_mm )
// ==============================================================================================

// ====================================== BRAKE PARAMETERS ======================================
#define BRAKE_PRESS_MIN_psi    -50   // The minimum value of the brake pressure sensor
#define BRAKE_PRESS_MAX_psi    2050  // The maximum value of the brake pressure sensor
#define BRAKE_LIGHT_THRESH_psi 150 // The pressure at which the brake light will be activated
// ==============================================================================================
// ============================= TRACTIVE SYSTEM CURRENT PARAMETERS =============================
#define TS_CURRENT_MIN_A   -85   // The minimum value of the current sensor
#define TS_CURRENT_MAX_A   85  // The maximum value of the current sensor
// ==============================================================================================

// ======================================= RTD PARAMETERS =======================================
#define PREDRIVE_BRAKE_THRESH_psi   100 // The minimum brake pressure to enter the driving state
// ==============================================================================================


// ====================================== SAFETY PARAMETERS =====================================
// -------------------------------------- Input Validation --------------------------------------
#define INPUT_TRIP_DELAY_ms 100  // The amount of time it takes an input fault to take effect

// -------------------------------------- APPS/Brake Check --------------------------------------
// This check is done using APPS1 (since APPS1 determines the applied torque) and the BSE
#define APPS_BRAKE_PRESS_THRESH_psi  50  // The minimum amount of brake pressure that will trip
// The minimum APPS position that will trip the APPS/Brake check
#define APPS_BRAKE_APPS1_THRESH_mm   ( APPS_TOTAL_TRAVEL * 0.25 ) + APPS_MIN_POS_mm
// The maximum APPS position that will reset the APPS/Brake check
#define APPS_BRAKE_RESET_THRESH_mm   ( APPS_TOTAL_TRAVEL * 0.05 ) + APPS_MIN_POS_mm

// ------------------------------------ APPS Correlation Check ----------------------------------
#define APPS_CORRELATION_THRESH_mm   ( APPS_TOTAL_TRAVEL_mm * 0.1 )
#define CORRELATION_TRIP_DELAY_ms 100  // The amount of time it takes a correlation fault to take effect

// ------------------------------------ TS Current/Brake Check ----------------------------------
#define BRAKE_TS_POWER_THRESH_W    5000
#define BRAKE_TS_PRESS_THRESH_psi  450
#define BRAKE_TS_CURRENT_THRESH_A  ( BRAKE_TS_POWER_THRESH_W / NOMINAL_PACK_VOLTAGE_V )
#define BRAKE_TS_DELAY_ms          350
// ==============================================================================================

// ====================================== COOLING PARAMETERS ====================================
#define IGBT_TEMP_THRESH_C        40 // Minimum IGBT temperature for cooling fan to turn on
#define GDB_TEMP_THRESH_C         40 // Minimum Gate Drive Board temp for cooling fan to turn on
#define CTRL_BOARD_TEMP_THRESH_C  40 // Minimum Control Board temp for cooling fan to turn on
#define MOTOR_TEMP_THRESH_C       30 // Minimum Motor temperature for cooling fan to turn on
// ==============================================================================================


// ================================== TRACTIVE SYSTEM PARAMETERS ================================
#define MOTOR_DIRECTION           1    // Motor direction; 0 is reverse, 1 is forward
#define MAX_CMD_TORQUE_Nm         150  // The maximum torque the
#define INVERTER_TIMEOUT_ms       100  // The time after which the vehicle state will be STARTUP
#define INVERTER_ENABLE           0x01 // Flags to enable the inverter
#define INVERTER_DISABLE          0x00 // Flags to disable the inverter
#define INVERTER_LOCKOUT          0x40 // Lockout is bit 7 of byte 6
// ==============================================================================================


#include "main.h"


// This is the vehicle state which is affected both by the actions
// of the driver and the current state of the inverter
typedef enum
{
	VEHICLE_STARTUP   = 0, // When the vehicle first turns on (no inverter communication)
	VEHICLE_LOCKOUT   = 1, // The vehicle can detect that the inverter is in lockout
	VEHICLE_STANDBY   = 2, // The inverter has exited lockout but no torque commands will be sent
	VEHICLE_PREDRIVE  = 3, // The vehicle buzzer is active and the driving state will be entered
	VEHICLE_DRIVING   = 4, // Torque commands are actively being sent from APPS positions
	VEHICLE_FAULT     = 5
} VEHICLE_STATE_t;

extern VEHICLE_STATE_t vehicle_state;

void init(CAN_HandleTypeDef* hcan_ptr);
void main_loop();
void can_buffer_handling_loop();

void update_cooling();    // Controls/updates the cooling system
void run_safety_checks(); // Runs safety checks on driver inputs
void update_outputs();    // Updates brake light and buzzer
void process_inverter();  // Updates vehicle state and applicable commands
void check_RTD_button();  //

#endif /* INC_VCU_H_ */
