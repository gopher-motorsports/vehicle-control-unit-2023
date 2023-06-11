/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_VCU_H_
#define INC_VCU_H_

#include "main.h"


// ========================================== CONSTANTS =========================================
#define MATH_PI         3.14159265
#define MATH_TAU        MATH_PI*2
#define SECONDS_PER_MIN 60
// ==============================================================================================

// ======================================= APPS PARAMETERS ======================================
#define APPS_MAX_POS_mm  20 // The position of the pedal at 100% torque
#define APPS_MIN_POS_mm  10  // The position of the pedal at 0% torque
#define APPS_TOTAL_TRAVEL_mm ( APPS_MAX_POS_mm - APPS_MIN_POS_mm )
// ==============================================================================================

// ====================================== BRAKE PARAMETERS ======================================
#define BRAKE_PRESS_MIN_psi    -50   // The minimum value of the brake pressure sensor
#define BRAKE_PRESS_MAX_psi    2050  // The maximum value of the brake pressure sensor
#define BRAKE_LIGHT_THRESH_psi 25 // The pressure at which the brake light will be activated
// ==============================================================================================
// ============================= TRACTIVE SYSTEM CURRENT PARAMETERS =============================
#define TS_CURRENT_MIN_A   -85   // The minimum value of the current sensor
#define TS_CURRENT_MAX_A   85  // The maximum value of the current sensor
// ==============================================================================================

// ================================== READY TO DRIVE PARAMETERS =================================
#define PREDRIVE_BRAKE_THRESH_psi  100  // The minimum brake pressure to enter the driving state
#define PREDRIVE_BUTTON_PRESSED    1    // The value of the button parameter when pressed
#define PREDRIVE_TIME_ms           2000 // The length of predrive in ms
#define RTD_BUTTON_PUSHED          GPIO_PIN_RESET
// ==============================================================================================


// ====================================== SAFETY PARAMETERS =====================================
// -------------------------------------- Input Validation --------------------------------------
#define INPUT_TRIP_DELAY_ms 100  // The amount of time it takes an input fault to take effect

// -------------------------------------- APPS/Brake Check --------------------------------------
// This check is done using APPS1 (since APPS1 determines the applied torque) and the BSE
#define APPS_BRAKE_PRESS_THRESH_psi  50  // The minimum amount of brake pressure that will trip
// The minimum APPS position that will trip the APPS/Brake check
#define APPS_BRAKE_APPS1_THRESH_mm   ( APPS_TOTAL_TRAVEL_mm * 0.25 ) + APPS_MIN_POS_mm
// The maximum APPS position that will reset the APPS/Brake check
#define APPS_BRAKE_RESET_THRESH_mm   ( APPS_TOTAL_TRAVEL_mm * 0.05 ) + APPS_MIN_POS_mm

// ------------------------------------ APPS Correlation Check ----------------------------------
#define APPS_CORRELATION_THRESH_mm  ( APPS_TOTAL_TRAVEL_mm * 0.1 )
#define CORRELATION_TRIP_DELAY_ms    100  // The amount of time it takes a correlation fault to take effect

// ------------------------------------ TS Current/Brake Check ----------------------------------
#define BRAKE_TS_CURRENT_THRESH_A  14  // The current limit when the brake and
#define BRAKE_TS_PRESS_THRESH_psi  450 // The amount of brake pressure needed
#define BRAKE_TS_MAX_TORQUE            // A clamp on the maximum amount of torque when triggered
#define BRAKE_TS_ON_DELAY_ms       50  // The amount of timer it takes the limit to turn on
#define BRAKE_TS_OFF_DELAY_ms      50  // The amount of timer it takes the limit to turn off
// ==============================================================================================

// ====================================== COOLING PARAMETERS ====================================
#define IGBT_TEMP_THRESH_C        40 // Minimum IGBT temperature for cooling fan to turn on
#define GDB_TEMP_THRESH_C         40 // Minimum Gate Drive Board temp for cooling fan to turn on
#define CTRL_BOARD_TEMP_THRESH_C  40 // Minimum Control Board temp for cooling fan to turn on
#define MOTOR_TEMP_THRESH_C       30 // Minimum Motor temperature for cooling fan to turn on
// ==============================================================================================


// ================================== TRACTIVE SYSTEM PARAMETERS ================================
#define MOTOR_DIRECTION         1      // Motor direction; 0 is reverse, 1 is forward
#define MAX_CMD_TORQUE_Nm       50    // The maximum torque that will be commanded
#define INVERTER_TIMEOUT_ms     100    // The time after which the vehicle will enter STARTUP
#define INVERTER_ENABLE         0x01   // Flags to enable the inverter
#define INVERTER_DISABLE        0x00   // Flags to disable the inverter
#define INVERTER_LOCKOUT        0x40   // Lockout is bit 7 of byte 6
#define INVERTER_CMD_ID         0x0C0  // The CAN ID of the inverter command
#define INVERTER_PARAM_ID       0x0C1  // The CAN ID of the parameter message
#define PARAM_CMD_FAULT_CLEAR   20     // Address of the fault clear parameter
#define PARAM_CMD_READ          0      // Value to send in parameter command to read value
#define PARAM_CMD_WRITE         1      // Value to send in parameter command to read value
#define PARAM_CMD_RESERVED1     0x00   // Reserved value in inverter parameter
#define PARAM_FAULT_CLEAR_DATA  0      // Value to send in the data field when clearing faults
#define PARAM_CMD_RESERVED2     0x0000 // Reserved value in inverter parameter
// ==============================================================================================

// ======================================== I/O PARAMETERS ======================================
#define BUZZER_ON        GPIO_PIN_SET
#define BUZZER_OFF       GPIO_PIN_RESET
#define BRAKE_LIGHT_ON   GPIO_PIN_SET
#define BRAKE_LIGHT_OFF  GPIO_PIN_RESET
// ==============================================================================================

// ======================================= BSPD PARAMETERS ======================================
// Whether each fault is active high or active low
#define BSPD_APPS1_FAULT     GPIO_PIN_RESET
#define BSPD_APPS2_FAULT     GPIO_PIN_RESET
#define BSPD_BRAKE_FAULT     GPIO_PIN_RESET
#define BSPD_TS_SNS_FAULT    GPIO_PIN_RESET
#define BSPD_TS_BRK_FAULT    GPIO_PIN_SET
// ==============================================================================================

// =================== THROTTLE CALCULATION ===================
// Throttle is calculated using APPS1 with APPS2 being used
// for redundancy in the correlation  check mandated by rules
// ============================================================

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

typedef enum {
	NONE = 0,
	RELEASE_PEDAL,
	BREAKING_FAULT,
	APPS_FAULT,
	BSPD_FAULT,
	AMS_FAULT,
	IMD_FAULT,
	VCU_FAULT,
	BMS_FAULT
} DISPLAY_FAULT_STATUS_t;

extern VEHICLE_STATE_t vehicle_state;

void init(CAN_HandleTypeDef* hcan_ptr);
void main_loop();
void can_buffer_handling_loop();

void update_RTD();         // Ready to drive logic
void process_sensors();    // Runs safety checks on driver inputs
void update_gcan_states(); // Updates GopherCAN states
void process_inverter();   // Updates vehicle state and applicable commands
void update_outputs();     // Updates brake light and buzzer
void update_cooling();     // Controls/updates the cooling system
void update_display_fault_status(); 	// Check all vehicle fault messages and sends best one to display

#endif /* INC_VCU_H_ */
