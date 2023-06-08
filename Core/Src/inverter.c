

#include <inverter.h>
#include "cmsis_os.h"
#include "GopherCAN.h"

static void update_inv_state(void);
static S8 send_can_message(U32 id, U8 dlc, U8* data);


INV_CTRL_STATE_t inv_ctrl_state = INV_LOCKOUT;
extern CAN_HandleTypeDef hcan2;

INV_CTRL_STATE_t get_inv_state(void)
{
	return inv_ctrl_state;
}

//void handle_inverter(DRIVE_STATE_t* curr_state)
//{
//	U8 msg_data[8];
//	U16 torque_req = 0;
//
//	update_inv_state();
//
//	// find the state of the inverter based on the status of the ECU
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
//
//	// if the inverter is locked out, we must first disable and enable the motor
//	switch (inv_ctrl_state)
//	{
//	case INV_LOCKOUT:
//		// send the disable then enable command with some delay
//		CAN_cmd.inverter_en = 0;
//		build_cmd_msg(msg_data, &CAN_cmd);
//		send_can_message(COMMAND_MSG_ID, 8, msg_data);
//
//		// also clear all faults when first starting up
//		rw_cmd.param_addr = FAULT_CLEAR;
//		rw_cmd.read_or_write = WRITE_CMD;
//		rw_cmd.data = 0;
//		build_param_cmd_msg(msg_data, &rw_cmd);
//		send_can_message(PARAM_RW_CMD_ID, 8, msg_data);
//		break;
//
//	case INV_DISABLED:
//		// try to enable the motor
//		if (*curr_state == READY_STATE)
//		{
//			CAN_cmd.inverter_en = 1;
//			build_cmd_msg(msg_data, &CAN_cmd);
//			send_can_message(COMMAND_MSG_ID, 8, msg_data);
//		}
//		break;
//
//	case INV_ENABLED:
//		// actually run the motor
//
//		// calculate the torque we want from the motor. Right now we are linear
//		if (vcu_brake_pressure_front.data <= BRAKE_PRESSURE_BREAK_THRESH_psi)
//		{
//			float curr_pp = vcu_apps2.data;
//			if (curr_pp >= APPS2_MAX_TRAVEL_mm)
//			{
//				torque_req = MAX_TORQUE_Nm;
//			}
//			else if (curr_pp <= APPS2_MIN_TRAVEL_mm)
//			{
//				torque_req = 0;
//			}
//			else
//			{
//				// linearly interpolate
//				// TODO
//			}
//		}
//
//		CAN_cmd.torque_cmd = torque_req;
//		build_cmd_msg(msg_data, &CAN_cmd);
//		send_can_message(COMMAND_MSG_ID, 8, msg_data);
//		break;
//
//	default:
//		// not sure how we are here
//		inv_ctrl_state = INV_LOCKOUT;
//		break;
//	}
//}

// send_can_message
//  function to build a tx_header and add the message to the BXCan hardware
static S8 send_can_message(U32 id, U8 dlc, U8* data)
{
	CAN_TxHeaderTypeDef tx_header;
	U32 tx_mailbox_num;

	tx_header.IDE = CAN_ID_STD;                                          // 29 bit id
	tx_header.TransmitGlobalTime = DISABLE;                              // do not send a timestamp
	tx_header.StdId = id;
	tx_header.RTR = DATA_MESSAGE;
	tx_header.DLC = dlc;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &tx_mailbox_num);

	return 0;
}


