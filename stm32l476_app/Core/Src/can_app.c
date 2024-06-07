/*
 * can_app.c
 *
 *  Created on: Jun 7, 2024
 *      Author: truongtx
 */

#include "can_app.h"
#include "can_conf.h"
/*------------------------------------------------------------------ Includes */

__attribute__((__section__(".board_info"))) const unsigned char BOARD_NAME[10] = "APP";

/* Public variables ----------------------------------------------------------*/
/* Declare all global variables here */

/* Global management -------------------*/
uint32_t G_mSCounter=0;
/*------------------- Global management */



/* CAN management ----------------------*/
CANTX_FIFO CanTxList;
/*---------------------- CAN management */

/* Public function definitions -----------------------------------------------*/

void CAN_SEND_1000ms(){

#if 0
	CanTxMsgTypeDef CAN_SEND_MSG;
	uint32_t tempbitstream;
#endif

}

void CAN_SEND_100ms(){

#if 0
	CanTxMsgTypeDef CAN_SEND_MSG;
	uint32_t tempbitstream;
#endif

}

void CAN_SEND_20ms(){
	CanTxMsgTypeDef CAN_SEND_MSG;

	static uint8_t heartbeat=0;

	// Increment Heartbeat
	heartbeat++;
	if (heartbeat >= 255)heartbeat = 0;

	//=========================================================================================
	// This message is designed to keep compatibility with ECU that was programmed to work with the PIC32 version
	CAN_SEND_MSG.header.IDE = CAN_ID_EXT;
	CAN_SEND_MSG.header.ExtId = 0x18FFFF00 + CAN_SA;
	CAN_SEND_MSG.header.RTR = CAN_RTR_DATA;
	CAN_SEND_MSG.header.DLC = 8;

	CAN_SEND_MSG.Data[0]=0x00;
	CAN_SEND_MSG.Data[1]=0x00;
	CAN_SEND_MSG.Data[2]=0x00;
	CAN_SEND_MSG.Data[3]=0x00;
	CAN_SEND_MSG.Data[4]=0x00;
	CAN_SEND_MSG.Data[5]=0x00;
	CAN_SEND_MSG.Data[6]=0x00;
	CAN_SEND_MSG.Data[7]=heartbeat;

	CANTX_FIFO_intellipush(&CanTxList, &CAN_SEND_MSG);

}
