/*
 * can_app.h
 *
 *  Created on: Jun 7, 2024
 *      Author: truongtx
 */

#ifndef INC_CAN_APP_H_
#define INC_CAN_APP_H_

#include <stdint.h>
#include "cantp.h"
/* Public variables ----------------------------------------------------------*/
/* Define all extern global variables here
 * Do not initialize value here */

/* Global management -------------------*/
extern uint32_t G_mSCounter;
/*------------------- Global management */


/* CAN management ----------------------*/
extern CANTX_FIFO CanTxList;
/*---------------------- CAN management */

/* function prototypes -------------------------------------------------------*/
void CAN_SEND_1000ms();
void CAN_SEND_100ms();
void CAN_SEND_20ms();
/*------------------------------------------------------- function prototypes */

#endif /* INC_CAN_APP_H_ */
