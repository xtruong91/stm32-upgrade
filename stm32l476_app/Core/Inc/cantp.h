/*
 * cantp.h
 *
 *  Created on: Jun 7, 2024
 *      Author: truongtx
 */

#ifndef INC_CANTP_H_
#define INC_CANTP_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32l4xx_hal.h"

/*------------------------------------------------------------------ Includes */

/* Public defines ------------------------------------------------------------*/
#define CANTX_FIFO_SIZE 50
/*------------------------------------------------------------ Public defines */

/* Public typedef ------------------------------------------------------------*/
typedef struct{
	CAN_TxHeaderTypeDef header;
	uint8_t Data[8];
}CanTxMsgTypeDef;

typedef struct{
	CanTxMsgTypeDef array[CANTX_FIFO_SIZE];

	uint8_t current_index;
	uint8_t current_qty_in_queue;
	uint8_t current_next_to_go_out;

	CanTxMsgTypeDef SendMsgBuff;
	uint8_t TxInProgress;

}CANTX_FIFO;


/* function prototypes -------------------------------------------------------*/
void CANTX_FIFO_init(CANTX_FIFO* o, CAN_HandleTypeDef* hcan);

void CANTX_FIFO_clear(CANTX_FIFO* o);

uint8_t CANTX_FIFO_isin(CANTX_FIFO* o,CanTxMsgTypeDef* pmsg);

uint8_t CANTX_FIFO_push(CANTX_FIFO* o,CanTxMsgTypeDef* pmsg);

uint8_t CANTX_FIFO_intellipush(CANTX_FIFO* o,CanTxMsgTypeDef* pmsg);

CanTxMsgTypeDef CANTX_FIFO_pull(CANTX_FIFO* o);

void CANTX_FIFO_INITIATE_TRANSMIT(CANTX_FIFO* o,CAN_HandleTypeDef* hcan);

/*------------------------------------------------------- function prototypes */

#endif /* INC_CANTP_H_ */
