/*
 * cantp.c
 *
 *  Created on: Jun 7, 2024
 *      Author: truongtx
 */

#include "cantp.h"


/* Private function prototypes -----------------------------------------------*/
uint8_t CanTxMsgCompare(const CanTxMsgTypeDef* msg1,
		const CanTxMsgTypeDef* msg2);

/* Private function definitions ----------------------------------------------*/
uint8_t CanTxMsgCompare(const CanTxMsgTypeDef* msg1,
		const CanTxMsgTypeDef* msg2) {
	if (msg1->header.ExtId == msg2->header.ExtId) {
		return 1;
	}

	return 0;
}

/* Public function definitions -----------------------------------------------*/

void CANTX_FIFO_init(CANTX_FIFO* o, CAN_HandleTypeDef* hcan) {

	int i;

	//hcan->pTxMsg = &(o->SendMsgBuff);

	CanTxMsgTypeDef tempzeromsg = { 0 };

	for (i = 0; i < CANTX_FIFO_SIZE; i++) {
		o->array[i] = tempzeromsg;
	}

	o->current_index = 0;
	o->current_qty_in_queue = 0;
	o->current_next_to_go_out = 0;
	o->TxInProgress = 0;
}

void CANTX_FIFO_clear(CANTX_FIFO* o) {
	while (o->current_qty_in_queue > 0) {
		CANTX_FIFO_pull(o);
	}
}

uint8_t CANTX_FIFO_isin(CANTX_FIFO* o, CanTxMsgTypeDef* pmsg) {
	uint8_t tmp = 0;
	uint8_t vindex = 0;
	uint8_t counter = 0;

	tmp = 0;
	counter = 0;

	if (o->current_qty_in_queue >= 0) {
		vindex = o->current_next_to_go_out;

		while (counter < o->current_qty_in_queue && !tmp) {

			if (CanTxMsgCompare(&(o->array[vindex]), pmsg)) {
				tmp = 1;
			}

			vindex += 1;

			if (vindex > (CANTX_FIFO_SIZE - 1)) {
				vindex = 0;
			}

			counter += 1;

		}

	}
	return tmp;
}

uint8_t CANTX_FIFO_push(CANTX_FIFO* o, CanTxMsgTypeDef* pmsg) {
	if (o->current_qty_in_queue >= CANTX_FIFO_SIZE) {
		return 0;
	} else {

		o->array[o->current_index] = *pmsg;
		//o->array[o->current_index] = val;

		if (o->current_qty_in_queue == 0) {
			o->current_next_to_go_out = o->current_index;
		}

		o->current_index += 1;
		if (o->current_index > (CANTX_FIFO_SIZE - 1)) {
			o->current_index = 0;
		}

		o->current_qty_in_queue += 1;

		//Initiate transmission with Interrupt here

		return 1;
	}
}

uint8_t CANTX_FIFO_intellipush(CANTX_FIFO* o, CanTxMsgTypeDef* pmsg) {
	uint8_t tmp = 0;

	if (!CANTX_FIFO_isin(o, pmsg)) {
		tmp = CANTX_FIFO_push(o, pmsg);
	}
	return tmp;
}

CanTxMsgTypeDef CANTX_FIFO_pull(CANTX_FIFO* o) {
	CanTxMsgTypeDef tempzeromsg = { 0 };
	CanTxMsgTypeDef dummy = { 0 };

	if (o->current_qty_in_queue > 0) {

		dummy = o->array[o->current_next_to_go_out];

		o->array[o->current_next_to_go_out] = tempzeromsg;

		o->current_next_to_go_out += 1;

		if (o->current_next_to_go_out > (CANTX_FIFO_SIZE - 1)) {
			o->current_next_to_go_out = 0;
		}

		o->current_qty_in_queue -= 1;

	}
	return dummy;
}

void CANTX_FIFO_INITIATE_TRANSMIT(CANTX_FIFO* o, CAN_HandleTypeDef* hcan) {
	uint32_t transmitmailbox;
	if (!o->TxInProgress && o->current_qty_in_queue) {
		o->SendMsgBuff = CANTX_FIFO_pull(o);

		o->TxInProgress=1;

		HAL_CAN_ActivateNotification(hcan,CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_AddTxMessage(hcan,&(o->SendMsgBuff.header),o->SendMsgBuff.Data,&transmitmailbox);

	}

}

/*----------------------------------------------- Public function definitions */
