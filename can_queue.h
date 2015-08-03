/*
 * can_queue.h
 *
 *  Created on: 03.08.2015
 *      Author: hd
 */

#ifndef CAN_QUEUE_H_
#define CAN_QUEUE_H_

#include "config.h"
#include "can.h"

typedef struct {
	void *next;
	void *prev;
	can_message_t msg;
} can_queue_item_t;

typedef struct {
	can_queue_item_t *first;
	can_queue_item_t *last;
	unsigned count;
} can_queue_t;

void can_queue_init(can_queue_t *queue);
void can_queue_push_back(can_queue_t *queue, can_queue_item_t *item);
int can_queue_pop_front(can_queue_t *queue, can_queue_item_t **item);

#endif /* CAN_QUEUE_H_ */
