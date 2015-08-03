/*
 * can_queue.c
 *
 *  Created on: 03.08.2015
 *      Author: hd
 */

#include "can_queue.h"

void can_queue_init(can_queue_t *queue) {
	queue->first = 0;
	queue->last = 0;
	queue->count = 0;
}

void can_queue_push_back(can_queue_t *queue, can_queue_item_t *item) {
	if (queue->count==0) {
		queue->first = item;
		queue->last = item;
		item->prev = 0;
		item->next = 0;
	} else {
		item->prev = queue->last;
		item->next = 0;
		if (queue->last) {
			queue->last->next = item;
		}
		queue->last = item;
	}
	queue->count++;
}

int can_queue_pop_front(can_queue_t *queue, can_queue_item_t **item) {

	if (queue->count==0) {
		return 0;
	} else {
		*item = queue->first;
		queue->first = (*item)->next;
		queue->count--;
		return 1;
	}
}
