/*
 * can.c
 *
 *  Created on: 25.07.2015
 *      Author: hd
 */

#include "can.h"

static can_rx_callback_t rx_callback = 0;

void can_init(void) {
	rx_callback = 0;
}

void can_register_rx_callback(can_rx_callback_t callback) {
	rx_callback = callback;
}

void can_notify_message(const can_message_t *msg);
void can_notify_message(const can_message_t *msg) {
	if (rx_callback != 0) {
		rx_callback(msg);
	}
}

void can_send_message(const can_message_t *msg) {
	// TODO really implement me

	can_message_t clone = *msg;
	if (clone.channel==0) {
		clone.channel = 1;
	} else {
		clone.channel = 0;
	}
	can_notify_message(&clone);
}

void can_set_bitrate(uint8_t channel, uint8_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw) {
	// TODO implement me
	(void)channel;
	(void)brp;
	(void)tseg1;
	(void)tseg2;
	(void)sjw;
}

void can_set_silent(uint8_t channel, uint8_t silent_mode) {
	// TODO implement me
	(void)channel;
	(void)silent_mode;
}


void can_set_led(uint8_t channel, led_mode_t mode, uint32_t timeout) {
	// TODO implement me
	(void)channel;
	(void)mode;
	(void)timeout;
}

void can_set_bus_active(uint8_t channel, uint16_t mode) {
	// TODO implement me
	(void)channel;
	(void)mode;
}
