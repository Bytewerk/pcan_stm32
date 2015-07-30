/*
 * can.c
 *
 *  Created on: 25.07.2015
 *      Author: hd
 */

#include "can.h"
#include "systime.h"
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static can_rx_callback_t rx_callback = 0;

typedef struct {
	led_mode_t mode;
	uint32_t t_next;
	uint8_t state;
} led_status_t;

static led_status_t led_status[2];

void can_init(void) {
	rx_callback = 0;
	memset(led_status, 0, sizeof(led_status));

	// enable led outputs
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
}

void can_poll_leds(void);

void can_poll(void) {
	can_poll_leds();
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

void can_set_bus_active(uint8_t channel, uint16_t mode) {
	// TODO implement me
	(void)channel;
	(void)mode;
}

uint8_t can_calc_message_len(const can_message_t *msg) {
	// NOTICE this ignores stuff bits and is therefore incorrect
	// to calculate the number of stuff bits, we would have to construct
	// the whole can frame which would probably be a lot of overhead
	if (msg->id_and_flags & can_flag_extid) {
		return 64 + 8*msg->dlc + 3;
	} else {
		return 44 + 8*msg->dlc + 3;
	}
}


void can_set_led_mode(uint8_t channel, led_mode_t mode, uint32_t timeout) {
	(void)timeout;
	if (channel < 2) {
		led_status[channel].mode = mode;
		led_status[channel].t_next = get_time_ms();
	}
}

static void set_led_state(uint8_t channel, uint8_t state) {
	uint16_t pin = (channel==0) ? GPIO12 : GPIO14;
	if (state) {
		gpio_set(GPIOD, pin);
	} else {
		gpio_clear(GPIOD, pin);
	}
}

void can_poll_leds(void) {

	uint32_t now = get_time_ms();

	for (uint8_t i=0; i<2; i++) {
		led_status_t *status = &led_status[i];

		if ( status->t_next && (status->t_next <= now)) {

			switch (status->mode) {
				case can_led_mode_auto:
				case can_led_mode_off:
					status->state = 0;
					status->t_next = 0;
					break;
				case can_led_mode_on:
					status->state = 1;
					status->t_next = 0;
					break;
				case can_led_mode_blink_fast:
					status->state = !status->state;
					status->t_next += 100;
					break;
				case can_led_mode_blink_slow:
					status->state = !status->state;
					status->t_next += 500;
					break;
			}
		}

		set_led_state(i, status->state);
	}
}
