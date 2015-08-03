/*
 * can.h
 *
 *  Created on: 25.07.2015
 *      Author: hd
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>
#include "config.h"

typedef struct  __attribute__ ((packed)) {

	uint8_t channel;
	uint8_t dlc;
	uint16_t dummy;

	uint32_t timestamp;
	uint32_t id_and_flags;

	union {
		uint8_t data[8];
		uint32_t data16[4];
		uint32_t data32[2];
		uint64_t data64;
	};

} can_message_t;

typedef void (*can_rx_callback_t)(const can_message_t *msg);

typedef enum {
	can_flag_rtr = 0x40000000,
	can_flag_extid = 0x80000000
} can_flags_t;

typedef enum {
	can_led_mode_auto,
	can_led_mode_blink_fast,
	can_led_mode_blink_slow,
	can_led_mode_on,
	can_led_mode_off
} led_mode_t;

void candle_can_init(void);
void candle_can_poll(void);

void candle_can_set_bitrate(uint8_t channel, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw);
void candle_can_send_message(const can_message_t *msg);
void candle_can_register_rx_callback(can_rx_callback_t callback);

void candle_can_set_bus_active(uint8_t channel, uint16_t mode);
void candle_can_set_silent(uint8_t channel, uint8_t silent_mode);

void candle_can_set_led_mode(uint8_t channel, led_mode_t mode, uint32_t timeout);
uint8_t candle_can_calc_message_len(const can_message_t *msg);

#endif /* CAN_H_ */
