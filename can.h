/*
 * can.h
 *
 *  Created on: 25.07.2015
 *      Author: hd
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>

typedef enum {
	can_led_mode_auto,
	can_led_mode_blink_fast,
	can_led_mode_blink_slow,
	can_led_mode_on,
	can_led_mode_off
} led_mode_t;

void can_init(void);
void can_set_bitrate(uint8_t channel, uint8_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw);
void can_set_bus_active(uint8_t channel, uint16_t mode);
void can_set_silent(uint8_t channel, uint8_t silent_mode);
void can_set_led(uint8_t channel, led_mode_t mode, uint32_t timeout);

#endif /* CAN_H_ */
