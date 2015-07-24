/*
 * commands.h
 *
 *  Created on: 14.07.2015
 *      Author: hd
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <stdint.h>

typedef enum {
	rt_set_bitrate = 0x02,
	rt_set_bus_active = 0x04,
	rt_set_silent  = 0x05,
	rt_set_warning_limit = 0x07,
	rt_set_filter = 0x0a,
	rt_set_error_frame = 0x0c,
	rt_error_status = 0x0d,
	rt_set_timestamp = 0x10,
	rt_request_busload = 0x11,
	rt_get_device_id = 0x12,
	rt_set_led = 0x1c,

	rt_rx_msg_8 = 0x80,
	rt_rx_msg_4 = 0x81,
	rt_rx_msg_0 = 0x82,
	rt_rx_rtr = 0x83,
	rt_rx_status = 0x84,
	rt_rx_timestamp = 0x85,
	rt_busload_info = 0x86,

	rt_tx_msg_8 = 0x41,
	rt_tx_msg_4 = 0x42,
	rt_tx_msg_0 = 0x43
} record_type_t;

typedef union {

	struct {
		uint8_t record_type;
	} generic;

	struct {
		uint8_t record_type;
		uint8_t channel;
		uint16_t led_mode;
		uint32_t timeout;
	} set_led;

	struct {
		uint8_t record_type;
		uint8_t channel;
		uint16_t bus_active_mode;
	} set_bus_active;

	struct {
		uint8_t record_type;
		uint8_t channel;
		uint16_t padding;
		uint32_t serial_number;
	} get_device_id;

	struct {
		uint8_t record_type;
		uint8_t channel;
		uint16_t padding;
		uint32_t ccbt;
	} set_bitrate;

	struct {
		uint8_t record_type;
		uint8_t channel;
		uint16_t silent_mode;
	} set_silent;

	struct {
		uint8_t record_type;
		uint8_t dummy;
		uint16_t filter_mode;
	} set_filter_mode;

	struct {
		uint8_t record_type;
		uint8_t dummy;
		uint16_t timestamp_mode;
	} set_timestamp_mode;

	struct {
		uint8_t record_type;
		uint8_t client;
		uint8_t flags;
		unsigned dlc:4;
		unsigned channel:4;
		uint32_t timestamp;
		uint32_t can_id;
		uint8_t data[8];
	} rx_message;

	struct {
		uint8_t record_type;
		uint8_t client;
		uint8_t flags;
		unsigned dlc:4;
		unsigned channel:4;
		uint32_t can_id;
		uint8_t data[8];
	} tx_message;

	struct {
		uint8_t record_type;
		uint8_t dummy[3];
		uint32_t unknown;
		uint32_t ts_us;
	} rx_timestamp;

	struct {
		uint8_t record_type;
		uint8_t channel;
		uint16_t status;
	} error_status;

	struct {
		uint8_t	record_type;
		uint8_t	channel;
		uint8_t	dummy;
		uint8_t	mode;
		uint16_t prescaler;
		uint16_t sampletimequanta;
	} request_busload;

	struct {
		uint8_t	record_type;
		uint8_t	channel;
		uint16_t buslast_val;
		uint32_t ts_us;
	} busload_info;

} candle_packet_record_t;


typedef struct {
	uint16_t record_count;
	uint16_t message_counter;
	candle_packet_record_t first_record;
} candle_usb_packet_t;


#endif /* COMMANDS_H_ */
