/*
 * usb_pcan_protocol.c
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#include "usb_pcan_protocol.h"
#include "usb_pcan.h"
#include <stdint.h>
#include <string.h>
#include <libopencm3/usb/usbd.h>
#include "commands.h"

static volatile uint8_t is_bus_active[2] = {0, 0};


static void cmd_set_led(uint8_t channel, uint16_t led_mode, uint32_t timeout) {
	(void)channel;
	(void)led_mode;
	(void)timeout;
}

static void cmd_set_bus_active(uint8_t channel, uint16_t bus_active_mode) {
	(void)channel;
	(void)bus_active_mode;
	is_bus_active[channel] = bus_active_mode!=0;
}

static void cmd_set_bitrate(uint8_t channel, uint32_t ccbt) {
	(void)channel;
	(void)ccbt;
}

static void cmd_set_silent(uint8_t channel, uint16_t silent_mode) {
	(void)channel;
	(void)silent_mode;
}

static void cmd_set_filter_mode(uint16_t filter_mode) {
	(void)filter_mode;
}

static void cmd_set_timestamp_mode(uint16_t timestamp_mode) {
	(void)timestamp_mode;
	pcan_status.timestamp_active = (timestamp_mode==1);
	pcan_status.t_next_timestamp = 0;
}

static uint32_t cmd_get_device_id(uint8_t channel) {
	return channel;
}

static uint16_t cmd_get_error_status(uint8_t channel) {
	(void)channel;
	return 0;
}

static void can_request_busload(uint8_t channel, uint8_t mode, uint16_t prescaler, uint16_t quanta) {
	(void)prescaler;
	(void)quanta;
	pcan_status.busload_mode[channel] = mode;
}



static void candle_usb_send_packet(usbd_device *usbd_dev, uint8_t ep, candle_usb_packet_t *packet, int packet_size) {
	static uint16_t message_counter = 0;
	packet->message_counter = message_counter++;
	while (usbd_ep_write_packet(usbd_dev, 0x80|ep, packet, packet_size) == 0);
}


static void can_test(usbd_device *usbd_dev, uint8_t ep, candle_packet_record_t *request) {
	candle_usb_packet_t canmsg;
	canmsg.record_count = 1;
	canmsg.first_record.rx_message.record_type = rt_rx_msg_8;
	canmsg.first_record.rx_message.client = 0;
	canmsg.first_record.rx_message.flags = request->tx_message.flags;
	canmsg.first_record.rx_message.channel = (request->tx_message.channel==0) ? 1 : 0;
	canmsg.first_record.rx_message.dlc = request->tx_message.dlc;
	canmsg.first_record.rx_message.can_id = request->tx_message.can_id;
	canmsg.first_record.rx_message.timestamp = 0;
	memcpy(canmsg.first_record.rx_message.data, request->tx_message.data, 8);
	candle_usb_send_packet(usbd_dev, 2, &canmsg, 4+sizeof(canmsg.first_record.rx_message));
}

void candle_usb_send_timestamp(usbd_device *usbd_dev, uint8_t ep) {
	static candle_usb_packet_t packet_timestamp;
	packet_timestamp.record_count = 1;
	packet_timestamp.first_record.rx_timestamp.record_type = rt_rx_timestamp;
	packet_timestamp.first_record.rx_timestamp.dummy[0] = 0;
	packet_timestamp.first_record.rx_timestamp.dummy[1] = 0;
	packet_timestamp.first_record.rx_timestamp.dummy[2] = 0;
	packet_timestamp.first_record.rx_timestamp.unknown = 0;
	packet_timestamp.first_record.rx_timestamp.ts_us = get_time_ms() * 1000;
	candle_usb_send_packet(usbd_dev, 2, &packet_timestamp, 4+sizeof(packet_timestamp.first_record.rx_timestamp));
}

void candle_usb_send_busload(usbd_device *usbd_dev, uint8_t ep, uint8_t channel) {
	static candle_usb_packet_t packet_busload;
	packet_busload.record_count = 1;
	packet_busload.first_record.busload_info.record_type = rt_busload_info;
	packet_busload.first_record.busload_info.channel = channel;
	packet_busload.first_record.busload_info.buslast_val = 500;
	packet_busload.first_record.busload_info.ts_us = get_time_ms() * 1000;
	candle_usb_send_packet(usbd_dev, 2, &packet_busload, 4+sizeof(packet_busload.first_record.busload_info));
}

void usb_pcan_protocol_handle_data(usbd_device *usbd_dev, uint8_t ep, uint8_t *buf, int len) {

	static candle_usb_packet_t reply;

	if (len>=4) {
		candle_usb_packet_t *packet = (candle_usb_packet_t*)buf;

		int pos = 4;
		int num_records = 0;

		while ((++num_records <= packet->record_count) && (pos<len)) {
			candle_packet_record_t *request = (candle_packet_record_t*) &buf[pos];
			switch (request->generic.record_type) {
				case rt_set_led:
					cmd_set_led(request->set_led.channel, request->set_led.led_mode, request->set_led.timeout);
					pos += 8;
					break;
				case rt_set_bus_active:
					cmd_set_bus_active(request->set_bus_active.channel, request->set_bus_active.bus_active_mode);
					pos += 4;
					break;
				case rt_get_device_id:
					reply.record_count = 1;
					reply.first_record.get_device_id.record_type = rt_get_device_id;
					reply.first_record.get_device_id.channel = request->get_device_id.channel;
					reply.first_record.get_device_id.serial_number = cmd_get_device_id(request->get_device_id.channel);
					candle_usb_send_packet(usbd_dev, ep, &reply, 4+8);
					pos += 8;
					break;
				case rt_set_bitrate:
					cmd_set_bitrate(request->set_bitrate.channel, request->set_bitrate.ccbt);
					pos += 8;
					break;
				case rt_set_silent:
					cmd_set_silent(request->set_silent.channel, request->set_silent.silent_mode);
					pos += 4;
					break;
				case rt_set_filter:
					cmd_set_filter_mode(request->set_filter_mode.filter_mode);
					pos += 4;
					break;
				case rt_set_timestamp:
					cmd_set_timestamp_mode(request->set_timestamp_mode.timestamp_mode);
					candle_usb_send_timestamp(usbd_dev, 2);
					pos += 4;
					break;
				case rt_tx_msg_0:
					can_test(usbd_dev, ep, request);
					pos += 12;
					break;
				case rt_tx_msg_4:
					can_test(usbd_dev, ep, request);
					pos += 16;
					break;
				case rt_tx_msg_8:
					can_test(usbd_dev, ep, request);
					pos += 20;
					break;
				case rt_error_status:
					reply.record_count = 1;
					reply.first_record.error_status.record_type = rt_error_status;
					reply.first_record.error_status.channel = request->error_status.channel;
					reply.first_record.error_status.status = cmd_get_error_status(request->error_status.channel);
					candle_usb_send_packet(usbd_dev, ep, &reply, 4+8);
					pos += 4;
					break;
				case rt_request_busload:
					can_request_busload(
						request->request_busload.channel,
						request->request_busload.mode,
						request->request_busload.prescaler,
						request->request_busload.sampletimequanta
					);
					pos += 8;
					break;
				default:
					// unknown request type, this is bad because we do not know how long the request would be...
					return;
			}
		}
	}
}
