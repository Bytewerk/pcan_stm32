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
#include "systime.h"
#include "pcan_usbpro_fw.h"
#include "pcan_usbpro_sizeof_rec.h"
#include "can.h"


typedef struct {
	uint16_t record_count;
	uint16_t message_counter;
	union pcan_usbpro_rec_t first_record;
} candle_usb_packet_t;


static uint32_t bits_transfered[2] = {0,0};
static usbd_device *dev = 0;

void ppro_usb_protocol_init(usbd_device *usbd_dev) {
	dev = usbd_dev;
	bits_transfered[0] = 0;
	bits_transfered[1] = 0;
}

static void ppro_set_bitrate(uint8_t channel, uint32_t ccbt) {
	//uint8_t  tripple_sample_mode = ccbt>>23 & 0x01;
	uint8_t  tseg2 = ((ccbt >> 20) & 0x07) + 1;
	uint8_t  tseg1 = ((ccbt >> 16) & 0x0F) + 1;
	uint8_t  sjw   = ((ccbt>>14) & 0x03) + 1;
	uint16_t brp_ppro = (ccbt & 0x3fff) + 1;

	uint16_t brp_stm = (24 * brp_ppro) / 56;
	can_set_bitrate(channel, brp_stm, tseg1, tseg2, sjw);
}

static void ppro_tx_message(uint8_t channel, uint32_t id_and_flags, uint8_t dlc, void *data) {
	can_message_t msg;

	if (channel>1) { channel = 1; }

	if (dlc>8) { dlc = 8; }
	msg.channel = channel;
	msg.dlc = dlc;
	msg.timestamp = 0;
	msg.id_and_flags = id_and_flags;
	memcpy(msg.data, data, dlc);

	bits_transfered[channel] += can_calc_message_len(&msg);
	can_send_message(&msg);
}

static void ppro_tx(union pcan_usbpro_rec_t *request) {
	uint32_t id = request->canmsg_tx.id;
	if (request->canmsg_tx.flags & 0x01) { id |= can_flag_rtr; }
	if (request->canmsg_tx.flags & 0x02) { id |= can_flag_extid; }
	ppro_tx_message(request->canmsg_tx.channel, id, request->canmsg_tx.dlc, request->canmsg_tx.data);
}

static void ppro_set_timestamp_mode(uint16_t timestamp_mode) {
	(void)timestamp_mode;
	pcan_status.timestamp_active = (timestamp_mode==1);
	pcan_status.t_next_timestamp = 0;
}

static uint32_t ppro_get_device_id(uint8_t channel) {
	// TODO implement device id handling
	return channel;
}

static uint16_t ppro_get_error_status(uint8_t channel) {
	// TODO implement me
	(void)channel;
	return 0;
}

static void ppro_request_busload(uint8_t channel, uint8_t mode, uint16_t prescaler, uint16_t quanta) {
	(void)prescaler;
	(void)quanta;
	pcan_status.busload_mode[channel] = mode;
}

static void ppro_usb_send_packet(uint8_t ep, candle_usb_packet_t *packet, int packet_size) {
	static uint16_t message_counter = 0;
	packet->message_counter = message_counter++;
	while (usbd_ep_write_packet(dev, 0x80|ep, packet, packet_size) == 0);
}

void ppro_rx_message(const can_message_t *msg) {
	candle_usb_packet_t canmsg;
	canmsg.record_count = 1;
	canmsg.first_record.canmsg_rx.data_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8;
	canmsg.first_record.canmsg_rx.client = 0;

	canmsg.first_record.canmsg_rx.timestamp32 = msg->timestamp;
	canmsg.first_record.canmsg_rx.channel = msg->channel;

	canmsg.first_record.canmsg_rx.id = msg->id_and_flags & 0x1FFFFFFF;
	canmsg.first_record.canmsg_rx.flags = msg->id_and_flags >> 30;

	canmsg.first_record.canmsg_rx.dlc = msg->dlc;

	memcpy(canmsg.first_record.canmsg_rx.data, msg->data, 8);

	ppro_usb_send_packet(2, &canmsg, 4+sizeof(canmsg.first_record.canmsg_rx));

	if (msg->channel<2) {
		bits_transfered[msg->channel] += can_calc_message_len(msg);
	}
}

void ppro_usb_send_timestamp(uint8_t ep) {
	(void)ep;
	static candle_usb_packet_t packet_timestamp;
	packet_timestamp.record_count = 1;
	packet_timestamp.first_record.calibration_ts_rx.data_type = DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX;
	packet_timestamp.first_record.calibration_ts_rx.dummy[0] = 0;
	packet_timestamp.first_record.calibration_ts_rx.dummy[1] = 0;
	packet_timestamp.first_record.calibration_ts_rx.dummy[2] = 0;
	packet_timestamp.first_record.calibration_ts_rx.timestamp64[0] = 0;
	packet_timestamp.first_record.calibration_ts_rx.timestamp64[1] = get_time_ms() * 1000;
	ppro_usb_send_packet(2, &packet_timestamp, 4+sizeof(packet_timestamp.first_record.calibration_ts_rx));
}

void ppro_usb_send_busload(uint8_t ep, uint8_t channel) {
	(void)ep;
	static candle_usb_packet_t packet_busload;
	packet_busload.record_count = 1;
	packet_busload.first_record.buslast_rx.data_type = DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX;
	packet_busload.first_record.buslast_rx.channel = channel;
	packet_busload.first_record.buslast_rx.buslast_val = 500;
	packet_busload.first_record.buslast_rx.timestamp32 = get_time_ms() * 1000;
	ppro_usb_send_packet(2, &packet_busload, 4+sizeof(packet_busload.first_record.buslast_rx));
}


void ppro_usb_protocol_handle_data(uint8_t ep, uint8_t *buf, int len) {

	static candle_usb_packet_t reply;

	if (len>=4) {
		candle_usb_packet_t *packet = (candle_usb_packet_t*)buf;

		int pos = 4;
		int num_records = 0;

		while ((++num_records <= packet->record_count) && (pos<len)) {

			union pcan_usbpro_rec_t *request = (union pcan_usbpro_rec_t*) &buf[pos];

			switch (request->data_type) {

				case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
					can_set_led(request->set_can_led.channel, request->set_can_led.mode, request->set_can_led.timeout);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
					can_set_bus_active(request->bus_activity.channel, request->bus_activity.onoff);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
					reply.record_count = 1;
					reply.first_record.dev_nr.data_type = DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR;
					reply.first_record.dev_nr.channel = request->dev_nr.channel;
					reply.first_record.dev_nr.serial_num = ppro_get_device_id(request->dev_nr.channel);
					ppro_usb_send_packet(ep, &reply, 4+8);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_SETBAUDRATE:
					ppro_set_bitrate(request->baudrate.channel, request->baudrate.CCBT);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_SETSILENTMODE:
					can_set_silent(request->silent_mode.channel, request->silent_mode.onoff);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_SETFILTERMODE:
					// ignored
					//ppro_set_filter_mode(request->filer_mode.filter_mode);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_CALIBRATION_MSG:
					ppro_set_timestamp_mode(request->calibration.mode);
					ppro_usb_send_timestamp(2);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_0:
				case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_4:
				case DATA_TYPE_USB2CAN_STRUCT_CANMSG_TX_8:
					ppro_tx(request);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS:
					reply.record_count = 1;
					reply.first_record.error_status.data_type = DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS;
					reply.first_record.error_status.channel = request->error_status.channel;
					reply.first_record.error_status.status = ppro_get_error_status(request->error_status.channel);
					ppro_usb_send_packet(ep, &reply, 4+8);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_SETGET_BUSLAST_MSG:
					ppro_request_busload(
						request->buslast.channel,
						request->buslast.mode,
						request->buslast.prescaler,
						request->buslast.sampletimequanta
					);
					break;
				default:
					// unknown request type, this is bad because we do not know how long the request would be...
					return;
			}

			pos += pcan_usbpro_sizeof_rec(request->data_type);
		}
	}
}
