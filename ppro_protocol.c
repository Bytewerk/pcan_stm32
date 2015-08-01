/*
 * usb_pcan_protocol.c
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <libopencm3/usb/usbd.h>
#include "systime.h"
#include "external/pcan_usbpro_fw.h"
#include "external/pcan_usbpro_sizeof_rec.h"
#include "can.h"
#include "ppro_protocol.h"
#include "ppro_usb.h"


typedef struct {
	uint16_t record_count;
	uint16_t message_counter;
	union pcan_usbpro_rec_t first_record;
} candle_usb_packet_t;


typedef struct {
	struct {
		uint16_t record_count;
		uint16_t message_counter;
		uint8_t data[120];
	} output_buffer;
	unsigned output_buffer_pos;
	uint32_t bits_transferred;
} channel_data_t;


static usbd_device *dev = 0;
static channel_data_t channel_data[2];


void ppro_usb_protocol_init(usbd_device *usbd_dev) {
	dev = usbd_dev;
	memset(channel_data, 0, sizeof(channel_data));
}

static void ppro_usb_flush(uint8_t ep) {
	assert((ep==1)||(ep==2));
	channel_data_t *data = &channel_data[ep-1];
	if (data->output_buffer.record_count > 0) {
		while (usbd_ep_write_packet(dev, 0x80|ep, &data->output_buffer, 4+data->output_buffer_pos) == 0);
		data->output_buffer.record_count = 0;
		data->output_buffer_pos = 0;
		data->output_buffer.message_counter++;
	}
}

void ppro_usb_flush_all(void) {
	ppro_usb_flush(1);
	ppro_usb_flush(2);
}

static void ppro_usb_enqueue_record(uint8_t ep, union pcan_usbpro_rec_t *record) {
	assert((ep==1)||(ep==2));
	channel_data_t *data = &channel_data[ep-1];

	uint8_t record_length = pcan_usbpro_sizeof_rec(record->data_type);
	if ((data->output_buffer_pos + record_length) > sizeof(data->output_buffer.data)) {
		// not enough space left in buffer
		ppro_usb_flush(ep);
	}

	data->output_buffer.record_count++;
	memcpy(&data->output_buffer.data[data->output_buffer_pos], record, record_length);
	data->output_buffer_pos += record_length;
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

	assert((channel==0)||(channel==1));

	if (dlc>8) { dlc = 8; }
	msg.channel = channel;
	msg.dlc = dlc;
	msg.timestamp = 0;
	msg.id_and_flags = id_and_flags;
	memcpy(msg.data, data, dlc);

	channel_data[channel].bits_transferred += can_calc_message_len(&msg);
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

void ppro_rx_message(const can_message_t *msg) {
	union pcan_usbpro_rec_t record;
	record.canmsg_rx.data_type = DATA_TYPE_USB2CAN_STRUCT_CANMSG_RX_8;
	record.canmsg_rx.client = 0;
	record.canmsg_rx.timestamp32 = msg->timestamp;
	record.canmsg_rx.channel = msg->channel;
	record.canmsg_rx.id = msg->id_and_flags & 0x1FFFFFFF;
	record.canmsg_rx.flags = msg->id_and_flags >> 30;
	record.canmsg_rx.dlc = msg->dlc;
	memcpy(record.canmsg_rx.data, msg->data, 8);

	ppro_usb_enqueue_record(2, &record);

	if (msg->channel<2) {
		channel_data[msg->channel].bits_transferred += can_calc_message_len(msg);
	}
}

void ppro_usb_send_timestamp(uint8_t ep) {
	union pcan_usbpro_rec_t record;
	record.calibration_ts_rx.data_type = DATA_TYPE_USB2CAN_STRUCT_CALIBRATION_TIMESTAMP_RX;
	record.calibration_ts_rx.dummy[0] = 0;
	record.calibration_ts_rx.dummy[1] = 0;
	record.calibration_ts_rx.dummy[2] = 0;
	record.calibration_ts_rx.timestamp64[0] = 0;
	record.calibration_ts_rx.timestamp64[1] = get_time_ms() * 1000;
	ppro_usb_enqueue_record(ep, &record);
}

void ppro_usb_send_busload(uint8_t ep, uint8_t channel) {
	assert((channel==0)||(channel==1));
	union pcan_usbpro_rec_t record;
	record.buslast_rx.data_type = DATA_TYPE_USB2CAN_STRUCT_BUSLAST_RX;
	record.buslast_rx.channel = channel;
	record.buslast_rx.buslast_val = channel_data[0].bits_transferred;
	channel_data[0].bits_transferred = 0;
	record.buslast_rx.timestamp32 = get_time_ms() * 1000;
	ppro_usb_enqueue_record(ep, &record);
}


void ppro_usb_protocol_handle_data(uint8_t ep, uint8_t *buf, int len) {
	union pcan_usbpro_rec_t record;

	if (len>=4) {
		candle_usb_packet_t *packet = (candle_usb_packet_t*)buf;

		int pos = 4;
		int num_records = 0;

		while ((++num_records <= packet->record_count) && (pos<len)) {

			union pcan_usbpro_rec_t *request = (union pcan_usbpro_rec_t*) &buf[pos];

			switch (request->data_type) {

				case DATA_TYPE_USB2CAN_STRUCT_FKT_SET_CANLED:
					can_set_led_mode(request->set_can_led.channel, request->set_can_led.mode, request->set_can_led.timeout);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_SETCANBUSACTIVATE:
					can_set_bus_active(request->bus_activity.channel, request->bus_activity.onoff);
					break;
				case DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR:
					record.dev_nr.data_type = DATA_TYPE_USB2CAN_STRUCT_FKT_GETDEVICENR;
					record.dev_nr.channel = request->dev_nr.channel;
					record.dev_nr.serial_num = ppro_get_device_id(request->dev_nr.channel);
					ppro_usb_enqueue_record(ep, &record);
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
					record.error_status.data_type = DATA_TYPE_USB2CAN_STRUCT_FKT_GETCANBUS_ERROR_STATUS;
					record.error_status.channel = request->error_status.channel;
					record.error_status.status = ppro_get_error_status(request->error_status.channel);
					ppro_usb_enqueue_record(ep, &record);
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
	ppro_usb_flush_all();
}

