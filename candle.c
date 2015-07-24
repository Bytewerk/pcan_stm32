#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>

#include "commands.h"
#include "systime.h"

//#define USE_USB_HS

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0110, // 0x200 with peak, use 0110 to indicate a usb1.1 device (which we are)
	.bDeviceClass = 0x00, // Device
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0c72,
	.idProduct = 0x000d,
	.bcdDevice = 0x0000,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor can_endp[] = {
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
}, 
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
}, 
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
}, 
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x02,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
}, 
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 0,
}, 
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x03,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
}, 
};

static const struct usb_endpoint_descriptor lin_endp[] = {
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x84,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
}, 
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x04,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
},
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x85,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
},
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x05,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
},
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x86,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 0,
},
{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x06,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, // PCAN: 512
	.bInterval = 0,
},
};

static const struct usb_interface_descriptor can_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 6,
	.bInterfaceClass = 0x00,
	.bInterfaceSubClass = 0x00,
	.bInterfaceProtocol = 0x00,
	.iInterface = 5,

	.endpoint = can_endp,
} };

static const struct usb_interface_descriptor lin_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 6,
	.bInterfaceClass = 0x00,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 6,

	.endpoint = lin_endp,
} };

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = can_iface,
}, {
	.num_altsetting = 1,
	.altsetting = lin_iface,
} };

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, // peak: 111
	.bNumInterfaces = 1, // peak: 2
	.bConfigurationValue = 1,
	.iConfiguration = 4,
	.bmAttributes = 0x80,
	.bMaxPower = 0x5A, // 180mA

	.interface = ifaces,
};

static const char * usb_strings[] = {
	"PEAK-System Technik GmbH",
	"PCAN-USB-PRO DEVICE",
	"string2",
	"Config00",
	"PCAN-USB-PRO-CAN Device",
	"PCAN-USB-PRO-LIN Device",
	"string6",
	"string7",
	"string8",
	"string9",
	"string10",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[512];


static char firmware_version[] = {0x44,0x33,0x22,0x11,0x01,0x03,0x03,0x00,0x12,0x06,0x0a,0x40,0x00,0x01,0x00,0x00};
static char bootloader_version[] = {0x22,0x22,0x11,0x11,0x01,0x08,0x03,0x00,0x1a,0x05,0x09,0x00,0xff,0xff,0xff,0xff,0x01,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
static char unknown_req_4[] = { 0x3a, 0x39, 0x38, 0x37, 0x01, 0x00, 0x00, 0x00 };
static char unknown_req_5[] = { 0x77, 0x22, 0x1a, 0x1a, 0x07, 0x00, 0x00, 0x00 };
static volatile uint8_t is_bus_active[2] = {0, 0};

struct {
	uint8_t timestamp_active;
	uint32_t t_next_timestamp;
	uint8_t busload_mode[2];
	uint32_t t_next_busload[2];
} pcan_status;

static int candle_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	static uint8_t buffer[128];

	switch (req->bRequest) {

		case 0: { // get information
			if (req->wValue==0x0000) { // bootloader version
				memcpy(*buf, bootloader_version, sizeof(bootloader_version));
				*len = sizeof(bootloader_version);
			} else if (req->wValue==0x0001) { // firmware version
				memcpy(*buf, firmware_version, sizeof(firmware_version));
				*len = sizeof(firmware_version);
			} else if (req->wValue==0x0004) { // unknown blob 1
				memcpy(*buf, unknown_req_4, sizeof(unknown_req_4));
				*len = sizeof(unknown_req_4);
			} else if (req->wValue==0x0005) { // unknown blob 2
				//candle_usb_data->driver_active = 1;
				memcpy(*buf, unknown_req_5, sizeof(unknown_req_5));
				*len = sizeof(unknown_req_5);
			}

			return 1;
		}

		case 2: { // config?


			if (req->wValue==0x0005) { // driver is now active, no idea what's in the rxdata (check linux driver?)
				//candle_usb_data->driver_active = 1;
				usbd_ep_read_packet(usbd_dev, 0, buffer, 16);
		        return USBD_REQ_HANDLED;
			}

		}

	}
	return 0;
}


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

static void candle_usb_send_timestamp(usbd_device *usbd_dev, uint8_t ep) {
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

static void candle_usb_send_busload(usbd_device *usbd_dev, uint8_t ep, uint8_t channel) {
	static candle_usb_packet_t packet_busload;
	packet_busload.record_count = 1;
	packet_busload.first_record.busload_info.record_type = rt_busload_info;
	packet_busload.first_record.busload_info.channel = channel;
	packet_busload.first_record.busload_info.buslast_val = 500;
	packet_busload.first_record.busload_info.ts_us = get_time_ms() * 1000;
	candle_usb_send_packet(usbd_dev, 2, &packet_busload, 4+sizeof(packet_busload.first_record.busload_info));
}


static uint8_t tx_in_use = 0;
static void candle_usb_tx_handler(usbd_device *usbd_dev, uint8_t ep) {
	tx_in_use++;
}

static void candle_usb_rx_handler(usbd_device *usbd_dev, uint8_t ep)
{
	uint8_t buf[256];
	int len = usbd_ep_read_packet(usbd_dev, ep, buf, sizeof(buf));
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

static void candle_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_register_control_callback(usbd_dev, 0x43, 0x7F, candle_control_request);

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_tx_handler);

	usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_tx_handler);

	usbd_ep_setup(usbd_dev, 0x03, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_tx_handler);

#ifdef USE_USB_HS
	usbd_ep_setup(usbd_dev, 0x04, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x84, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_tx_handler);

	usbd_ep_setup(usbd_dev, 0x05, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x85, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_tx_handler);
#endif

	//candle_usb_send_timestamp(usbd_dev, 2);
}

int main(void)
{

	usbd_device *usbd_dev;

	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
				GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
	systime_setup();

	gpio_set(GPIOD, GPIO12 | GPIO14);
//	while (1) {
//		gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);
//		delay_ms(100);
//	}

#ifdef USE_USB_HS
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_OTGHS);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF12, GPIO13 | GPIO14 | GPIO15);

	usbd_dev = usbd_init(&otghs_usb_driver, &dev, &config,
			usb_strings, 10,
			usbd_control_buffer, sizeof(usbd_control_buffer));
#else
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_OTGFS);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 10,
			usbd_control_buffer, sizeof(usbd_control_buffer));

#endif

	memset(&pcan_status, 0, sizeof(pcan_status));
	usbd_register_set_config_callback(usbd_dev, candle_set_config);

	while (1) {
		usbd_poll(usbd_dev);
		gpio_toggle(GPIOD, GPIO12 | GPIO13 | GPIO14 | GPIO15);

		if (pcan_status.timestamp_active && (pcan_status.t_next_timestamp <= get_time_ms())) {
			candle_usb_send_timestamp(usbd_dev, 2);
			pcan_status.t_next_timestamp = get_time_ms() + 1000;
		}

		for (int i=0; i<2; i++) {
			if (pcan_status.busload_mode[i] && (pcan_status.t_next_busload[i] <= get_time_ms())) {
				candle_usb_send_busload(usbd_dev, 2, i);
				pcan_status.t_next_busload[i] = get_time_ms() + 8;
			}
		}
	}
}
