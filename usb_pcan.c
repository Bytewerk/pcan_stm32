/*
 * usb_pcan.c
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#include "usb_pcan.h"

#include <string.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbd.h>
#include "usb_descriptor.h"
#include "commands.h"

pcan_status_t pcan_status;

static usbd_device *usbdev;
static uint8_t usbd_control_buffer[512];
static char firmware_version[] = {0x44,0x33,0x22,0x11,0x01,0x03,0x03,0x00,0x12,0x06,0x0a,0x40,0x00,0x01,0x00,0x00};
static char bootloader_version[] = {0x22,0x22,0x11,0x11,0x01,0x08,0x03,0x00,0x1a,0x05,0x09,0x00,0xff,0xff,0xff,0xff,0x01,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00};
static char unknown_req_4[] = { 0x3a, 0x39, 0x38, 0x37, 0x01, 0x00, 0x00, 0x00 };
static char unknown_req_5[] = { 0x77, 0x22, 0x1a, 0x1a, 0x07, 0x00, 0x00, 0x00 };

static void candle_usb_rx_handler(usbd_device *usbd_dev, uint8_t ep)
{
	uint8_t buf[256];
	int len = usbd_ep_read_packet(usbd_dev, ep, buf, sizeof(buf));
	usb_pcan_protocol_handle_data(usbd_dev, ep, buf, len);
}

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

static void candle_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_register_control_callback(usbd_dev, 0x43, 0x7F, candle_control_request);

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 128, NULL);

	usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 128, NULL);

	usbd_ep_setup(usbd_dev, 0x03, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_BULK, 128, NULL);

#ifdef USE_USB_HS
	usbd_ep_setup(usbd_dev, 0x04, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x84, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_tx_handler);

	usbd_ep_setup(usbd_dev, 0x05, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_rx_handler);
	usbd_ep_setup(usbd_dev, 0x85, USB_ENDPOINT_ATTR_BULK, 128, candle_usb_tx_handler);
#endif

	//candle_usb_send_timestamp(usbd_dev, 2);
}


void usb_pcan_init(void) {
	memset(&pcan_status, 0, sizeof(pcan_status));

#ifdef USE_USB_HS
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_OTGHS);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO13 | GPIO14 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF12, GPIO13 | GPIO14 | GPIO15);

	usbdev = usbd_init(&otghs_usb_driver, &device_descriptor, &config,
			usb_strings, 10,
			usbd_control_buffer, sizeof(usbd_control_buffer));
#else
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_OTGFS);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

	usbdev = usbd_init(&otgfs_usb_driver, &device_descriptor, &config_descriptor,
			usb_strings, 10,
			usbd_control_buffer, sizeof(usbd_control_buffer));

#endif


	usbd_register_set_config_callback(usbdev, candle_set_config);
}

void usb_pcan_poll(void) {
	usbd_poll(usbdev);


	if (pcan_status.timestamp_active && (pcan_status.t_next_timestamp <= get_time_ms())) {
		candle_usb_send_timestamp(usbdev, 2);
		pcan_status.t_next_timestamp = get_time_ms() + 1000;
	}

	for (int i=0; i<2; i++) {
		if (pcan_status.busload_mode[i] && (pcan_status.t_next_busload[i] <= get_time_ms())) {
			candle_usb_send_busload(usbdev, 2, i);
			pcan_status.t_next_busload[i] = get_time_ms() + 8;
		}
	}
}
