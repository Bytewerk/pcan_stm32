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

#include "systime.h"
#include "pcan_usbpro_fw.h"
#include "usb_descriptor.h"
#include "usb_pcan_protocol.h"

pcan_status_t pcan_status;

static usbd_device *usbdev;
static uint8_t usbd_control_buffer[512];

static struct pcan_usbpro_bootloader_info_t bootloader_info = {
	.ctrl_type = 0x11112222,
	.version = {1, 8, 3, 0},
	.day = 26,
	.month = 5,
	.year = 9,
	.dummy = 0,
	.serial_num_high = 0xFFFFFFFF,
	.serial_num_low  = 0x00000001,
	.hw_type = 0x00000100,
	.hw_rev = 0x00000000
};

static struct pcan_usbpro_ext_firmware_info_t firmware_info = {
	.ctrl_type = 0x11223344,
	.version = {1, 3, 3, 0 },
	.day = 18,
	.month = 6,
	.year = 10,
	.dummy = 0x40,
	.fw_type = 0x00000100
};

static struct pcan_usbpro_uc_chipid_t uc_chip_id = {
	.ctrl_type = 0, // TODO find valid values
	.chip_id = 0
};

static struct pcan_usbpro_usb_chipid_t usb_chip_id = {
		.ctrl_type = 0, // TODO find valid values
		.chip_id = 0
};

static struct pcan_usbpro_device_nr_t device_nr = {
	.ctrl_type = 0x3738393a,
	.device_nr = 1
};

static struct pcan_usbpro_cpld_info_t cpld_info = {
	.ctrl_type = 0x1a1a2277,
	.cpld_nr = 7
};

static struct pcan_usbpro_info_mode_t info_mode = {
	.ctrl_type = 0, // TODO find valid values
	.mode = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
	.flags = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }
};

static struct pcan_usbpro_time_mode_t time_mode = {
	.ctrl_type = 0,
	.time_mode = 0,
	.flags = 0
};

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

	if (req->bRequest == 0) { // get information

		switch (req->wValue) {

			case USB_VENDOR_REQUEST_wVALUE_INFO_BOOTLOADER:
				memcpy(*buf, &bootloader_info, sizeof(bootloader_info));
				*len = sizeof(bootloader_info);
				break;

			case USB_VENDOR_REQUEST_wVALUE_INFO_FIRMWARE:
				memcpy(*buf, &firmware_info, sizeof(firmware_info));
				*len = sizeof(firmware_info);
				break;

			case USB_VENDOR_REQUEST_wVALUE_INFO_uC_CHIPID:
				memcpy(*buf, &uc_chip_id, sizeof(uc_chip_id));
				*len = sizeof(uc_chip_id);
				break;

			case USB_VENDOR_REQUEST_wVALUE_INFO_USB_CHIPID:
				memcpy(*buf, &usb_chip_id, sizeof(usb_chip_id));
				*len = sizeof(usb_chip_id);
				break;

			case USB_VENDOR_REQUEST_wVALUE_INFO_DEVICENR:
				memcpy(*buf, &device_nr, sizeof(device_nr));
				*len = sizeof(device_nr);
				break;

			case USB_VENDOR_REQUEST_wVALUE_INFO_CPLD:
				memcpy(*buf, &cpld_info, sizeof(cpld_info));
				*len = sizeof(cpld_info);
				break;

			case USB_VENDOR_REQUEST_wVALUE_INFO_MODE:
				memcpy(*buf, &info_mode, sizeof(info_mode));
				*len = sizeof(info_mode);
				break;

			case USB_VENDOR_REQUEST_wVALUE_INFO_TIMEMODE:
				memcpy(*buf, &time_mode, sizeof(time_mode));
				*len = sizeof(time_mode);
				break;

		}

		return 1;

	} else if (req->bRequest == 2) { // config ?
		if (req->wValue==USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED) {
			usbd_ep_read_packet(usbd_dev, 0, buffer, 16);
			// data is 16 bytes, of which only 2 are used:
			// data[0] == driver (CAN=0, LIN=1)
			// data[1] == driver loaded (0/1)
	        return USBD_REQ_HANDLED;
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

	for (uint8_t i=0; i<2; i++) {
		if (pcan_status.busload_mode[i] && (pcan_status.t_next_busload[i] <= get_time_ms())) {
			candle_usb_send_busload(usbdev, 2, i);
			pcan_status.t_next_busload[i] = get_time_ms() + 8;
		}
	}
}
