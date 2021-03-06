/*
 * usb_descriptor.h
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#ifndef PPRO_USB_DESCR_H_
#define PPRO_USB_DESCR_H_

#include <libopencm3/usb/usbd.h>

extern const struct usb_device_descriptor device_descriptor;
extern const struct usb_config_descriptor config_descriptor;
extern const char *usb_strings[6];


#endif /* PPRO_USB_DESCR_H_ */
