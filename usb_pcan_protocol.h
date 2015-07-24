/*
 * usb_pcan_protocol.h
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#ifndef USB_PCAN_PROTOCOL_H_
#define USB_PCAN_PROTOCOL_H_

#include <stdint.h>
#include <libopencm3/usb/usbd.h>

void usb_pcan_protocol_handle_data(usbd_device *usbd_dev, uint8_t ep, uint8_t *buf, int len);
void candle_usb_send_timestamp(usbd_device *usbd_dev, uint8_t ep);
void candle_usb_send_busload(usbd_device *usbd_dev, uint8_t ep, uint8_t channel);


#endif /* USB_PCAN_PROTOCOL_H_ */
