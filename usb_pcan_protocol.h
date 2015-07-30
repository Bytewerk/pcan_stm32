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
#include "can.h"

void ppro_usb_protocol_init(usbd_device *usbd_dev);
void ppro_usb_protocol_handle_data(uint8_t ep, uint8_t *buf, int len);
void ppro_usb_send_timestamp(uint8_t ep);
void ppro_usb_send_busload(uint8_t ep, uint8_t channel);
void ppro_rx_message(const can_message_t *msg);


#endif /* USB_PCAN_PROTOCOL_H_ */
