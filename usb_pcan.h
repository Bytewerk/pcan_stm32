/*
 * usb_pcan.h
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#ifndef USB_PCAN_H_
#define USB_PCAN_H_

#include <stdint.h>

typedef struct {
	uint8_t timestamp_active;
	uint32_t t_next_timestamp;
	uint8_t busload_mode[2];
	uint32_t t_next_busload[2];
} pcan_status_t;

extern pcan_status_t pcan_status;


void usb_pcan_init(void);
void usb_pcan_poll(void);

#endif /* USB_PCAN_H_ */
