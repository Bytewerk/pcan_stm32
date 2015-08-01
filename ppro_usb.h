/*
 * usb_pcan.h
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#ifndef PPRO_USB_H_
#define PPRO_USB_H_

#include <stdint.h>

typedef struct {
	uint8_t timestamp_active;
	uint32_t t_next_timestamp;
	uint8_t busload_mode[2];
	uint32_t t_next_busload[2];
} ppro_status_t;

extern ppro_status_t ppro_status;


void ppro_usb_init(void);
void ppro_usb_poll(void);

#endif /* PPRO_USB_H_ */
