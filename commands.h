/*
 * commands.h
 *
 *  Created on: 14.07.2015
 *      Author: hd
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <stdint.h>
#include "pcan_usbpro_fw.h"

typedef struct {
	uint16_t record_count;
	uint16_t message_counter;
	union pcan_usbpro_rec_t first_record;
} candle_usb_packet_t;


#endif /* COMMANDS_H_ */
