/*
 * can.h
 *
 *  Created on: 25.07.2015
 *      Author: hd
 */

#ifndef CAN_H_
#define CAN_H_

#include <stdint.h>

void can_set_bitrate(uint8_t channel, uint8_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw);
void can_set_silent(uint8_t channel, uint8_t silent_mode);

#endif /* CAN_H_ */
