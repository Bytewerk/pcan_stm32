/*
 * systime.h
 *
 *  Created on: 21.07.2015
 *      Author: hd
 */

#ifndef SYSTIME_H_
#define SYSTIME_H_

#include <stdint.h>

void delay_ms(uint32_t delay);
uint32_t get_time_ms(void);
void systime_setup(void);

#endif /* SYSTIME_H_ */
