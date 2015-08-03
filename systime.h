/*
 * systime.h
 *
 *  Created on: 21.07.2015
 *      Author: hd
 */

#ifndef SYSTIME_H_
#define SYSTIME_H_

#include <stdint.h>

void systime_setup(uint32_t cpufreq_kHz);

uint32_t get_time_ms(void);
uint32_t get_time_us32(void);
uint64_t get_time_us64(void);

void delay_ms(uint32_t delay);

#endif /* SYSTIME_H_ */
