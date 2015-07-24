/*
 * main.c
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#include <libopencm3/stm32/rcc.h>

#include "config.h"
#include "usb_pcan.h"
#include "systime.h"

int main(void) {
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);
	systime_setup();
	usb_pcan_init();

	while (1) {
		usb_pcan_poll();
	}
}
