/*
 * main.c
 *
 *  Created on: 24.07.2015
 *      Author: hd
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>

#include "config.h"
#include "can.h"
#include "ppro_protocol.h"
#include "ppro_usb.h"
#include "systime.h"

int main(void) {

	struct rcc_clock_scale clkcfg_96mhz = {
		.pllm = 4,
		.plln = 192,
		.pllp = 4,
		.pllq = 8,
		.flash_config = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS, // TODO check if this is sane (seems to work for 48MHz as well as 120MHz)
		.hpre = RCC_CFGR_HPRE_DIV_NONE,
		.ppre1 = RCC_CFGR_PPRE_DIV_4,
		.ppre2 = RCC_CFGR_PPRE_DIV_2,
		.power_save = 1,
		.ahb_frequency = 96000000,
		.apb1_frequency = 24000000,
		.apb2_frequency = 48000000
	};

	rcc_clock_setup_hse_3v3(&clkcfg_96mhz);
	//rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_120MHZ]);
	systime_setup(96000);

	candle_can_init();
	ppro_usb_init();
	candle_can_register_rx_callback(ppro_rx_message);

	while (1) {
		ppro_usb_poll();
		candle_can_poll();
	}
}
