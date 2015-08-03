/*
 * can.c
 *
 *  Created on: 25.07.2015
 *      Author: hd
 */

#include "can.h"
#include "systime.h"
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/can.h>

static can_rx_callback_t rx_callback = 0;

typedef struct {
	led_mode_t mode;
	uint32_t t_next;
	uint8_t state;
} led_status_t;

static led_status_t led_status[2];


static void candle_reset_can(uint32_t can) {
	// set reset bit in master control register
	CAN_MCR(can) |= CAN_MCR_RESET;
	// wait for reset bit to become zero again (reset complete?)
	while (CAN_MCR(can) & CAN_MCR(can));
}

static int candle_can_goto_init_mode(uint32_t can) {
	// set initialization request bit in MCR (must also clear SLEEP bit)
	CAN_MCR(can) &= ~CAN_MCR_SLEEP;
	CAN_MCR(can) |= CAN_MCR_INRQ;

	// wait for initialization mode confirmation
	for (int i=0; i<0xFFFF; i++) {
		if ((CAN_MSR(can) & CAN_MSR_INAK) == CAN_MSR_INAK) {
			return 0; // controller is in initialization mode
		}
	}

	return -1; // timeout waiting for INAK flag
}

static int candle_is_in_init_mode(uint32_t can) {
	return (CAN_MCR(can) & CAN_MCR_INRQ) && (CAN_MSR(can) & CAN_MSR_INAK);
}

static int candle_can_goto_normal_mode(uint32_t can) {
	// clear initialization request bit
	CAN_MCR(can) &= ~CAN_MCR_INRQ;

	for (int i=0; i<0x0FFFFF; i++) {
		if ((CAN_MSR(can) & CAN_MSR_INAK) == 0) {
			return 0; // controller is not in initialization mode any more
		}
	}

	return -1; // timeout waiting for INAK flag to become zero
}

void candle_can_init(void) {
	rx_callback = 0;
	memset(led_status, 0, sizeof(led_status));

	// enable led outputs
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);

	// enable can1 peripheral
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_CAN1);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1);
	gpio_set_af(GPIOD, GPIO_AF9, GPIO0 | GPIO1);
	candle_reset_can(CAN1);
	candle_can_goto_init_mode(CAN1);

	// enable can2 peripheral
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_CAN2);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO12 | GPIO13);
	gpio_set_af(GPIOB, GPIO_AF9, GPIO12 | GPIO13);
	candle_reset_can(CAN2);
	candle_can_goto_init_mode(CAN2);

}

void can_poll_leds(void);

void candle_can_poll(void) {
	can_poll_leds();
}

void candle_can_register_rx_callback(can_rx_callback_t callback) {
	rx_callback = callback;
}

void can_notify_message(const can_message_t *msg);
void can_notify_message(const can_message_t *msg) {
	if (rx_callback != 0) {
		rx_callback(msg);
	}
}

void candle_can_send_message(const can_message_t *msg) {
	// TODO really implement me

	can_message_t clone = *msg;
	if (clone.channel==0) {
		clone.channel = 1;
	} else {
		clone.channel = 0;
	}
	can_notify_message(&clone);
}

void candle_can_set_bitrate(uint8_t channel, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw) {
	uint32_t can = (channel==0) ? CAN1 : CAN2;

	int was_in_init_mode = candle_is_in_init_mode(can);
	if (!was_in_init_mode) {
		candle_can_goto_init_mode(can);
	}

	uint32_t cfg = CAN_BTR(can);

	if (sjw>3) { sjw = 3; }
	cfg &= ~CAN_BTR_SJW_MASK;
	cfg |= (sjw<<CAN_BTR_SJW_SHIFT);

	if (tseg2>7) { tseg2 = 7; }
	cfg &= ~CAN_BTR_TS2_MASK;
	cfg |= (tseg2<<CAN_BTR_TS2_SHIFT);

	if (tseg1>15) { tseg1 = 15; }
	cfg &= ~CAN_BTR_TS1_MASK;
	cfg |= (tseg1<<CAN_BTR_TS1_SHIFT);

	if (brp>1023) { brp = 1023; }
	cfg &= ~CAN_BTR_BRP_MASK;
	cfg |= brp;

	CAN_BTR(can) = cfg;

	if (!was_in_init_mode) {
		candle_can_goto_normal_mode(can);
	}
}

void candle_can_set_silent(uint8_t channel, uint8_t silent_mode) {
	uint32_t can = (channel==0) ? CAN1 : CAN2;

	int was_in_init_mode = candle_is_in_init_mode(can);
	if (!was_in_init_mode) {
		candle_can_goto_init_mode(can);
	}

	if (silent_mode) {
		CAN_BTR(can) = CAN_BTR(can) | 0x80000000; // set SILM bit
	} else {
		CAN_BTR(can) = CAN_BTR(can) & 0x7FFFFFFF; // clear SILM bit
	}

	if (!was_in_init_mode) {
		candle_can_goto_normal_mode(can);
	}
}

void candle_can_set_bus_active(uint8_t channel, uint16_t mode) {
	uint32_t can = (channel==0) ? CAN1 : CAN2;
	if (mode) {
		candle_can_goto_normal_mode(can);
	} else {
		candle_can_goto_init_mode(can);
	}
}

uint8_t candle_can_calc_message_len(const can_message_t *msg) {
	// NOTICE this ignores stuff bits and is therefore incorrect
	// to calculate the number of stuff bits, we would have to construct
	// the whole can frame which would probably be a lot of overhead
	if (msg->id_and_flags & can_flag_extid) {
		return 64 + 8*msg->dlc + 3;
	} else {
		return 44 + 8*msg->dlc + 3;
	}
}


void candle_can_set_led_mode(uint8_t channel, led_mode_t mode, uint32_t timeout) {
	(void)timeout;
	if (channel < 2) {
		led_status[channel].mode = mode;
		led_status[channel].t_next = get_time_ms();
	}
}

static void set_led_state(uint8_t channel, uint8_t state) {
	uint16_t pin = (channel==0) ? GPIO12 : GPIO14;
	if (state) {
		gpio_set(GPIOD, pin);
	} else {
		gpio_clear(GPIOD, pin);
	}
}

void can_poll_leds(void) {

	uint32_t now = get_time_ms();

	for (uint8_t i=0; i<2; i++) {
		led_status_t *status = &led_status[i];

		if ( status->t_next && (status->t_next <= now)) {

			switch (status->mode) {
				case can_led_mode_auto:
				case can_led_mode_off:
					status->state = 0;
					status->t_next = 0;
					break;
				case can_led_mode_on:
					status->state = 1;
					status->t_next = 0;
					break;
				case can_led_mode_blink_fast:
					status->state = !status->state;
					status->t_next += 100;
					break;
				case can_led_mode_blink_slow:
					status->state = !status->state;
					status->t_next += 500;
					break;
			}
		}

		set_led_state(i, status->state);
	}
}
