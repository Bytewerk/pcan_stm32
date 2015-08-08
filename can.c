/*
 * can.c
 *
 *  Created on: 25.07.2015
 *      Author: hd
 */

#include "can.h"
#include "systime.h"
#include "can_queue.h"
#include <string.h>
#include <assert.h>
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

typedef struct {
	can_queue_item_t txqueue_items[TX_QUEUE_LENGTH];
	can_queue_t tx_queue;
	can_queue_t tx_message_pool;
} channel_data_t;

static channel_data_t channel_data[2];

static void reset_can(uint32_t can) {
	// set reset bit in master control register
	CAN_MCR(can) |= CAN_MCR_RESET;
	// wait for reset bit to become zero again (reset complete?)
	while (CAN_MCR(can) & CAN_MCR_RESET);
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

static int init_can(uint32_t can) {

	if (candle_can_goto_init_mode(can)<0) {
		return -1;
	}

	// use automatic bus-off management
	// (leave bus-off state automatically if can status seems to be okay)
	CAN_MCR(can) |= CAN_MCR_ABOM;

	// lock full rx fifo (throw away newer messages)
	CAN_MCR(can) |= CAN_MCR_RFLM;

	// use tx mailboxes in fifo mode
	CAN_MCR(can) |= CAN_MCR_TXFP;

	return 0;
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

static void clear_tx_queue(uint8_t channel) {
	assert((channel==0) || (channel==1));
	channel_data_t *data = &channel_data[channel];

	can_queue_init(&data->tx_queue);
	can_queue_init(&data->tx_message_pool);
	memset(data->txqueue_items, 0, sizeof(data->txqueue_items));
	for (int i=0; i<TX_QUEUE_LENGTH; i++) {
		can_queue_push_back(&data->tx_message_pool, &data->txqueue_items[i]);
	}
}

void candle_can_init(void) {
	rx_callback = 0;
	memset(led_status, 0, sizeof(led_status));

	clear_tx_queue(0);
	clear_tx_queue(1);

	// enable led outputs
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);

	// enable can1 peripheral
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_CAN1);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0 | GPIO1);
	gpio_set_af(GPIOD, GPIO_AF9, GPIO0 | GPIO1);
	reset_can(CAN1);
	init_can(CAN1);

	// enable can1 transceiver
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_clear(GPIOC, GPIO6);

	// enable can2 peripheral
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_CAN2);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO12 | GPIO13);
	gpio_set_af(GPIOB, GPIO_AF9, GPIO12 | GPIO13);
	reset_can(CAN2);
	init_can(CAN2);

	// enable can2 transceiver
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
	gpio_clear(GPIOD, GPIO11);

	// init filter banks
	CAN_FMR(CAN1) |= CAN_FMR_FINIT; // switch filter banks to init mode

	// configure usage of 14 filter banks for can1 and 14 banks for can2
	CAN_FMR(CAN1) &= ~CAN_FMR_CAN2SB_MASK;
	CAN_FMR(CAN1) |= (14<<CAN_FMR_CAN2SB_SHIFT);

	can_filter_id_mask_32bit_init(CAN1,  0, 0, 0, 0, 1); // set a catch-all filter for CAN1 fifo 0
	can_filter_id_mask_32bit_init(CAN2, 14, 0, 0, 0, 1); // set a catch-all filter for CAN2 fifo 0

	CAN_FMR(CAN1) &= ~CAN_FMR_FINIT; // switch filter banks to active mode

}

void can_poll_leds(void);

static void candle_can_handle_tx_queue(uint8_t channel);
static void candle_can_poll_rx(uint8_t channel);

void candle_can_poll(void) {

	candle_can_poll_rx(0);
	candle_can_poll_rx(1);

	candle_can_handle_tx_queue(0);
	candle_can_handle_tx_queue(1);

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

	/*
	 * it's possible that we get messages via usb faster than we can send them out.
	 * in that case, we want to queue them up.
	 * for simplicity, we just put them in a queue here and leave the transmit handling
	 * to candle_can_poll() / candle_can_handle_tx_queue()
	 */

	assert( (msg->channel==0) || (msg->channel==1) );
	channel_data_t *data = &channel_data[msg->channel];
	can_queue_item_t *item;

	if (data->tx_message_pool.count==0) {
		// no free slot in tx queue, try sending out messages
		candle_can_handle_tx_queue(msg->channel);
	}

	if (can_queue_pop_front(&data->tx_message_pool, &item)) {
		item->msg = *msg;
		can_queue_push_back(&data->tx_queue, item);

		// try to send message immediately, if possible
		candle_can_handle_tx_queue(msg->channel);
	} else {
		// still no free slot in tx queue
	}
}

static uint32_t find_empty_mailbox(uint32_t can) {
	if ((CAN_TSR(can) & CAN_TSR_TME0) == CAN_TSR_TME0) {
		return CAN_MBOX0;
	} else if ((CAN_TSR(can) & CAN_TSR_TME1) == CAN_TSR_TME1) {
		return CAN_MBOX1;
	} else if ((CAN_TSR(can) & CAN_TSR_TME2) == CAN_TSR_TME2) {
		return CAN_MBOX2;
	} else {
		return 0;
	}
}

static void send_can_message(uint32_t can, uint32_t mailbox, can_message_t *msg) {
	uint32_t id_and_flags = msg->id_and_flags;

	if (id_and_flags & can_flag_extid) {
		CAN_TIxR(can, mailbox) = CAN_TIxR_IDE | ((id_and_flags & 0x1FFFFFFF) << CAN_TIxR_EXID_SHIFT);
	} else {
		CAN_TIxR(can, mailbox) = (id_and_flags & 0x7FF) << CAN_TIxR_STID_SHIFT;
	}

	if (id_and_flags & can_flag_rtr) {
		CAN_TIxR(can, mailbox) |= CAN_TIxR_RTR;
	}

	CAN_TDTxR(can, mailbox) &= ~CAN_TDTxR_DLC_MASK;
	CAN_TDTxR(can, mailbox) |= (msg->dlc & CAN_TDTxR_DLC_MASK);
	CAN_TDHxR(can, mailbox) = msg->data32[1];
	CAN_TDLxR(can, mailbox) = msg->data32[0];
	CAN_TIxR(can, mailbox) |= CAN_TIxR_TXRQ;
}

static void candle_can_handle_tx_queue(uint8_t channel) {
	assert( (channel==0) || (channel==1) );
	uint32_t can = (channel==0) ? CAN1 : CAN2;
	channel_data_t *data = &channel_data[channel];

	if (data->tx_queue.count>0) {
		uint32_t mailbox = find_empty_mailbox(can);
		if (mailbox) {
			can_queue_item_t *item;
			if (can_queue_pop_front(&data->tx_queue, &item)) {
				send_can_message(can, mailbox, &item->msg);

				// return message buffer to pool queue
				can_queue_push_back(&data->tx_message_pool, item);
			}
		}
	}
}

static void candle_can_handle_fifo(uint8_t channel, uint32_t fifo) {
	assert( (channel==0) || (channel==1) );
	uint32_t can = (channel==0) ? CAN1 : CAN2;

	can_message_t msg;
	msg.channel = channel;
	msg.timestamp = get_time_us32();

	uint32_t RIR = CAN_RIxR(can, fifo);

	if (RIR & CAN_RIxR_IDE) {
		msg.id_and_flags = can_flag_extid | ((RIR >> CAN_RIxR_EXID_SHIFT) & CAN_RIxR_EXID_MASK);
	} else {
		msg.id_and_flags = (RIR >> CAN_RIxR_STID_SHIFT) & CAN_RIxR_STID_MASK;
	}

	if (RIR & CAN_RIxR_RTR) {
		msg.id_and_flags |= can_flag_rtr;
	}

	msg.dlc = CAN_RDTxR(can, fifo) & CAN_RDTxR_DLC_MASK;
	msg.data32[0] = CAN_RDLxR(can, fifo);
	msg.data32[1] = CAN_RDHxR(can, fifo);

	can_notify_message(&msg);
}

static void candle_can_poll_rx(uint8_t channel) {
	assert( (channel==0) || (channel==1) );
	uint32_t can = (channel==0) ? CAN1 : CAN2;

	while (CAN_RF0R(can) & CAN_RF0R_FMP0_MASK) { // there are messages waiting in FIFO0
		candle_can_handle_fifo(channel, CAN_FIFO0);
		CAN_RF0R(can) |= CAN_RF1R_RFOM1; // release fifo message
	}
}

void candle_can_set_bitrate(uint8_t channel, uint16_t brp, uint8_t tseg1, uint8_t tseg2, uint8_t sjw) {
	uint32_t can = (channel==0) ? CAN1 : CAN2;

	int was_in_init_mode = candle_is_in_init_mode(can);
	if (!was_in_init_mode) {
		candle_can_goto_init_mode(can);
	}

	uint32_t cfg = CAN_BTR(can);

	if (sjw>4) { sjw = 4; }
	cfg &= ~CAN_BTR_SJW_MASK;
	cfg |= ((sjw-1)<<CAN_BTR_SJW_SHIFT);

	if (tseg2>8) { tseg2 = 8; }
	cfg &= ~CAN_BTR_TS2_MASK;
	cfg |= ((tseg2-1)<<CAN_BTR_TS2_SHIFT);

	if (tseg1>16) { tseg1 = 16; }
	cfg &= ~CAN_BTR_TS1_MASK;
	cfg |= ((tseg1-1)<<CAN_BTR_TS1_SHIFT);

	if (brp>1024) { brp = 1024; }
	cfg &= ~CAN_BTR_BRP_MASK;
	cfg |= (brp-1);

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
