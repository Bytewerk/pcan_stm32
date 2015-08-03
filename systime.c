#include "systime.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

volatile uint32_t systick_ms;
static uint32_t cpufreq = 1;

void sys_tick_handler(void) {
	systick_ms++;
}

void systime_setup(uint32_t cpufreq_kHz) {
	cpufreq = cpufreq_kHz;
	systick_set_reload(cpufreq_kHz-1);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	systick_interrupt_enable();
}

uint32_t get_time_ms(void) {
	return systick_ms;
}

uint32_t get_time_us32(void) {
	uint32_t us = 1000ul*systick_ms;
	uint32_t ticks = systick_get_value();
	return us + (1000ul*ticks)/cpufreq;
}

uint64_t get_time_us64(void) {
	uint64_t us = 1000ull*systick_ms;
	uint32_t ticks = systick_get_value();
	return us + (1000ull*ticks)/cpufreq;
}

void delay_ms(uint32_t delay) {
	uint32_t wake = systick_ms + delay;
	while (wake > systick_ms);
}
