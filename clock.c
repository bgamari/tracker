#include "clock.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f4/rcc.h>

volatile uint32_t msTicks;      /* counts 1ms timeTicks */

void init_clock()
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);
}

void init_systick()
{
    systick_set_reload(168e6 / 1000);
    systick_interrupt_enable();
    systick_counter_enable();
}

void delay_ms(unsigned int ms) {
    uint32_t curTicks;
    curTicks = msTicks;
    while ((msTicks - curTicks) < ms);
}

void sys_tick_handler(void) {
    msTicks++;
}
