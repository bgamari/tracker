#include "clock.h"
#include "config.h"

#include <libopencm3/cm3/systick.h>

volatile uint32_t msTicks;      /* counts 1ms timeTicks */

void init_systick()
{
    systick_set_reload(CLK_BASE_M4 / 8 / 1000);
    systick_interrupt_enable();
    systick_counter_enable();
}

void delay_ms(unsigned int ms) {
    uint32_t curTicks;
    curTicks = msTicks;
    while ((msTicks - curTicks) < ms);
}

void sys_tick_handler() {
    msTicks++;
}
