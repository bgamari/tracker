#include <libopencm3/cm3/systick.h>

#include "clock.h"
#include "config.h"
#include "utils.h"

volatile uint32_t msTicks;      /* counts 1ms timeTicks */

volatile struct event_counters event_counters[N_EVENT_COUNTERS];
unsigned int active_event_counter_index = 0;
#define active_event_counters (&event_counters[active_event_counter_index])

struct event_counters get_last_event_counters(void)
{
    return event_counters[(active_event_counter_index-1) % N_EVENT_COUNTERS];
}

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

    if (msTicks % 1024 == 0) {
        unsigned int next = (active_event_counter_index + 1) % N_EVENT_COUNTERS;
        memzero((void*) &event_counters[next], sizeof(struct event_counters));
        active_event_counter_index = next;
    }
}
