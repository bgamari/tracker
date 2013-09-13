#pragma once
#include <stdint.h>

extern volatile uint32_t msTicks;      /* counts 1ms timeTicks */

void init_systick();
void delay_ms(unsigned int ms);

struct event_counters {
    unsigned int feedback_counter;
    unsigned int adc_sample_counter;
};

// public
struct event_counters get_last_event_counters(void);

// private
#define N_EVENT_COUNTERS 4
extern volatile struct event_counters event_counters[N_EVENT_COUNTERS];
extern unsigned int active_event_counter_index;

// public
#define increment_event_counter(counter) \
    event_counters[active_event_counter_index].counter++;

