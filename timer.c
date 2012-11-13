#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/rcc.h>

#include "timer.h"
#include "clock.h"

void setup_periodic_timer(uint32_t timer, unsigned int freq_in_hz)
{
    unsigned int prescaler = 1;
    while (rcc_ppre1_frequency / prescaler / freq_in_hz > 0xffff)
        prescaler *= 2;
    timer_reset(timer);
    timer_set_prescaler(timer, prescaler-1);
    timer_set_period(timer, rcc_ppre1_frequency / prescaler / freq_in_hz);
    timer_direction_down(timer);
    timer_enable_preload(timer);
}
