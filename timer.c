#include <libopencm3/lpc43xx/timer.h>

#include "config.h"
#include "timer.h"

// Uses match 3 for reset
void setup_periodic_timer(uint32_t timer, unsigned int freq_in_hz)
{
    timer_disable_counter(timer);
    timer_reset(timer);
    TIMER_MR3(timer) = CLK_BASE_M4 / freq_in_hz;
    TIMER_MCR(timer) &= ~(TIMER_MCR_MR3I | TIMER_MCR_MR3R | TIMER_MCR_MR3S);
    TIMER_MCR(timer) |= TIMER_MCR_MR3R;
}
