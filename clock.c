#include "clock.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/rcc.h>

volatile uint32_t msTicks;      /* counts 1ms timeTicks */

void init_clock()
{
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3);
    rcc_perhipheral_enable_clock(&RCC_AHB1_ENR, RCC_AHB1ENR_IOPDEN);
}

void init_systick()
{
    systick_set_reload();
    systick_interrupt_enable();
    systick_counter_enable();
}

void Delay_ms(unsigned int ms) {
    uint32_t curTicks;
    curTicks = msTicks;
    while ((msTicks - curTicks) < ms);
}

void SysTick_Handler(void) {
    msTicks++;
}
