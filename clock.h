#pragma once
#include <stdint.h>

extern volatile uint32_t msTicks;      /* counts 1ms timeTicks */

void init_systick();
void delay_ms(unsigned int ms);

