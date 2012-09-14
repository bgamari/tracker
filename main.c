/*
 * libmanyuc - Turn leds on in C example
 * Copyright (C) 2012 - Margarita Manterola Rivero
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

#include <math.h>
#include "libmanyuc.h"

#include "tracker.h"
#include "dac.h"
#include "beagle_spi.h"
#include "feedback.h"

Serial_t ser1, ser3;

struct dac_update_t updates[] = {
    { channel_a, 0x4400 },
    { channel_b, 0x4400 },
    { channel_c, 0x4400 },
    { channel_d, 0x4400 },
    { channel_e, 0x4400 },
    { channel_f, 0x4400 },
    { channel_g, 0x4400 },
    { channel_g, 0x4400 },
};

/* This example turns all 4 leds on and then off */
int main(void) {
    // Access 4 leds predefined as LED1, LED2, LED3, LED4
    Pin_t leds[] = {
        Pin_Init(LED1, 1, Output),
        Pin_Init(LED2, 1, Output),
        Pin_Init(LED3, 1, Output),
        Pin_Init(LED4, 1, Output)
    };
    int nleds = 4, i = 0;
    ser1 = Serial_Init(0, 115200);
    ser3 = Serial_Init(2, 115200);

    // UART1 (Beagle)
    Pin_Init(ARM_PA9, 1, Alt7);
    Pin_Init(ARM_PA10, 1, Alt7);
    // UART3
    Pin_Init(ARM_PB10, 1, Alt7);
    Pin_Init(ARM_PB11, 1, Alt7);

    beagle_spi_init();
    adc_init();
    dac_spi_init();
    //dac_i2s_init();
    feedback_init();

    SYSCFG->CMPCR = 0x1; // Enable I/O compensation cell

    // Turn all leds on and then off,
    // with a delay of 0.2s among operations.
    // This goes on indefinitely
    Serial_Put_Bytes(ser1, 0, "non-dma\n", sizeof("non-dma\n"));
    float t = 0;
    while (1) {
#if 0
        for (i = 0; i < nleds; i++) {
            Pin_On(leds[i]);
            Delay(0.2);
        }
        for (i = 0; i < nleds; i++) {
            Pin_Off(leds[i]);
            Delay(0.2);
        }
        Serial_Put_Bytes(ser1, NONBLOCKING, "hello world!\n", sizeof("hello world!\n"));
#endif
        //ADC1->CR2 |= ADC_CR2_JSWSTART; // Start ADC conversion
        updates[0].value = 0x7fff * 0.5*(sin(t) + 1);
        //updates[0].value += 0x100;
        t += 1e-4;
        set_dac(3, updates);
    }
}

// vim:expandtab:smartindent:tabstop=4:softtabstop=4:shiftwidth=4:
