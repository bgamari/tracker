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

#include "libmanyuc.h"
#include "dac.h"

Serial_t ser1, ser3;

enum sample_time_t {
    SAMPLE_TIME_3_CYCLES = 0x0,
    SAMPLE_TIME_15_CYCLES,
    SAMPLE_TIME_28_CYCLES,
    SAMPLE_TIME_56_CYCLES,
    SAMPLE_TIME_84_CYCLES,
    SAMPLE_TIME_112_CYCLES,
    SAMPLE_TIME_144_CYCLES,
    SAMPLE_TIME_480_CYCLES,
};

void set_sample_times(enum sample_time_t sample_time) {
    unsigned int tmp = 0;
    for (int i = 0; i < 8; i++) {
        tmp |= sample_time;
        tmp <<= 3;
    }
    ADC1->SMPR1 = tmp;
    ADC1->SMPR2 = tmp;
}

typedef unsigned int adc_channel_t;
void set_injected_sequence(ADC_TypeDef *adc,
                           unsigned int num_samples,
                           adc_channel_t channels[]) {
    if (num_samples == 0)
        return;
    if (num_samples > 4)
        num_samples = 4;
    adc->JSQR = num_samples-1;
    for (unsigned int i=0; i<4; i++) {
        adc->JSQR <<= 5;
        if (i < num_samples)
            adc->JSQR |= 0xf & channels[i];
    }
}

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
    spi_init();
    //i2s_init();

    // UART1 (Beagle)
    Pin_Init(ARM_PA9, 1, Alt7);
    Pin_Init(ARM_PA10, 1, Alt7);
    // UART3
    Pin_Init(ARM_PB10, 1, Alt7);
    Pin_Init(ARM_PB11, 1, Alt7);
    // SPI1 (Beagle)
    Pin_Init(ARM_PA4, 1, Alt5);
    Pin_Init(ARM_PA5, 1, Alt5);
    Pin_Init(ARM_PA6, 1, Alt5);
    Pin_Init(ARM_PA7, 1, Alt5);

    // ADC
    Pin_Init(ARM_PA0, 1, Analog); // ADC123_IN0
    Pin_Init(ARM_PA1, 1, Analog); // ADC123_IN1
    Pin_Init(ARM_PA2, 1, Analog); // ADC123_IN2
    Pin_Init(ARM_PA3, 1, Analog); // ADC123_IN3
    Pin_Init(ARM_PB0, 1, Analog); // ADC12_IN8
    Pin_Init(ARM_PB1, 1, Analog); // ADC12_IN9
    Pin_Init(ARM_PC4, 1, Analog); // ADC12_IN14
    Pin_Init(ARM_PC5, 1, Analog); // ADC12_IN15

    SYSCFG->CMPCR = 0x1; // Enable I/O compensation cell

    // Initialize ADC
    NVIC_EnableIRQ(ADC_IRQn);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR1 |= ADC_CR1_EOCIE | ADC_CR1_JEOCIE | ADC_CR1_SCAN;
    set_sample_times(SAMPLE_TIME_28_CYCLES);
    adc_channel_t channels[] = { 0, 1, 2, 3 };
    set_injected_sequence(ADC1, 4, channels);

    // Turn all leds on and then off,
    // with a delay of 0.2s among operations.
    // This goes on indefinitely
    Serial_Put_Bytes(ser1, 0, "non-dma\n", sizeof("non-dma\n"));
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
        ADC1->CR2 |= ADC_CR2_JSWSTART; // Start ADC conversion
        updates[0].value += 0x1000;
        set_dac(3, updates);
    }
}


void ADC_IRQHandler() {
    char tmp[64];
    int n = snprintf(tmp, 64, "ADC: %d, %d, %d, %d\n",
                     ADC1->JDR1, ADC1->JDR2, ADC1->JDR3, ADC1->JDR4);
    //Serial_Put_Bytes(ser1, 0, tmp, n);
    ADC1->SR = 0;
}

// vim:expandtab:smartindent:tabstop=4:softtabstop=4:shiftwidth=4:
