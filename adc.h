#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

enum adc_sample_time_t {
    SAMPLE_TIME_8CYC = 0x0,
};

// Subsystem initialization
void adc_init();
void adc_set_buffers(unsigned int length, uint16_t *buffer1, uint16_t *buffer2);
void adc_set_streaming(bool on);

// Triggering
int adc_set_trigger_freq(uint32_t freq);
int adc_trigger_start();
int adc_trigger_stop();
void adc_manual_trigger();

// Accessing sample buffers
uint16_t *adc_get_last_sample();

uint16_t *adc_get_active_buffer();
uint16_t *adc_get_inactive_buffer();
