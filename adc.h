#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

// Subsystem initialization and configuration
void adc_init();

enum adc_oversampling_t {
    ADC_OVERSAMPLE_NONE = 0x0,
    ADC_OVERSAMPLE_2    = 0x1,
    ADC_OVERSAMPLE_4    = 0x2,
    ADC_OVERSAMPLE_8    = 0x3,
    ADC_OVERSAMPLE_16   = 0x4,
    ADC_OVERSAMPLE_32   = 0x5,
    ADC_OVERSAMPLE_64   = 0x6,
};

void adc_set_oversampling(enum adc_oversampling_t os);

enum adc_range_t {
    ADC_RANGE_PLUS_MINUS_5  = 0,
    ADC_RANGE_PLUS_MINUS_10 = 1,
};

void adc_set_range(enum adc_range_t range);

// Streaming samples into buffer
typedef int16_t* (*adc_buffer_done_cb)(int16_t*);
void adc_start(unsigned int samples, int16_t* buf, adc_buffer_done_cb done);
unsigned int adc_get_decimation();
int adc_set_decimation(const unsigned int decimation_factor);

// Triggering
enum trigger_mode { TRIGGER_OFF, TRIGGER_AUTO, TRIGGER_MANUAL };
void adc_set_trigger_mode(enum trigger_mode mode);
int adc_set_trigger_freq(uint32_t freq);
int adc_manual_trigger();

// Accessing sample buffers
int16_t *adc_get_last_frame();
int16_t *adc_get_active_buffer();
