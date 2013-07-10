#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

// Subsystem initialization and configuration
void adc_init();

enum adc_sample_time_t {
    SAMPLE_TIME_8CYC = 0x0,
};

void adc_set_sample_time(enum adc_sample_time_t time);

// Streaming samples into buffer
typedef uint16_t* (*adc_buffer_done_cb)(uint16_t*);
void adc_start(unsigned int samples, uint16_t* buf, adc_buffer_done_cb done);

// Triggering
enum trigger_mode { TRIGGER_OFF, TRIGGER_AUTO, TRIGGER_MANUAL };
void adc_set_trigger_mode(enum trigger_mode mode);
int adc_set_trigger_freq(uint32_t freq);
int adc_manual_trigger();

// Accessing sample buffers
uint16_t *adc_get_last_sample();
uint16_t *adc_get_active_buffer();
