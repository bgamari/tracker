#pragma once

#include "tracker.h"

typedef unsigned int adc_channel_t;

struct adc_sample_t {
  uint16_t channel[N_INPUTS];
};

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

void adc_init();

void set_sample_times(enum sample_time_t sample_time);

void set_injected_sequence(ADC_TypeDef *adc,
                           unsigned int num_samples,
                           adc_channel_t channels[]);

void set_regular_sequence(ADC_TypeDef *adc,
                          unsigned int num_samples,
                          adc_channel_t channels[]);

void adc_dma_start(unsigned int nsamples, struct adc_sample_t *buf);