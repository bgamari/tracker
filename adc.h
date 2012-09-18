#pragma once

#include <stdbool.h>
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

struct adc_t {
    ADC_TypeDef *adc;

    // DMA state
    DMA_TypeDef *dma;
    DMA_Stream_TypeDef *dma_stream;
    uint8_t dma_channel;
    bool dma_started;
    struct adc_sample_t *buffer;
    unsigned int buffer_nsamps;

    // Callbacks
    void (*overflow_cb)();
    void (*buffer_full_cb)();
};

extern struct adc_t adc1, adc2;

void adc_init();

void adc_set_sample_times(struct adc_t *adc,
                          enum sample_time_t sample_time);

void adc_set_regular_sequence(struct adc_t *adc,
                              unsigned int num_samples,
                              adc_channel_t channels[]);

void adc_set_timer_freq(unsigned int freq);

enum adc_trigger_t {
  TRIGGER_MANUAL,
  TRIGGER_TIMER,
  TRIGGER_TIM4_CC4,
  TRIGGER_CONTINUOUS,
};

int adc_dma_start(struct adc_t *adc,
                  unsigned int nsamples, struct adc_sample_t *buf,
                  enum adc_trigger_t trigger);

void adc_trigger(struct adc_t *adc);

void adc_dma_stop(struct adc_t *adc);

struct adc_sample_t adc_get_last_sample();

