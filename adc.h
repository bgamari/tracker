#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

struct adc_t {
    uint32_t adc;

    // DMA state
    uint32_t dma;
    uint16_t dma_stream;
    uint32_t dma_channel;
    bool dma_started;
    uint16_t *buffer, *buffer2;
    unsigned int buffer_nsamps;
    unsigned int nchannels;

    // Callbacks
    void (*overflow_cb)(struct adc_t *adc);
    void (*buffer_full_cb)(struct adc_t *adc);
};

extern struct adc_t adc1, adc2;

void adc_init();

void adc_config_channels(struct adc_t *adc, unsigned int nchans, u8 *channels);

void adc_set_timer_freq(unsigned int freq);

enum adc_trigger_t {
  TRIGGER_MANUAL,
  TRIGGER_TIM3_CC1,
  TRIGGER_TIM4_CC4,
  TRIGGER_CONTINUOUS,
};

int adc_dma_start(struct adc_t *adc,
                  unsigned int nsamples, uint16_t *buf, uint16_t *buf2,
                  enum adc_trigger_t trigger);

void adc_trigger(struct adc_t *adc);

void adc_dma_stop(struct adc_t *adc);

uint16_t *adc_get_last_sample();

uint16_t *adc_get_active_buffer(struct adc_t *adc);
uint16_t *adc_get_idle_buffer(struct adc_t *adc);
