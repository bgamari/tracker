#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

enum adc_trigger_src_t {
    ADC_TRIGGER_MANUAL = 0,
    ADC_TRIGGER_CONTINUOUS = 1,

    ADC_TRIGGER_TIM1_CC1=0x11, ADC_TRIGGER_TIM1_CC2=0x12, ADC_TRIGGER_TIM1_CC3=0x13,
    ADC_TRIGGER_TIM2_CC2=0x22, ADC_TRIGGER_TIM2_CC3=0x23, ADC_TRIGGER_TIM2_CC4=0x24,
    ADC_TRIGGER_TIM3_CC1=0x31,
    ADC_TRIGGER_TIM4_CC4=0x44,
    ADC_TRIGGER_TIM5_CC1=0x51, ADC_TRIGGER_TIM5_CC2=0x52, ADC_TRIGGER_TIM5_CC3=0x53, 
    ADC_TRIGGER_TIM8_CC1=0x81, 
};
    
struct adc_t {
    uint32_t adc;
    bool running;

    // DMA state
    uint32_t dma;
    uint16_t dma_stream;
    uint32_t dma_channel;
    bool dma_started;
    uint16_t *buffer, *buffer2;
    unsigned int buffer_nsamps;
    unsigned int nchannels;

    // Triggering
    enum adc_trigger_src_t trigger_src;

    // Callbacks
    void (*overflow_cb)(struct adc_t *adc);
    void (*buffer_full_cb)(struct adc_t *adc);
};

extern struct adc_t adc1, adc2;

// Subsystem initialization
void adc_init();

// ADC configuration
void adc_config_channels(struct adc_t *adc, unsigned int nchans, uint8_t *channels);

// Triggering
int adc_set_trigger_freq(struct adc_t *adc, uint32_t freq);
int adc_trigger_start(struct adc_t *adc);
int adc_trigger_stop(struct adc_t *adc);
void adc_manual_trigger(struct adc_t *adc);

// DMA
int adc_dma_start(struct adc_t *adc,
                  unsigned int nsamples, uint16_t *buf, uint16_t *buf2);
void adc_dma_stop(struct adc_t *adc);

// Accessing sample buffers
uint16_t *adc_get_last_sample(struct adc_t *adc);

uint16_t *adc_get_active_buffer(struct adc_t *adc);
uint16_t *adc_get_idle_buffer(struct adc_t *adc);
