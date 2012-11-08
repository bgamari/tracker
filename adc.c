#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <stdlib.h>
#include <stdbool.h>
#include "adc.h"

struct adc_t adc1 = {
    .adc = ADC1,
    .dma = DMA2,
    .dma_stream = DMA_STREAM4,
    .dma_channel = DMA_SCR_CHSEL_0,
    .dma_started = false,
    .nchannels = 0,
    .buffer = NULL,
    .buffer_nsamps = 0,
    .overflow_cb = NULL,
    .buffer_full_cb = NULL,
};

struct adc_t adc2 = {
    .adc = ADC2,
    .dma = DMA2,
    .dma_stream = DMA_STREAM3,
    .dma_channel = DMA_SCR_CHSEL_1,
    .dma_started = false,
    .nchannels = 0,
    .buffer = NULL,
    .buffer_nsamps = 0,
    .overflow_cb = NULL,
    .buffer_full_cb = NULL,
};

void adc_init()
{
    nvic_enable_irq(NVIC_ADC_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM4_IRQ);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

    // Initialize ADCs
    adc_power_on(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_enable_overrun_interrupt(ADC1);

    adc_power_on(ADC2);
    adc_enable_scan_mode(ADC2);
    adc_enable_overrun_interrupt(ADC2);
}

void adc_config_channels(struct adc_t *adc, unsigned int nchans, u8 *channels)
{
    adc_set_regular_sequence(adc->adc, nchans, channels);
    adc->nchannels = nchans;
}

/* Note: buffer can't reside in core-coupled memory */
int adc_dma_start(struct adc_t *adc,
                  unsigned int nsamples, uint16_t *buf,
                  enum adc_trigger_t trigger)
{
    if (adc->dma_started)
        return -1;
    if (adc->nchannels == 0)
        return -2;

    adc->buffer = buf;
    adc->buffer_nsamps = nsamples;

    u32 dma = adc->dma;
    u32 stream = adc->dma_stream;
    dma_stream_reset(dma, stream);
    dma_set_peripheral_address(dma, stream, (uint32_t) &ADC_DR(adc->adc));
    dma_set_memory_address(dma, stream, (uint32_t) buf);
    dma_set_number_of_data(dma, stream, nsamples * adc->nchannels);

    dma_enable_memory_increment_mode(dma, stream);
    dma_enable_circular_mode(dma, stream);
    dma_channel_select(dma, stream, adc->dma_channel);
    dma_enable_transfer_complete_interrupt(dma, stream);
    dma_set_peripheral_size(dma, stream, DMA_SCR_PSIZE_16BIT);

    dma_clear_interrupt_flags(dma, stream, 0xffffffff);
    dma_enable_stream(dma, stream);

    if (trigger == TRIGGER_CONTINUOUS)
        adc_set_continuous_conversion_mode(adc->adc);
    else if (trigger == TRIGGER_TIM3_CC1) {
        adc_enable_external_trigger_regular(adc->adc,
                                            ADC_CR2_EXTSEL_TIM3_CC1,
                                            ADC_CR2_EXTEN_RISING_EDGE);
    } else if (trigger == TRIGGER_TIM4_CC4) {
        adc_enable_external_trigger_regular(adc->adc,
                                            ADC_CR2_EXTSEL_TIM4_CC4,
                                            ADC_CR2_EXTEN_RISING_EDGE);
    }
      
    adc_set_dma_continue(adc->adc);
    adc_enable_dma(adc->adc);
    adc_trigger(adc);
    return 0;
}

void adc_trigger(struct adc_t *adc)
{
    adc_start_conversion_regular(adc->adc);
}

void adc_dma_stop(struct adc_t *adc)
{
    adc_set_single_conversion_mode(adc->adc);
    adc->dma_started = false;
}

void dma2_stream4_isr() {
    if (dma_get_interrupt_flag(DMA2, 4, DMA_ISR_TCIF) && adc1.buffer_full_cb) {
        dma_clear_interrupt_flags(DMA2, 4, DMA_ISR_TCIF);
        adc1.buffer_full_cb();
    }
    if (dma_get_interrupt_flag(DMA2, 4, DMA_ISR_TEIF)) { // error
        dma_clear_interrupt_flags(DMA2, 4, DMA_ISR_TEIF);
        adc1.dma_started = false;
    }
}

void adc_isr() {
    if (adc_get_overrun_flag(ADC1)) {
        adc_clear_overrun_flag(ADC1);
        if (adc1.overflow_cb)
            adc1.overflow_cb();
    }
}

uint16_t *adc_get_last_sample(struct adc_t *adc)
{
    unsigned int ndtr = DMA_SNDTR(adc->dma, adc->dma_stream);
    unsigned int n = 2*adc->buffer_nsamps - ndtr / adc->nchannels - 2;
    n %= adc->buffer_nsamps;
    return &adc->buffer[n * adc->nchannels];
}

