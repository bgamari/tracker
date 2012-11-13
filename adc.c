#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/rcc.h>

#include <stdlib.h>
#include <stdbool.h>

#include "adc.h"
#include "timer.h"

struct adc_t adc1, adc2;

void init_adc1() {
    adc1.adc = ADC1;
    adc1.dma = DMA2;
    adc1.dma_stream = DMA_STREAM4;
    adc1.dma_channel = DMA_SCR_CHSEL_0;
    adc1.dma_started = false;
    adc1.nchannels = 0;
    adc1.buffer = NULL;
    adc1.buffer_nsamps = 0;
    adc1.overflow_cb = NULL;
    adc1.buffer_full_cb = NULL;
    adc1.trigger_src = ADC_TRIGGER_TIM3_CC1;
};

void init_adc2() {
    adc2.adc = ADC2;
    adc2.dma = DMA2;
    adc2.dma_stream = DMA_STREAM3;
    adc2.dma_channel = DMA_SCR_CHSEL_1;
    adc2.dma_started = false;
    adc2.nchannels = 0;
    adc2.buffer = NULL;
    adc2.buffer_nsamps = 0;
    adc2.overflow_cb = NULL;
    adc2.buffer_full_cb = NULL;
    adc2.trigger_src = ADC_TRIGGER_TIM4_CC4;
};

void adc_init()
{
    init_adc1();
    init_adc2();

    nvic_enable_irq(NVIC_ADC_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM4_IRQ);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);

    // Initialize timers for triggering
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM3EN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);

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

static uint32_t get_trigger_src_timer(enum adc_trigger_src_t src) {
    switch ((src >> 4) & 0xf) {
    case 1: return TIM1;
    case 2: return TIM2;
    case 3: return TIM3;
    case 4: return TIM4;
    case 5: return TIM5;
    case 8: return TIM8;
    default: return 0;
    }
}

static uint32_t get_trigger_src_cc(enum adc_trigger_src_t src) {
    switch (src & 0xf) {
    case 1: return TIM_OC1;
    case 2: return TIM_OC2;
    case 3: return TIM_OC3;
    case 4: return TIM_OC4;
    default: return 0xffffffff;
    }
}

static void adc_setup_trigger(struct adc_t *adc)
{
    uint32_t tmp;
    switch (adc->trigger_src) {
    case ADC_TRIGGER_MANUAL: return;
    case ADC_TRIGGER_CONTINUOUS:
        adc_set_continuous_conversion_mode(adc->adc);
        return;
    case ADC_TRIGGER_TIM1_CC1: tmp = ADC_CR2_EXTSEL_TIM1_CC1; break;
    case ADC_TRIGGER_TIM1_CC2: tmp = ADC_CR2_EXTSEL_TIM1_CC2; break;
    case ADC_TRIGGER_TIM1_CC3: tmp = ADC_CR2_EXTSEL_TIM1_CC3; break;
    case ADC_TRIGGER_TIM2_CC2: tmp = ADC_CR2_EXTSEL_TIM2_CC2; break;
    case ADC_TRIGGER_TIM2_CC3: tmp = ADC_CR2_EXTSEL_TIM2_CC3; break;
    case ADC_TRIGGER_TIM2_CC4: tmp = ADC_CR2_EXTSEL_TIM2_CC4; break;
    case ADC_TRIGGER_TIM3_CC1: tmp = ADC_CR2_EXTSEL_TIM3_CC1; break;
    case ADC_TRIGGER_TIM4_CC4: tmp = ADC_CR2_EXTSEL_TIM4_CC4; break;
    case ADC_TRIGGER_TIM5_CC1: tmp = ADC_CR2_EXTSEL_TIM5_CC1; break;
    case ADC_TRIGGER_TIM5_CC2: tmp = ADC_CR2_EXTSEL_TIM5_CC2; break;
    case ADC_TRIGGER_TIM5_CC3: tmp = ADC_CR2_EXTSEL_TIM5_CC3; break;
    case ADC_TRIGGER_TIM8_CC1: tmp = ADC_CR2_EXTSEL_TIM8_CC1; break;
    default: return;
    }
    adc_enable_external_trigger_regular(adc->adc, tmp, ADC_CR2_EXTEN_RISING_EDGE);
}

int adc_set_trigger_freq(struct adc_t *adc, uint32_t freq)
{
    uint32_t timer = get_trigger_src_timer(adc->trigger_src);
    tim_oc_id cc = (tim_oc_id) get_trigger_src_cc(adc->trigger_src);
    if (timer == 0 || cc == 0xffffffff) return 1;
    bool running = TIM_CR1(timer) & TIM_CR1_CEN;
        
    setup_periodic_timer(timer, freq);
    timer_set_oc_mode(timer, cc, TIM_OCM_TOGGLE);
    timer_set_oc_value(timer, cc, TIM_ARR(timer));
    timer_enable_oc_output(timer, cc);
    if (running)
        timer_enable_counter(timer);
    return 0;
}

int adc_trigger_start(struct adc_t *adc)
{
    uint32_t timer = get_trigger_src_timer(adc->trigger_src);
    if (timer == 0) return 1;
    timer_enable_counter(timer);
    return 0;
}

int adc_trigger_stop(struct adc_t *adc)
{
    uint32_t timer = get_trigger_src_timer(adc->trigger_src);
    if (timer == 0) return 1;
    timer_disable_counter(timer);
    return 0;
}

void adc_manual_trigger(struct adc_t *adc)
{
    adc_start_conversion_regular(adc->adc);
}

/* Note: buffer can't reside in core-coupled memory */
int adc_dma_start(struct adc_t *adc,
                  unsigned int nsamples, uint16_t *buf, uint16_t *buf2)
{
    if (adc->dma_started)
        return -1;
    if (adc->nchannels == 0)
        return -2;

    adc->buffer = buf;
    adc->buffer2 = buf2;
    adc->buffer_nsamps = nsamples;

    adc_setup_trigger(adc);

    u32 dma = adc->dma;
    u32 stream = adc->dma_stream;
    dma_stream_reset(dma, stream);
    dma_set_peripheral_address(dma, stream, (uint32_t) &ADC_DR(adc->adc));
    dma_set_memory_address(dma, stream, (uint32_t) buf);
    if (buf2) {
        dma_set_memory_address_1(dma, stream, (uint32_t) buf2);
        dma_enable_double_buffer_mode(dma, stream);
    }
    dma_set_number_of_data(dma, stream, nsamples * adc->nchannels);
    dma_enable_fifo_mode(dma, stream);
    dma_enable_memory_increment_mode(dma, stream);
    dma_enable_circular_mode(dma, stream);
    dma_channel_select(dma, stream, adc->dma_channel);
    dma_set_peripheral_size(dma, stream, DMA_SCR_PSIZE_16BIT);
    dma_set_memory_size(dma, stream, DMA_SCR_MSIZE_16BIT);

    dma_enable_fifo_error_interrupt(dma, stream);
    dma_enable_transfer_error_interrupt(dma, stream);
    dma_enable_transfer_complete_interrupt(dma, stream);
    dma_clear_interrupt_flags(dma, stream, 0xffffffff);
    dma_enable_stream(dma, stream);

    adc_set_dma_continue(adc->adc);
    adc_enable_dma(adc->adc);
    adc->dma_started = true;
    return 0;
}

void adc_dma_stop(struct adc_t *adc)
{
    adc_set_single_conversion_mode(adc->adc);
    dma_disable_stream(adc->dma, adc->dma_stream);
    adc->dma_started = false;
}

void dma2_stream4_isr() {
    if (dma_get_interrupt_flag(DMA2, 4, DMA_ISR_TCIF)) {
        dma_clear_interrupt_flags(DMA2, 4, DMA_ISR_TCIF);
        if (adc1.buffer_full_cb)
            adc1.buffer_full_cb(&adc1);
    }
    if (dma_get_interrupt_flag(DMA2, 4, DMA_ISR_TEIF)) { // Transfer error
        dma_clear_interrupt_flags(DMA2, 4, DMA_ISR_TEIF);
        adc1.dma_started = false;
    }
    if (dma_get_interrupt_flag(DMA2, 4, DMA_ISR_FEIF)) { // FIFO error
        dma_clear_interrupt_flags(DMA2, 4, DMA_ISR_TEIF);
        adc1.dma_started = false;
    }
}

void adc_isr() {
    if (adc_get_overrun_flag(ADC1)) {
        adc_clear_overrun_flag(ADC1);
        if (adc1.overflow_cb)
            adc1.overflow_cb(&adc1);
    }
    if (adc_get_overrun_flag(ADC2)) {
        adc_clear_overrun_flag(ADC2);
        if (adc2.overflow_cb)
            adc2.overflow_cb(&adc2);
    }
}

uint16_t *adc_get_active_buffer(struct adc_t *adc)
{
    if (adc->buffer2 == NULL)
        return adc->buffer;
    if (dma_get_target(adc->dma, adc->dma_stream))
        return adc->buffer2;
    else
        return adc->buffer;
}

uint16_t *adc_get_idle_buffer(struct adc_t *adc)
{
    if (adc->buffer2 == NULL)
        return NULL;
    if (dma_get_target(adc->dma, adc->dma_stream))
        return adc->buffer;
    else
        return adc->buffer2;
}

uint16_t *adc_get_last_sample(struct adc_t *adc)
{
    uint16_t *buffer = adc_get_active_buffer(adc);
    unsigned int ndtr = DMA_SNDTR(adc->dma, adc->dma_stream);
    unsigned int n = 2*adc->buffer_nsamps - ndtr / adc->nchannels - 2;
    n %= adc->buffer_nsamps;
    return &buffer[n * adc->nchannels];
}

