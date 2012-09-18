#include <stdbool.h>
#include "adc.h"

struct adc_t adc1 = {
    .dma = DMA2,
    .dma_stream = DMA2_Stream4,
    .dma_channel = 0,
    .dma_started = false,
    .buffer = NULL,
    .buffer_nsamps = 0,
    .overflow_cb = NULL,
    .buffer_full_cb = NULL,
};

struct adc_t adc2 = {
    .dma = DMA2,
    .dma_stream = DMA2_Stream3,
    .dma_channel = 1,
    .dma_started = false,
    .buffer = NULL,
    .buffer_nsamps = 0,
    .overflow_cb = NULL,
    .buffer_full_cb = NULL,
};

void adc_set_sample_times(struct adc_t *adc,
                          enum sample_time_t sample_time)
{
    unsigned int tmp = 0;
    for (int i = 0; i < 8; i++) {
        tmp |= sample_time;
        tmp <<= 3;
    }
    adc->adc->SMPR1 = tmp;
    adc->adc->SMPR2 = tmp;
}

void adc_set_regular_sequence(struct adc_t *adc,
                              unsigned int num_samples,
                              adc_channel_t channels[])
{
    if (num_samples == 0)
        return;
    if (num_samples > 16)
        num_samples = 16;

    __IO uint32_t *reg = &adc->adc->SQR3;
    uint32_t k = 0;
    for (unsigned int i=0; i<3; i++) {
        uint32_t tmp = 0;
        for (unsigned int j=0; j<6; j++) {
            adc_channel_t chan = 0;
            if (k < num_samples)
                chan = channels[k] & 0x1f;
            tmp |= chan << (5*j);
            k++;
        }
        *reg = tmp;
        reg--;
    }
    adc->adc->SQR1 |= (num_samples-1) << 20;
}

void adc_set_timer_freq(unsigned int freq)
{
    unsigned int prescaler = 1;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    while (SlowPeripheralClock / prescaler / freq > 0xffff)
        prescaler *= 2;
    TIM3->PSC = prescaler - 1;
    TIM3->ARR = SlowPeripheralClock / prescaler / freq;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->CR2 = 0;
    TIM3->CCR1 = 0;
    TIM3->CCER = TIM_CCER_CC1E;
}

void adc_init()
{
    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Initialize ADCs
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_OVRIE;

    ADC2->CR2 |= ADC_CR2_ADON;
    ADC2->CR1 |= ADC_CR1_SCAN | ADC_CR1_OVRIE;
}

/* Note: buffer can't reside in core-coupled memory */
int adc_dma_start(struct adc_t *adc,
                  unsigned int nsamples, struct adc_sample_t *buf,
                  enum adc_trigger_t trigger)
{
    if (adc->dma_started)
        return -1;

    adc->buffer = buf;
    adc->buffer_nsamps = nsamples;

    adc->dma_stream->PAR = (uint32_t) &adc->adc->DR;
    adc->dma_stream->M0AR = (uint32_t) buf;
    adc->dma_stream->NDTR = nsamples * N_INPUTS;
    adc->dma_stream->CR = DMA_SxCR_MINC | DMA_SxCR_CIRC;
    adc->dma_stream->CR |= adc->dma_channel << 25;
    adc->dma_stream->CR |= DMA_SxCR_TCIE;
    adc->dma_stream->CR |= 1 << 11; // PSIZE=16 bits
    adc->dma->HIFCR = 0xffffffff;
    adc->dma_stream->CR |= DMA_SxCR_EN;

    if (trigger == TRIGGER_CONTINUOUS)
        adc->adc->CR2 |= ADC_CR2_CONT;
    else if (trigger == TRIGGER_TIMER) {
        adc->adc->CR2 |= 0x1 << 29; // EXTEN = Rising edge
        adc->adc->CR2 |= 0x7 << 24; // TIM3 CC1
        TIM3->CR1 |= TIM_CR1_CEN;
    } else if (trigger == TRIGGER_TIM4_CC4) {
        adc->adc->CR2 |= 0x1 << 29; // EXTEN = Rising edge
        adc->adc->CR2 |= 0x9 << 24; // TIM3 CC1
    }
      
    adc->adc->CR2 |= ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_SWSTART;
    return 0;
}

void adc_trigger(struct adc_t *adc)
{
    adc->adc->CR2 |= ADC_CR2_SWSTART;
}

void adc_dma_stop(struct adc_t *adc)
{
    adc->adc->CR2 &= ~ADC_CR2_CONT;
    adc->dma_started = false;
}

void DMA2_Stream4_IRQHandler() {
    uint32_t flag = DMA2->HISR;
    DMA2->HIFCR = 0xff;
    if (flag & DMA_HISR_TCIF4 && adc1.buffer_full_cb)
        adc1.buffer_full_cb();
    if (flag & DMA_HISR_TEIF4) // error
        adc1.dma_started = false;
}

void ADC_IRQHandler() {
    if (ADC1->SR & ADC_SR_OVR) {
        ADC1->SR &= ~ADC_SR_OVR;
        if (adc1.overflow_cb)
            adc1.overflow_cb();
    }
}

struct adc_sample_t adc_get_last_sample(struct adc_t *adc)
{
    unsigned int n = 2*adc->buffer_nsamps - adc->dma_stream->NDTR / N_INPUTS - 2;
    n %= BUFFER_DEPTH;
    return adc->buffer[n];
}

