#include <stdbool.h>
#include "adc.h"

extern uint32_t SlowPeripheralClock;

bool adc_dma_started;

void (*adc_overflow_cb)();
void (*adc_buffer_full_cb)();

static struct adc_sample_t *buffer;
static unsigned int buffer_nsamps;

void set_sample_times(enum sample_time_t sample_time)
{
    unsigned int tmp = 0;
    for (int i = 0; i < 8; i++) {
        tmp |= sample_time;
        tmp <<= 3;
    }
    ADC1->SMPR1 = tmp;
    ADC1->SMPR2 = tmp;
}

void adc_set_regular_sequence(ADC_TypeDef *adc,
                              unsigned int num_samples,
                              adc_channel_t channels[])
{
    if (num_samples == 0)
        return;
    if (num_samples > 16)
        num_samples = 16;

    __IO uint32_t *reg = &adc->SQR3;
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
    adc->SQR1 |= (num_samples-1) << 20;
}

void set_adc_timer_freq(unsigned int freq)
{
    unsigned int prescaler = 1;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    while (SlowPeripheralClock / prescaler / freq > 65536)
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

    // Initialize ADC
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR1 |= ADC_CR1_JEOCIE | ADC_CR1_SCAN | ADC_CR1_OVRIE;
}

/* Note: buffer can't reside in core-coupled memory */
int adc_dma_start(unsigned int nsamples, struct adc_sample_t *buf,
                  enum adc_trigger_t trigger)
{
    if (adc_dma_started)
        return -1;

    buffer = buf;
    buffer_nsamps = nsamples;

    DMA2_Stream4->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream4->M0AR = (uint32_t) buffer;
    DMA2_Stream4->NDTR = BUFFER_DEPTH * N_INPUTS;
    DMA2_Stream4->CR = DMA_SxCR_MINC | DMA_SxCR_CIRC;
    DMA2_Stream4->CR |= DMA_SxCR_TCIE;
    DMA2_Stream4->CR |= 1 << 11; // PSIZE=16 bits
    DMA2->HIFCR = 0xffffffff;
    DMA2_Stream4->CR |= DMA_SxCR_EN;

    if (trigger == TRIGGER_CONTINUOUS)
        ADC1->CR2 |= ADC_CR2_CONT;
    else if (trigger == TRIGGER_TIMER) {
        ADC1->CR2 |= 0x7 << 24;
        TIM3->CR1 |= TIM_CR1_CEN;
    }
    ADC1->CR2 |= ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_SWSTART;
    return 0;
}

void adc_trigger()
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void adc_dma_stop()
{
    ADC1->CR2 &= ~ADC_CR2_CONT;
    adc_dma_started = false;
}

void DMA2_Stream4_IRQHandler() {
    uint32_t flag = DMA2->HISR;
    DMA2->HIFCR = 0xff;
    if (flag & DMA_HISR_TCIF4 && adc_buffer_full_cb)
        adc_buffer_full_cb();
    if (flag & DMA_HISR_TEIF4) // error
        adc_dma_started = false;
}

void ADC_IRQHandler() {
    if (ADC1->SR & ADC_SR_OVR) {
        ADC1->SR &= ~ADC_SR_OVR;
        if (adc_overflow_cb)
            adc_overflow_cb();
    }
}

struct adc_sample_t *adc_get_last_sample()
{
    unsigned int n = 2*buffer_nsamps - DMA2_Stream4->NDTR / N_INPUTS - 2;
    n %= BUFFER_DEPTH;
    return &buffer[n];
}

