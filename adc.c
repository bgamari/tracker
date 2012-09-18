#include "adc.h"

void (*adc_overflow_cb)();
void (*adc_buffer_full_cb)();

static struct adc_sample_t *buffer;

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

void set_injected_sequence(ADC_TypeDef *adc,
                           unsigned int num_samples,
                           adc_channel_t channels[])
{
    if (num_samples == 0)
        return;
    if (num_samples > 4)
        num_samples = 4;
    adc->JSQR = num_samples-1;
    for (unsigned int i=0; i<4; i++) {
        adc->JSQR <<= 5;
        if (i < num_samples)
            adc->JSQR |= 0xf & channels[i];
    }
}

void set_regular_sequence(ADC_TypeDef *adc,
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
void adc_dma_start(unsigned int nsamples, struct adc_sample_t *buf)
{
    buffer = buf;
    DMA2_Stream4->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream4->M0AR = (uint32_t) buffer;
    DMA2_Stream4->NDTR = BUFFER_DEPTH * N_INPUTS;
    DMA2_Stream4->CR = DMA_SxCR_MINC | DMA_SxCR_CIRC;
    DMA2_Stream4->CR |= DMA_SxCR_TCIE;
    DMA2_Stream4->CR |= 1 << 11; // PSIZE=16 bits
    DMA2->HIFCR = 0xffffffff;
    DMA2_Stream4->CR |= DMA_SxCR_EN;
    ADC1->CR2 |= ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_SWSTART;
}

void DMA2_Stream4_IRQHandler() {
    DMA2->HIFCR = 0xff;
    if (adc_buffer_full_cb)
        adc_buffer_full_cb();
}

void ADC_IRQHandler() {
    if (ADC1->SR & ADC_SR_OVR) {
        ADC1->SR &= ~ADC_SR_OVR;
        if (adc_overflow_cb)
            adc_overflow_cb();
    }
}

