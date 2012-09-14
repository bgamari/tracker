#include "tracker.h"
#include "feedback.h"
#include "dac.h"

#define N_INPUTS 4
#define N_OUTPUTS 4
#define BUFFER_DEPTH 1000

struct adc_sample_t {
  uint16_t channel[N_INPUTS];
};

static struct adc_sample_t sample_buffer[BUFFER_DEPTH] __attribute__((section (".dma_data"))) = { 0 };
static volatile unsigned int buffer_start_time = 0;

struct dac_update_t updates[] = {
    { channel_a, 0x4400 },
    { channel_b, 0x4400 },
    { channel_c, 0x4400 },
    { channel_d, 0x4400 },
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

typedef unsigned int adc_channel_t;

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
    Pin_Init(ARM_PA0, 1, Analog); // ADC123_IN0
    Pin_Init(ARM_PA1, 1, Analog); // ADC123_IN1
    Pin_Init(ARM_PA2, 1, Analog); // ADC123_IN2
    Pin_Init(ARM_PA3, 1, Analog); // ADC123_IN3
    Pin_Init(ARM_PB0, 1, Analog); // ADC12_IN8
    Pin_Init(ARM_PB1, 1, Analog); // ADC12_IN9
    Pin_Init(ARM_PC4, 1, Analog); // ADC12_IN14
    Pin_Init(ARM_PC5, 1, Analog); // ADC12_IN15

    NVIC_EnableIRQ(ADC_IRQn);
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Initialize ADC
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR1 |= ADC_CR1_JEOCIE | ADC_CR1_SCAN | ADC_CR1_OVRIE;
    set_sample_times(SAMPLE_TIME_84_CYCLES);
    adc_channel_t channels[] = { 0, 1, 2, 3 };
    set_injected_sequence(ADC1, 4, channels);
    set_regular_sequence(ADC1, 4, channels);

    DMA2_Stream4->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream4->M0AR = (uint32_t) sample_buffer;
    DMA2_Stream4->NDTR = BUFFER_DEPTH * N_INPUTS;
    DMA2_Stream4->CR = DMA_SxCR_MINC | DMA_SxCR_CIRC;
    DMA2_Stream4->CR |= DMA_SxCR_TCIE;
    DMA2_Stream4->CR |= 1 << 11; // PSIZE=16 bits
    DMA2->HIFCR = 0xffffffff;
    DMA2_Stream4->CR |= DMA_SxCR_EN;

    ADC1->CR2 |= ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_SWSTART | ADC_CR2_CONT;
}

void DMA2_Stream4_IRQHandler() {
    unsigned int length = msTicks - buffer_start_time;
    buffer_start_time = msTicks;
    DMA2->HIFCR = 0xff;
    char msg[] = "done\n";
    Serial_Put_Bytes(ser1, NONBLOCKING, msg, sizeof(msg));
}

void ADC_IRQHandler() {
    if (ADC1->SR & ADC_SR_OVR) {
        char *msg = "adc-overrun\n";
        ADC1->SR &= ~ADC_SR_OVR;
        Serial_Put_Bytes(ser1, NONBLOCKING, msg, sizeof(msg));
    }
}

void feedback_init()
{
    NVIC_EnableIRQ(TIM2_IRQn);
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->ARR = 1000;
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void do_feedback()
{
    unsigned int n = BUFFER_DEPTH - DMA2_Stream4->NDTR / N_INPUTS - 1;
    struct adc_sample_t sample = sample_buffer[n];
    for (int i=0; i<4; i++)
        updates[i].value = sample.channel[i] << 4;
    set_dac(4, updates);
}

void TIM2_IRQHandler()
{
    TIM2->SR &= ~TIM_SR_UIF;
    do_feedback();
}
