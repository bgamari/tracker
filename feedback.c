#include "tracker.h"
#include "feedback.h"
#include "adc.h"
#include "dac.h"

static struct adc_sample_t sample_buffer[BUFFER_DEPTH] __attribute__((section (".dma_data"))) = { 0 };

static volatile unsigned int buffer_start_time = 0;

static struct adc_sample_t pos_buffer[BUFFER_DEPTH] __attribute__((section (".dma_data"))) = { 0 };

static signed int feedback_gains[N_INPUTS][N_OUTPUTS];

struct dac_update_t updates[] = {
    { channel_a, 0x4400 },
    { channel_b, 0x4400 },
    { channel_c, 0x4400 },
    { channel_d, 0x4400 },
};

void adc_buffer_full()
{
    unsigned int length = msTicks - buffer_start_time;
    buffer_start_time = msTicks;
}

void adc_overflow()
{
    char *msg = "adc-overrun\n";
    Serial_Put_Bytes(ser1, NONBLOCKING, msg, sizeof(msg));
}

void feedback_init()
{
    NVIC_EnableIRQ(TIM2_IRQn);
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->ARR = 100;
    TIM2->CR1 = TIM_CR1_ARPE;
    TIM2->CR1 |= TIM_CR1_CEN;

    Pin_Init(ARM_PA0, 1, Analog); // ADC123_IN0
    Pin_Init(ARM_PA1, 1, Analog); // ADC123_IN1
    Pin_Init(ARM_PA2, 1, Analog); // ADC123_IN2
    Pin_Init(ARM_PA3, 1, Analog); // ADC123_IN3
    Pin_Init(ARM_PB0, 1, Analog); // ADC12_IN8
    Pin_Init(ARM_PB1, 1, Analog); // ADC12_IN9
    Pin_Init(ARM_PC2, 1, Analog); // ADC123_IN12
    Pin_Init(ARM_PC3, 1, Analog); // ADC123_IN13
    Pin_Init(ARM_PC4, 1, Analog); // ADC12_IN14
    Pin_Init(ARM_PC5, 1, Analog); // ADC12_IN15

    adc_channel_t channels[] = { 0, 1, 2, 12 };
    adc_set_sample_times(ADC1, SAMPLE_TIME_84_CYCLES);
    adc_set_regular_sequence(ADC1, 4, channels);
    adc_init();
    adc_buffer_full_cb = adc_buffer_full;
    adc_overflow_cb = adc_overflow;

    feedback_gains[0][0] = 0.5 * 0xffff;
    feedback_gains[1][1] = 0.2 * 0xffff;
    feedback_gains[2][2] = 0.8 * 0xffff;
}

void do_feedback()
{
    struct adc_sample_t sample = *adc_get_last_sample();
    for (int i=0; i<N_OUTPUTS; i++) {
        unsigned int tmp = 0;
        for (unsigned int j=0; j<N_INPUTS; j++) 
            tmp += feedback_gains[j][i] * sample.channel[i] / 0x1000;
        updates[i].value = tmp;
    }
    set_dac(4, updates);
}

void TIM2_IRQHandler()
{
    TIM2->SR &= ~TIM_SR_UIF;
    do_feedback();
}

