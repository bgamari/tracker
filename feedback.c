#include <stdbool.h>

#include "tracker.h"
#include "feedback.h"
#include "adc.h"
#include "dac.h"

#define BUFFER_DEPTH 1000
#define SENSOR_INPUTS 4
#define STAGE_INPUTS 3
#define STAGE_OUTPUTS 3

static uint16_t sample_buffer[BUFFER_DEPTH][STAGE_INPUTS] __attribute__((section (".dma_data"))) = { 0 };

static volatile unsigned int buffer_start_time = 0;

static uint16_t pos_buffer[BUFFER_DEPTH][STAGE_INPUTS] __attribute__((section (".dma_data"))) = { 0 };

static bool feedback_running = false;

static signed int feedback_gains[STAGE_INPUTS][STAGE_OUTPUTS] = { 0 };

struct dac_update_t updates[] = {
    { channel_a, 0x4400 },
    { channel_b, 0x4400 },
    { channel_c, 0x4400 },
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

void set_adc_trigger_freq(unsigned int freq)
{
    unsigned int prescaler = 1;
    while (SlowPeripheralClock / prescaler / freq > 0xffff)
        prescaler *= 2;
    TIM3->PSC = prescaler - 1;
    TIM3->ARR = SlowPeripheralClock / prescaler / freq;
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->CR2 = 0;
    TIM3->CCR1 = 0;
    TIM3->CCER = TIM_CCER_CC1E;
}

void feedback_init()
{
    NVIC_EnableIRQ(TIM2_IRQn);
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->ARR = 100;
    TIM2->CR1 = TIM_CR1_ARPE;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    set_adc_trigger_freq(1000);

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

    feedback_gains[0][0] = 0.5 * 0xffff;
    feedback_gains[1][1] = 0.2 * 0xffff;
    feedback_gains[2][2] = 0.8 * 0xffff;
}

void feedback_start()
{
    adc1.buffer_full_cb = adc_buffer_full;
    adc1.overflow_cb = adc_overflow;
    adc_dma_start(&adc1, BUFFER_DEPTH, &sample_buffer[0][0], TRIGGER_CONTINUOUS);
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM3->CR1 |= TIM_CR1_CEN;
    feedback_running = true;
}

void feedback_stop()
{
    TIM3->CR1 &= ~TIM_CR1_CEN;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    adc_dma_stop(&adc1);
    feedback_running = false;
}
    
void do_feedback()
{
    uint16_t *sample = adc_get_last_sample(&adc1);
    for (int i=0; i<STAGE_OUTPUTS; i++) {
        unsigned int tmp = 0;
        for (unsigned int j=0; j<STAGE_INPUTS; j++) 
            tmp += feedback_gains[j][i] * sample[j] / 0x1000;
        updates[i].value = tmp;
    }
    set_dac(STAGE_OUTPUTS, updates);
}

void TIM2_IRQHandler()
{
    TIM2->SR &= ~TIM_SR_UIF;
    do_feedback();
}

