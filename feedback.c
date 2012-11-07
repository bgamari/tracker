#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <stdbool.h>

#include "tracker.h"
#include "feedback.h"
#include "adc.h"
#include "dac.h"
#include "clock.h"

#define BUFFER_DEPTH 1000

static uint16_t sample_buffer[BUFFER_DEPTH][STAGE_INPUTS] __attribute__((section (".dma_data"))) = { };

static volatile unsigned int buffer_start_time = 0;

//static uint16_t pos_buffer[BUFFER_DEPTH][STAGE_INPUTS] __attribute__((section (".dma_data"))) = { };

static bool feedback_running = false;

signed int feedback_gains[STAGE_INPUTS][STAGE_OUTPUTS] = { };
signed int feedback_setpoint[STAGE_OUTPUTS] = { };

struct dac_update_t updates[] = {
    { channel_a, 0x4400 },
    { channel_b, 0x4400 },
    { channel_c, 0x4400 },
};

void adc_buffer_full()
{
    //unsigned int length = msTicks - buffer_start_time;
    buffer_start_time = msTicks;
}

void adc_overflow()
{
    //char *msg = "adc-overrun\n";
}

void set_adc_trigger_freq(unsigned int freq)
{
    unsigned int prescaler = 1;
    while (rcc_ppre1_frequency / prescaler / freq > 0xffff)
        prescaler *= 2;
    timer_reset(TIM3);
    timer_set_prescaler(TIM3, prescaler);
    timer_set_period(TIM3, rcc_ppre1_frequency / prescaler / freq);
    timer_enable_preload(TIM3);
    timer_enable_oc_output(TIM3, TIM_OC1);
}

void feedback_init()
{
    nvic_enable_irq(NVIC_TIM2_IRQ);
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
    timer_reset(TIM2);
    timer_enable_irq(TIM2, TIM_DIER_UIE);
    timer_set_period(TIM2, 5000);
    timer_enable_preload(TIM2);

    RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
    set_adc_trigger_freq(1000);

    // ADC123_IN0, ADC123_IN1, ADC123_IN2, ADC123_IN3
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3);
    // ADC12_IN8, ADC12_IN9
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    // ADC123_IN12, ADC123_IN13, ADC12_IN14, ADC12_IN15
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2 | GPIO3 | GPIO4 | GPIO5);

    feedback_gains[0][0] = 0.5 * 0xffff;
    feedback_gains[1][1] = 0.2 * 0xffff;
    feedback_gains[2][2] = 0.8 * 0xffff;
}

void feedback_start()
{
    adc1.buffer_full_cb = adc_buffer_full;
    adc1.overflow_cb = adc_overflow;
    adc_dma_start(&adc1, BUFFER_DEPTH, &sample_buffer[0][0], TRIGGER_CONTINUOUS);
    timer_enable_counter(TIM2);
    timer_enable_counter(TIM3);
    feedback_running = true;
}

void feedback_stop()
{
    timer_disable_counter(TIM2);
    timer_disable_counter(TIM3);
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

void tim2_isr(void)
{
    TIM2_SR &= ~TIM_SR_UIF;
    do_feedback();
}

