#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>

#include <stdbool.h>
#include <unistd.h>

#include <array>

#include "tracker.h"
#include "feedback.h"
#include "adc.h"
#include "dac.h"
#include "timer.h"
#include "clock.h"

#define BUFFER_DEPTH 1024

static uint16_t psd_buffer[BUFFER_DEPTH][PSD_INPUTS] __attribute__((section (".dma_data"))) = { };
static uint16_t stage_buffer[BUFFER_DEPTH][STAGE_INPUTS] __attribute__((section (".dma_data"))) = { };

static volatile unsigned int buffer_start_time = 0;

//static uint16_t pos_buffer[BUFFER_DEPTH][STAGE_INPUTS] __attribute__((section (".dma_data"))) = { };

static bool feedback_running = false;

enum feedback_mode_t feedback_mode = STAGE_FEEDBACK;

signed int psd_fb_gains[PSD_INPUTS][STAGE_OUTPUTS] = { };
signed int psd_fb_setpoint[STAGE_OUTPUTS] = { };

signed int stage_fb_gains[STAGE_INPUTS][STAGE_OUTPUTS] = { };
signed int stage_fb_setpoint[STAGE_OUTPUTS] = { };

signed int output_gains[STAGE_OUTPUTS] = { };

struct dac_update_t updates[] = {
    { channel_a, 0x4400 },
    { channel_b, 0x4400 },
    { channel_c, 0x4400 },
};

void adc_buffer_full(struct adc_t *adc)
{
    //unsigned int length = msTicks - buffer_start_time;
    buffer_start_time = msTicks;
}

void adc_overflow(struct adc_t *adc)
{
    //char *msg = "adc-overrun\n";
}

void feedback_set_loop_freq(unsigned int freq)
{
    setup_periodic_timer(TIM2, freq);
    timer_enable_irq(TIM2, TIM_DIER_UIE);
    if (feedback_running)
        timer_enable_counter(TIM2);
}

void feedback_init()
{
    nvic_set_priority(NVIC_TIM2_IRQ, 10);
    nvic_enable_irq(NVIC_TIM2_IRQ);

    feedback_set_loop_freq(10000);

    // ADC123_IN0, ADC123_IN1, ADC123_IN2, ADC123_IN3
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1 | GPIO2 | GPIO3);
    // ADC12_IN8, ADC12_IN9
    gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);
    // ADC123_IN12, ADC123_IN13, ADC12_IN14, ADC12_IN15
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2 | GPIO3 | GPIO4 | GPIO5);
}

void feedback_start()
{
    if (feedback_running) return;
    psd_adc->buffer_full_cb = adc_buffer_full;
    psd_adc->overflow_cb = adc_overflow;
    adc_dma_start(psd_adc, BUFFER_DEPTH, &psd_buffer[0][0], NULL);

    stage_adc->buffer_full_cb = adc_buffer_full;
    stage_adc->overflow_cb = adc_overflow;
    adc_dma_start(stage_adc, BUFFER_DEPTH, &stage_buffer[0][0], NULL);

    timer_enable_counter(TIM2);
    timer_enable_counter(TIM3);
    feedback_running = true;
}

void feedback_stop()
{
    if (!feedback_running) return;
    timer_disable_counter(TIM2);
    timer_disable_counter(TIM3);
    adc_dma_stop(&adc1);
    feedback_running = false;
}
    
void do_feedback()
{
    std::array<signed int, 3> error;

    if (feedback_mode == PSD_FEEDBACK) {
        uint16_t *sample = adc_get_last_sample(psd_adc);
        for (int i=0; i<STAGE_OUTPUTS; i++) {
            unsigned int tmp = 0;
            for (unsigned int j=0; j<PSD_INPUTS; j++) 
                tmp += psd_fb_gains[j][i] * sample[j] / 0x1000;
            error[i] = psd_fb_setpoint[i] - tmp;
        }

    } else {
        uint16_t *sample = adc_get_last_sample(stage_adc);
        for (int i=0; i<STAGE_OUTPUTS; i++) {
            signed int tmp = 0;
            for (unsigned int j=0; j<STAGE_INPUTS; j++) 
                tmp += stage_fb_gains[j][i] * sample[j] / 0x1000;
            error[i] = stage_fb_setpoint[i] - tmp;
        }
    }

    // TODO: Put error into PID loop
    for (int i=0; i<STAGE_OUTPUTS; i++)
        updates[i].value += output_gains[i] * error[i] / 0x1000;
    set_dac(STAGE_OUTPUTS, updates);
}

void tim2_isr(void)
{
    TIM2_SR &= ~TIM_SR_UIF;
    do_feedback();
}

