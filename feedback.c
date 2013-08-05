#include <libopencm3/lpc43xx/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#include "tracker.h"
#include "feedback.h"
#include "adc.h"
#include "dac.h"
#include "timer.h"
#include "clock.h"

#define BUFFER_DEPTH 1024

static int16_t buffer[BUFFER_DEPTH][INPUTS] __attribute__((section (".dma_data"))) = { };

static volatile unsigned int buffer_start_time = 0;

//static int16_t pos_buffer[BUFFER_DEPTH][STAGE_INPUTS] __attribute__((section (".dma_data"))) = { };

static enum feedback_mode_t feedback_mode = NO_FEEDBACK;

signed int psd_fb_gains[PSD_INPUTS][STAGE_OUTPUTS] = { };
signed int psd_fb_setpoint[STAGE_OUTPUTS] = { };

signed int stage_fb_gains[STAGE_INPUTS][STAGE_OUTPUTS] = { };
signed int stage_fb_setpoint[STAGE_OUTPUTS] = { };

signed int max_error = 1000;
signed int output_gains[STAGE_OUTPUTS] = { };

struct dac_update_t updates[] = {
        { channel_a, 0x4400 },
        { channel_b, 0x4400 },
        { channel_c, 0x4400 },
};

void feedback_set_loop_freq(unsigned int freq)
{
        setup_periodic_timer(TIMER2, freq);
        TIMER2_IR |= TIMER_IR_MR0INT;
        if (feedback_mode != NO_FEEDBACK)
                timer_enable_counter(TIMER2);
}

void feedback_init()
{
        nvic_set_priority(NVIC_TIMER2_IRQ, 10);
        nvic_enable_irq(NVIC_TIMER2_IRQ);

        feedback_set_loop_freq(10000);
}

enum feedback_mode_t feedback_get_mode()
{
        return feedback_mode;
}

void feedback_set_mode(enum feedback_mode_t mode)
{
        if (mode == feedback_mode) return;

        switch (mode) {
        case NO_FEEDBACK:
                timer_disable_counter(TIMER2);
                break;

        case PSD_FEEDBACK:
        case STAGE_FEEDBACK:
                buffer[0][0] = buffer[1][1];
                timer_enable_counter(TIMER2);
        }
        feedback_mode = mode;
}
    
void do_feedback()
{
        signed int error[3];

        if (feedback_mode == PSD_FEEDBACK) {
                int16_t *sample = adc_get_last_frame();
                for (int i=0; i<STAGE_OUTPUTS; i++) {
                        unsigned int tmp = 0;
                        for (unsigned int j=0; j<PSD_INPUTS; j++) 
                                tmp += psd_fb_gains[j][i] * sample[j] / 0x1000;
                        error[i] = psd_fb_setpoint[i] - tmp;
                }

        } else {
                int16_t *sample = adc_get_last_frame();
                for (int i=0; i<STAGE_OUTPUTS; i++) {
                        signed int tmp = 0;
                        for (unsigned int j=0; j<STAGE_INPUTS; j++) 
                                tmp += stage_fb_gains[j][i] * sample[j] / 0x1000;
                        error[i] = stage_fb_setpoint[i] - tmp;
                }
        }

        for (int i=0; i<STAGE_OUTPUTS; i++) {
                if (abs(error[i]) > max_error)
                        feedback_set_mode(NO_FEEDBACK);
        }

        // TODO: Put error into PID loop
        for (int i=0; i<STAGE_OUTPUTS; i++)
                updates[i].value += output_gains[i] * error[i] / 0x1000;
        set_dac(STAGE_OUTPUTS, updates);
}

void timer2_isr(void)
{
        TIMER2_IR |= TIMER_IR_MR0INT;
        do_feedback();
}

