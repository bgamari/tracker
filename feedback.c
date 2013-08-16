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

static void feedback_update()
{
        set_dac(STAGE_OUTPUTS, updates);
}        

int feedback_set_position(uint16_t setpoint[3])
{
        if (feedback_mode != NO_FEEDBACK)
                return 1;
        for (unsigned int i=0; i<3; i++)
                updates[i].value = setpoint[i];
        feedback_update();
        return 0;
}

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
                timer_enable_counter(TIMER2);
                break;
        }
        feedback_mode = mode;
}
    
void do_feedback()
{
        signed int error[3];

        if (feedback_mode == NO_FEEDBACK) {
                return;
        } else if (feedback_mode == PSD_FEEDBACK) {
                int16_t *sample = adc_get_last_frame();
                for (int i=0; i<STAGE_OUTPUTS; i++) {
                        unsigned int tmp = 0;
                        for (unsigned int j=0; j<PSD_INPUTS; j++) 
                                tmp += psd_fb_gains[j][i] * sample[j] / 0x1000;
                        error[i] = psd_fb_setpoint[i] - tmp;
                }

        } else if (feedback_mode == STAGE_FEEDBACK) {
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
        feedback_update();
}

void timer2_isr(void)
{
        TIMER2_IR |= TIMER_IR_MR0INT;
        do_feedback();
}

