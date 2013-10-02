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

fixed24_t psd_fb_gains[PSD_INPUTS][STAGE_OUTPUTS] = { };
signed int psd_fb_setpoint[PSD_INPUTS] = { };

fixed16_t stage_fb_gains[STAGE_INPUTS][STAGE_OUTPUTS] = { };
signed int stage_fb_setpoint[STAGE_INPUTS] = { };

signed int max_error = 1000;

struct pi_channel stage_outputs[STAGE_OUTPUTS];

struct excitation_buffer excitations[STAGE_OUTPUTS];

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
        switch (feedback_mode) {
        case NO_FEEDBACK:
                for (unsigned int i=0; i<STAGE_OUTPUTS; i++)
                        updates[i].value = setpoint[i];
                feedback_update();
                return 0;

        case STAGE_FEEDBACK:
                for (unsigned int i=0; i<STAGE_INPUTS; i++)
                        stage_fb_setpoint[i] = setpoint[i];
                return 0;

        default:
                return 1;
        }
}

void feedback_set_loop_freq(unsigned int freq)
{
        setup_periodic_timer(TIMER2, freq/2); // HACK: division by two
        // interrupt on reset (match 3)
        TIMER2_MCR |= TIMER_MCR_MR3I;
        if (feedback_mode != NO_FEEDBACK)
                timer_enable_counter(TIMER2);
}

void feedback_init()
{
        nvic_set_priority(NVIC_TIMER2_IRQ, 10);
        nvic_enable_irq(NVIC_TIMER2_IRQ);
        feedback_set_loop_freq(10000);
        for (unsigned int i=0; i<STAGE_OUTPUTS; i++)
                pi_reset(&stage_outputs[i]);
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
        int32_t error[STAGE_OUTPUTS];

        if (adc_get_trigger_mode() == TRIGGER_OFF) return;

        if (feedback_mode == NO_FEEDBACK) {
                return;

        } else if (feedback_mode == PSD_FEEDBACK) {
                adc_frame_t *sample = adc_get_last_frame();
                for (int i=0; i<STAGE_OUTPUTS; i++) {
                        error[i] = 0;
                        for (unsigned int j=0; j<PSD_INPUTS; j++) 
                                error[i] += psd_fb_gains[j][i] * (psd_fb_setpoint[j] - (int32_t) (*sample)[j+3]);
                        error[i] /= 1<<24;
                }

        } else if (feedback_mode == STAGE_FEEDBACK) {
                adc_frame_t *sample = adc_get_last_frame();
                for (int i=0; i<STAGE_INPUTS; i++) {
                        error[i] = (stage_fb_setpoint[i] - (int32_t) (*sample)[i]) * stage_fb_gains[i][i];
                        error[i] /= 1<<16;
                }
        }

        for (int i=0; i<STAGE_OUTPUTS; i++) {
                struct excitation_buffer* exc = &excitations[i];
                if (exc->length > 0) {
                        error[i] += exc->samples[exc->offset];
                        exc->offset = (exc->offset+1) % exc->length;
                }
                
                if (abs(error[i]) > max_error) {
                        feedback_set_mode(NO_FEEDBACK);
                        return;
                }

                pi_add_sample(&stage_outputs[i], error[i]);
                int32_t v = updates[i].value + pi_get_response(&stage_outputs[i]);
                if (v < 0) v = 0;
                else if (v > 0xffff) v = 0xffff;
                updates[i].value = v;
        }

        feedback_update();
        increment_event_counter(feedback_counter);
}

void timer2_isr(void)
{
        TIMER2_IR |= TIMER_IR_MR0INT;
        do_feedback();
}

