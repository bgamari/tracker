#include <libopencm3/lpc43xx/uart.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/cm3/scb.h>

#include "hackrf_usb/usb.h"
#include "tracker_usb.h"

#include <string.h>

#include "commands.h"
#include "pin.h"
#include "clock.h"
#include "tracker.h"
#include "dac.h"
#include "uart.h"
#include "feedback.h"

struct reply {
        uint8_t cmd;
        uint8_t status;
        uint8_t data[490];
};

struct reply reply;

#define ACK 0x06
#define NACK 0x07

static void send_ack(void)
{
        reply.status = ACK;
        send_reply(&reply, 2);
}

static void send_nack(void)
{
        reply.status = NACK;
        send_reply(&reply, 2);
}

void process_cmd(struct cmd_frame_t *cmd)
{
        int res;
        
        reply.status = ACK; // change to NACK as necessary
        reply.cmd = cmd->cmd;

        switch (cmd->cmd) {
        case CMD_ECHO:
                reply.data[0] = cmd->echo.length;
                memcpy(&reply.data[1], cmd->echo.data, cmd->echo.length);
                send_reply(&reply, cmd->echo.length+3);
                break;

        case CMD_RESET:
                if (cmd->reset_magic == RESET_MAGIC) {
                        usb_peripheral_reset();
                        CREG_M4MEMMAP = 0x10400000;
                        scb_reset_system();
                } else {
                        send_nack();
                }
                break;

        case CMD_GET_EVENT_COUNTERS:
        {
                struct event_counters counters = get_last_event_counters();
                memcpy(reply.data, &counters, sizeof(struct event_counters));
                send_reply(&reply, 2+sizeof(struct event_counters));
                break;
        }

        case CMD_GET_STAGE_GAINS:
                memcpy(reply.data, stage_fb_gains, sizeof(stage_fb_gains));
                send_reply(&reply, 2+sizeof(stage_fb_gains));
                break;

        case CMD_SET_STAGE_GAINS:
                memcpy(stage_fb_gains, cmd->set_stage_gains, sizeof(stage_fb_gains));
                send_ack();
                break;
                
        case CMD_GET_STAGE_SETPOINT:
                memcpy(reply.data, stage_fb_setpoint, sizeof(stage_fb_setpoint));
                send_reply(&reply, 2+sizeof(stage_fb_setpoint));
                break;

        case CMD_SET_STAGE_SETPOINT:
                memcpy(stage_fb_setpoint, cmd->set_stage_setpoint, sizeof(stage_fb_setpoint));
                send_ack();
                break;

        case CMD_GET_PSD_GAINS:
                memcpy(reply.data, psd_fb_gains, sizeof(psd_fb_gains));
                send_reply(&reply, 2+sizeof(psd_fb_gains));
                break;

        case CMD_SET_PSD_GAINS:
                memcpy(psd_fb_gains, cmd->set_psd_gains, sizeof(psd_fb_gains));
                send_ack();
                break;

        case CMD_GET_PSD_SETPOINT:
                memcpy(reply.data, psd_fb_setpoint, sizeof(psd_fb_setpoint));
                send_reply(&reply, 2+sizeof(psd_fb_setpoint));
                break;

        case CMD_SET_PSD_SETPOINT:
                memcpy(psd_fb_setpoint, cmd->set_psd_setpoint, sizeof(psd_fb_setpoint));
                send_ack();
                break;
        
        case CMD_GET_MAX_ERROR:
                memcpy(reply.data, &max_error, sizeof(max_error));
                send_reply(&reply, 2+sizeof(max_error));
                break;

        case CMD_SET_MAX_ERROR:
                max_error = cmd->set_max_error;
                send_ack();
                break;

        case CMD_GET_OUTPUT_GAINS:
        {
                fixed16_t* tmp = (fixed16_t*) reply.data;
                for (unsigned int i=0; i<STAGE_OUTPUTS; i++) {
                        *tmp = stage_outputs[i].p_gain; tmp++;
                        *tmp = stage_outputs[i].i_gain; tmp++;
                }
                send_reply(&reply, 2+2*STAGE_OUTPUTS*sizeof(fixed16_t));
                break;
        }

        case CMD_SET_OUTPUT_GAINS:
        {
                for (unsigned int i=0; i<STAGE_OUTPUTS; i++) {
                        stage_outputs[i].p_gain = cmd->set_output_gains[i][0];
                        stage_outputs[i].i_gain = cmd->set_output_gains[i][1];
                }
                send_ack();
                break;
        }

        case CMD_GET_OUTPUT_TAUS:
        {
                for (unsigned int i=0; i<STAGE_OUTPUTS; i++) {
                        reply.data[i] = pi_get_tau(&stage_outputs[i]);
                }
                send_reply(&reply, 2+3);
                break;
        }

        case CMD_SET_OUTPUT_TAUS:
        {
                for (unsigned int i=0; i<STAGE_OUTPUTS; i++) {
                        pi_set_tau(&stage_outputs[i], cmd->set_output_taus[i]);
                }
                send_ack();
                break;
        }

        case CMD_SET_ADC_FREQ:
                adc_set_trigger_freq(cmd->set_adc_freq);
                send_ack();
                break;

        case CMD_SET_ADC_TRIGGER_MODE:
                adc_set_trigger_mode(cmd->set_adc_trigger_mode);
                send_ack();
                break;

        case CMD_START_ADC_STREAM:
                adc_streaming = true;
                send_ack();
                break;

        case CMD_STOP_ADC_STREAM:
                adc_streaming = false;
                send_ack();
                break;

        case CMD_GET_ADC_DECIMATION:
        {
                uint32_t* r = (uint32_t*) reply.data;
                r[0] = adc_get_decimation();
                send_reply(&reply, 2+4);
                break;
        }
                
        case CMD_SET_ADC_DECIMATION:
                if (adc_set_decimation(cmd->set_adc_decimation) == 0) {
                        send_ack();
                } else {
                        send_nack();
                }
                break;

        case CMD_SET_FEEDBACK_FREQ:
                feedback_set_loop_freq(cmd->set_feedback_freq);
                send_ack();
                break;

        case CMD_GET_FEEDBACK_MODE:
                reply.data[0] = feedback_get_mode();
                send_reply(&reply, 3);
                break;

        case CMD_SET_FEEDBACK_MODE:
                feedback_set_mode(cmd->set_feedback_mode);
                send_ack();
                break;

        case CMD_SET_RAW_POS:
                if (feedback_set_position(cmd->set_raw_pos) == 0) {
                        send_ack();
                } else {
                        send_nack();
                }
                break;

        case CMD_CLEAR_PATH:
                clear_path();
                send_ack();
                break;
                
        case CMD_ENQUEUE_POINTS:
                res = 0;
                if (cmd->enqueue_points.npts > 0)
                        res = enqueue_points((uint16_t*) &cmd->enqueue_points.points, cmd->enqueue_points.npts);
                if (res == 0) {
                        reply.data[0] = is_path_running();
                        send_reply(&reply, 3);
                } else {
                        send_nack();
                }
                break;

        case CMD_START_PATH:
                if (start_path(cmd->start_path.freq, cmd->start_path.synchronous_adc) == 0) {
                        send_ack();
                } else {
                        send_nack();
                }
                break;

        case CMD_SET_EXCITATION:
        {
                struct set_excitation* const se = &cmd->set_excitation;
                struct excitation_buffer* const exc = &excitations[se->channel];

                if ((se->length + se->offset > MAX_EXCITATION_LENGTH)
                    || (se->channel >= 3))
                {
                        send_nack();
                        break;
                }

                exc->length = 0;
                exc->offset = 0;
                if (se->length != 0) {
                        memcpy(&exc->samples[se->offset],
                               &se->samples,
                               sizeof(uint16_t) * se->length);
                }
                exc->length = se->total_length;
                send_ack();
                break;
        }

        default:
                send_nack();
                return;
        }
}
