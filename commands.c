#include <libopencm3/lpc43xx/uart.h>

#include "tracker_usb.h"

#include <string.h>

#include "commands.h"
#include "pin.h"
#include "clock.h"
#include "tracker.h"
#include "dac.h"
#include "uart.h"

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
        
        reply.cmd = cmd->cmd;

        switch (cmd->cmd) {
        case CMD_ECHO:
                reply.status = ACK;
                reply.data[0] = cmd->echo.length;
                memcpy(&reply.data[1], cmd->echo.data, cmd->echo.length);
                send_reply(&reply, cmd->echo.length+3);
                break;

        case CMD_SET_STAGE_GAINS:
                memcpy(stage_fb_gains, cmd->set_stage_gains, sizeof(stage_fb_gains));
                send_ack();
                break;
                
        case CMD_SET_STAGE_SETPOINT:
                memcpy(stage_fb_setpoint, cmd->set_stage_setpoint, sizeof(stage_fb_setpoint));
                send_ack();
                break;

        case CMD_SET_PSD_GAINS:
                memcpy(psd_fb_gains, cmd->set_psd_gains, sizeof(psd_fb_gains));
                send_ack();
                break;

        case CMD_SET_PSD_SETPOINT:
                memcpy(psd_fb_setpoint, cmd->set_psd_setpoint, sizeof(psd_fb_setpoint));
                send_ack();
                break;
        
        case CMD_SET_MAX_ERROR:
                max_error = cmd->set_max_error;
                send_ack();
                break;

        case CMD_SET_OUTPUT_GAINS:
                memcpy(output_gains, cmd->set_output_gains, sizeof(output_gains));
                send_ack();
                break;

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

        case CMD_SET_FEEDBACK_FREQ:
                feedback_set_loop_freq(cmd->set_feedback_freq);
                send_ack();
                break;

        case CMD_SET_FEEDBACK_MODE:
                feedback_set_mode(cmd->set_feedback_mode);
                send_ack();
                break;

        case CMD_CLEAR_PATH:
                clear_path();
                send_ack();
                break;
                
        case CMD_ENQUEUE_POINTS:
                res = enqueue_points((uint16_t*) &cmd->enqueue_points.points, cmd->enqueue_points.npts);
                if (res == 0) {
                        reply.status = ACK;
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

        default:
                send_nack();
                return;
        }
}
