#include <libopencm3/lpc43xx/uart.h>

#include "tracker_usb.h"

#include <string.h>

#include "commands.h"
#include "pin.h"
#include "clock.h"
#include "tracker.h"
#include "adc.h"
#include "dac.h"
#include "uart.h"
#include "scan.h"
#include "feedback.h"

uint8_t reply_buffer[512];

#define ACK 0x06
#define NACK 0x07

void process_cmd(struct cmd_frame_t *cmd)
{
        switch (cmd->cmd) {
        case CMD_ECHO:
                reply_buffer[0] = ACK;
                memcpy(&reply_buffer[1], cmd->echo.data, cmd->echo.length);
                send_reply(reply_buffer, cmd->echo.length+1);
                break;

#if 0
        case CMD_RUN_SCAN:
                uart_write(UART0_NUM, 0x06);
                uart_write(UART0_NUM, 0); // FIXME
                uart_write(UART0_NUM, 0);
                raster_scan(&cmd->run_scan.raster_scan);
                break;

        case CMD_SET_STAGE_GAINS:
                memcpy(stage_fb_gains, cmd->set_stage_gains, sizeof(stage_fb_gains));
                send_reply(true, 0, NULL);
                break;
        case CMD_SET_STAGE_SETPOINT:
                memcpy(stage_fb_setpoint, cmd->set_stage_setpoint, sizeof(stage_fb_setpoint));
                send_reply(true, 0, NULL);
                break;

        case CMD_SET_PSD_GAINS:
                memcpy(psd_fb_gains, cmd->set_psd_gains, sizeof(psd_fb_gains));
                send_reply(true, 0, NULL);
                break;
        case CMD_SET_PSD_SETPOINT:
                memcpy(psd_fb_setpoint, cmd->set_psd_setpoint, sizeof(psd_fb_setpoint));
                send_reply(true, 0, NULL);
                break;
        
        case CMD_SET_MAX_ERROR:
                max_error = cmd->set_max_error;
                send_reply(true, 0, NULL);
                break;

        case CMD_SET_OUTPUT_GAINS:
                memcpy(output_gains, cmd->set_output_gains, sizeof(output_gains));
                send_reply(true, 0, NULL);
                break;

        case CMD_SET_ADC_FREQ:
                adc_set_trigger_freq(cmd->set_adc_freq);
                send_reply(true, 0, NULL);
                break;

        case CMD_SET_FEEDBACK_FREQ:
                feedback_set_loop_freq(cmd->set_feedback_freq);
                send_reply(true, 0, NULL);
                break;

        case CMD_SET_FEEDBACK_MODE:
                feedback_mode = cmd->set_feedback_mode;
                send_reply(true, 0, NULL);
                break;
#endif

        default:
                reply_buffer[0] = NACK;
                send_reply(reply_buffer, 1);
                return;
        }
}
