#include <libopencm3/lpc43xx/uart.h>

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

void send_reply(bool ack, uint16_t length, char *data)
{
    uart_write(UART0_NUM, 0x06 ? ack : 0x15);
    uart_write(UART0_NUM, length>>8);
    uart_write(UART0_NUM, length>>0);
    if (length > 0)
        uart_send_bytes(length, data);
}

void process_cmd(struct cmd_frame_t *cmd)
{
    switch (cmd->cmd) {
    case CMD_ECHO:
        send_reply(true, cmd->echo.length, (char *) cmd->echo.data);
        break;

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

    default:
        send_reply(false, 0, NULL);
        return;
    }
}
