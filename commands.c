#include <libopencm3/stm32/f4/usart.h>

#include <string.h>

#include "commands.h"
#include "pin.h"
#include "clock.h"
#include "tracker.h"
#include "beagle_spi.h"
#include "adc.h"
#include "dac.h"
#include "uart.h"
#include "scan.h"
#include "feedback.h"

void send_reply(bool ack, uint16_t length, char *data)
{
    usart_send_blocking(USART1, 0x06 ? ack : 0x15);
    usart_send_blocking(USART1, length>>8);
    usart_send_blocking(USART1, length>>0);
    if (length > 0)
        uart_send_bytes(length, data);
}

void process_cmd(struct cmd_frame_t *cmd)
{
    switch (cmd->cmd) {
    case CMD_ECHO:
        send_reply(true, cmd->echo.length, (char *) cmd->echo.data);
        break;
    case CMD_SPI_ECHO:
        beagle_spi_dma_tx(cmd->echo.length, (char *) cmd->echo.data);
        break;
    case CMD_RUN_SCAN:
        usart_send_blocking(USART1, 0x06);
        usart_send_blocking(USART1, 0); // FIXME
        usart_send_blocking(USART1, 0);
        raster_scan(&cmd->run_scan.raster_scan);
        break;
    case CMD_SET_STAGE_GAINS:
        memcpy(stage_fb_gains, cmd->set_stage_gains, sizeof(stage_fb_gains));
        send_reply(true, 0, NULL);
        break;
    case CMD_SET_PSD_GAINS:
        memcpy(psd_fb_gains, cmd->set_psd_gains, sizeof(psd_fb_gains));
        send_reply(true, 0, NULL);
        break;
    case CMD_SET_OUTPUT_GAINS:
        memcpy(output_gains, cmd->set_output_gains, sizeof(output_gains));
        send_reply(true, 0, NULL);
        break;
    case CMD_START_FEEDBACK:
        feedback_start();
        send_reply(true, 0, NULL);
        break;
    case CMD_STOP_FEEDBACK:
        feedback_stop();
        send_reply(true, 0, NULL);
        break;
    case CMD_SET_FEEDBACK_FREQ:
        feedback_set_loop_freq(cmd->set_feedback_freq);
        send_reply(true, 0, NULL);
        break;
    case CMD_SET_ADC_FREQ:
        adc_set_trigger_freq(&adc1, cmd->set_adc_freq);
        send_reply(true, 0, NULL);
        break;
    default:
        send_reply(false, 0, NULL);
        return;
    }
}
