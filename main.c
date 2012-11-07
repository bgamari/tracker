#include <string.h>
#include <math.h>
#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/syscfg.h>

#include "pin.h"
#include "clock.h"
#include "tracker.h"
#include "beagle_spi.h"
#include "adc.h"
#include "dac.h"
#include "uart.h"
#include "scan.h"
#include "feedback.h"
#include "commands.h"

u8 channels[] = { 0, 1, 2, 12 };

void frame_recvd(unsigned int length, uint8_t *frame)
{
    struct cmd_frame_t *cmd = (struct cmd_frame_t *) frame;
    switch (cmd->cmd) {
    case CMD_ECHO:
        uart_start_tx_from_buffer(cmd->echo.length, (char *) cmd->echo.data);
        break;
    case CMD_RUN_SCAN:
        raster_scan(&cmd->run_scan.raster_scan);
        uart_start_tx_from_buffer(1, "\x06");
        break;
    case CMD_SET_GAINS:
        memcpy(feedback_gains, cmd->set_gains.feedback_gains, sizeof(feedback_gains));
        uart_start_tx_from_buffer(1, "\x06");
        break;
    case CMD_START_FEEDBACK:
        feedback_start();
        uart_start_tx_from_buffer(1, "\x06");
        break;
    case CMD_STOP_FEEDBACK:
        feedback_stop();
        uart_start_tx_from_buffer(1, "\x06");
        break;
    default:
        uart_start_tx_from_buffer(1, "\x15");
        return;
    }
}

#define NLEDS 3
struct pin_t leds[NLEDS] = {
  { .port = GPIOC, .pin = GPIO15 },
  { .port = GPIOC, .pin = GPIO0 },
  { .port = GPIOC, .pin = GPIO1 },
};

/* This example turns all 4 leds on and then off */
int main(void) {
    // Access 4 leds predefined as LED1, LED2, LED3, LED4
    init_clock();
    init_systick();
    
    for (int i=0; i<NLEDS; i++)
        pin_setup_output(&leds[i]);

    SYSCFG_CMPCR = 0x1; // Enable I/O compensation cell

    // UART1 (Beagle)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);
    // UART3
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10 | GPIO11);
    gpio_set_af(GPIOB, GPIO_AF7, GPIO10 | GPIO11);

    uart_init(115200);
    uart_frame_recvd_cb = frame_recvd;
    beagle_spi_init();

    adc_init();
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_55DOT5CYC);
    adc_set_regular_sequence(ADC1, 4, channels);

    dac_spi_init();
    //dac_i2s_init();
    feedback_init();

    feedback_start();

    // Turn all leds on and then off,
    // with a delay of 0.2s among operations.
    // This goes on indefinitely
    while (1) {
#if 1
        for (int i = 0; i < NLEDS; i++) {
            pin_on(&leds[i]);
            delay_ms(200);
        }
        for (int i = 0; i < NLEDS; i++) {
            pin_off(&leds[i]);
            delay_ms(200);
        }
#endif
    }
}

// vim:expandtab:smartindent:tabstop=4:softtabstop=4:shiftwidth=4:
