#include <string.h>
#include <math.h>

#include <libopencm3/stm32/f4/adc.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/pwr.h>
#include <libopencm3/stm32/f4/flash.h>
#include <libopencm3/stm32/f4/syscfg.h>
#include <libopencm3/stm32/f4/usart.h>

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
#include "adc_forward.h"

u8 psd_channels[PSD_INPUTS] = { 2, 1, 0, 13 };
u8 stage_channels[STAGE_INPUTS] = { 9, 8, 15 };

struct adc_t *psd_adc = &adc1;
struct adc_t *stage_adc = &adc2;

#define NLEDS 3
struct pin_t leds[NLEDS] = {
  { port : GPIOC, pin : GPIO15 },
  { port : GPIOC, pin : GPIO0 },
  { port : GPIOC, pin : GPIO1 },
};

void frame_recvd(unsigned int length, uint8_t *frame)
{
    process_cmd((struct cmd_frame_t *) frame);
}

/* This example turns all 4 leds on and then off */
int main(void) {
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_PWREN);
    PWR_CR |= PWR_CR_VOS;
    while (!(PWR_CSR & PWR_CSR_VOSRDY));
    SYSCFG_CMPCR = 0x1; // Enable I/O compensation cell

    flash_set_ws(FLASH_LATENCY_5WS);
    rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);
    init_systick();
    
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPAEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPCEN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_IOPDEN);

    for (int i=0; i<NLEDS; i++)
        pin_setup_output(&leds[i]);

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
    adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_55DOT5CYC);

    adc_config_channels(psd_adc, 4, psd_channels);
    adc_set_trigger_freq(psd_adc, 20000);
    adc_trigger_start(psd_adc);

    adc_config_channels(stage_adc, 3, stage_channels);
    adc_set_trigger_freq(stage_adc, 20000);
    adc_trigger_start(stage_adc);

    dac_init();

    feedback_init();
    adcfwd_start(&adc1);
    //feedback_start();

    // Turn all leds on and then off,
    // with a delay of 0.2s among operations.
    // This goes on indefinitely
    while (1) {
#if 0
        for (int i=0; i<0xffff; i+=100) {
            struct dac_update_t updates = {broadcast, i};
            //delay_ms(1);
            set_dac(1, &updates);
        }
#endif
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
