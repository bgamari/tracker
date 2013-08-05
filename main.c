#include <string.h>
#include <math.h>

#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/scu.h>

#include "pin.h"
#include "clock.h"
#include "tracker.h"
#include "buffer.h"
#include "adc.h"
#include "dac.h"
#include "uart.h"
#include "feedback.h"
#include "commands.h"
#include "tracker_usb.h"

#include <libopencm3/lpc43xx/uart.h>

uint8_t psd_channels[PSD_INPUTS] = { 2, 1, 0, 13 };
uint8_t stage_channels[STAGE_INPUTS] = { 9, 8, 15 };

#define NLEDS 4
struct pin_t leds[NLEDS] = {
        { port : GPIO1, pin : GPIOPIN12 },
        { port : GPIO1, pin : GPIOPIN11 },
        { port : GPIO0, pin : GPIOPIN14 },
        { port : GPIO2, pin : GPIOPIN8  },
};

void frame_recvd(unsigned int length, uint8_t *frame)
{
        process_cmd((struct cmd_frame_t *) frame);
}

static void init_clock()
{
        /* set xtal oscillator to low frequency mode */
        CGU_XTAL_OSC_CTRL &= ~CGU_XTAL_OSC_CTRL_HF;

        /* power on the oscillator and wait until stable */
        CGU_XTAL_OSC_CTRL &= ~CGU_XTAL_OSC_CTRL_ENABLE; /* power on oscillator */

        /* use XTAL_OSC as clock source for BASE_M4_CLK (CPU) */
        CGU_BASE_M4_CLK = CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_XTAL);

        /* use XTAL_OSC as clock source for APB1 */
        CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_AUTOBLOCK
                | CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_XTAL);

        /* use XTAL_OSC as clock source for PLL1 */
        /* Start PLL1 at 12MHz * 17 / (2+2) = 51MHz. */
        CGU_PLL1_CTRL = CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_XTAL)
                | CGU_PLL1_CTRL_PSEL(1)
                | CGU_PLL1_CTRL_NSEL(0)
                | CGU_PLL1_CTRL_MSEL(16)
                | CGU_PLL1_CTRL_PD;

        /* power on PLL1 and wait until stable */
        CGU_PLL1_CTRL &= ~CGU_PLL1_CTRL_PD;
        while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK));

        /* use PLL1 as clock source for BASE_M4_CLK (CPU) */
        CGU_BASE_M4_CLK = CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_PLL1);

        /* Move PLL1 up to 12MHz * 17 = 204MHz. */
        CGU_PLL1_CTRL = CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_XTAL)
                | CGU_PLL1_CTRL_PSEL(0)
                | CGU_PLL1_CTRL_NSEL(0)
                | CGU_PLL1_CTRL_MSEL(16)
                | CGU_PLL1_CTRL_FBSEL;
        //| CGU_PLL1_CTRL_DIRECT;

        /* wait until stable */
        while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK));

        /* use XTAL_OSC as clock source for PLL0USB */
        CGU_PLL0USB_CTRL = CGU_PLL0USB_CTRL_PD
                | CGU_PLL0USB_CTRL_AUTOBLOCK
                | CGU_PLL0USB_CTRL_CLK_SEL(CGU_SRC_XTAL);
        while (CGU_PLL0USB_STAT & CGU_PLL0USB_STAT_LOCK);

        /* configure PLL0USB to produce 480 MHz clock from 12 MHz XTAL_OSC */
        /* Values from User Manual v1.4 Table 94, for 12MHz oscillator. */
        CGU_PLL0USB_MDIV = 0x06167FFA;
        CGU_PLL0USB_NP_DIV = 0x00302062;
        CGU_PLL0USB_CTRL |= (CGU_PLL0USB_CTRL_PD
                             | CGU_PLL0USB_CTRL_DIRECTI
                             | CGU_PLL0USB_CTRL_DIRECTO
                             | CGU_PLL0USB_CTRL_CLKEN);

        /* power on PLL0USB and wait until stable */
        CGU_PLL0USB_CTRL &= ~CGU_PLL0USB_CTRL_PD;
        while (!(CGU_PLL0USB_STAT & CGU_PLL0USB_STAT_LOCK));

        /* use PLL0USB as clock source for USB0 */
        CGU_BASE_USB0_CLK = CGU_BASE_USB0_CLK_AUTOBLOCK
                | CGU_BASE_USB0_CLK_CLK_SEL(CGU_SRC_PLL0USB);

        /* Switch peripheral clock over to use PLL1 (204MHz) */
        CGU_BASE_PERIPH_CLK = CGU_BASE_PERIPH_CLK_AUTOBLOCK
                | CGU_BASE_PERIPH_CLK_CLK_SEL(CGU_SRC_PLL1);

        /* Switch APB1 clock over to use PLL1 (204MHz) */
        CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_AUTOBLOCK
                | CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_PLL1);
}

buffer_t* active_buffer = NULL;

int16_t* buffer_full(int16_t* buffer_data)
{
        buffer_t* buffer = (buffer_t*) buffer_data;
        if (adc_streaming)
                tracker_usb_send_buffer(buffer);
        else
                put_buffer(buffer);

        buffer_t* active_buffer = take_buffer();
        if (active_buffer == NULL) while (1); // uh oh
        return active_buffer->data;
}

/* This example turns all 4 leds on and then off */
int main(void) {
        CREG_FLASHCFGA = (CREG_FLASHCFGA & ~CREG_FLASHCFGA_FLASHTIM_MASK) | CREG_FLASHCFGA_FLASHTIM(0x9);
        CREG_FLASHCFGB = (CREG_FLASHCFGB & ~CREG_FLASHCFGB_FLASHTIM_MASK) | CREG_FLASHCFGB_FLASHTIM(0x9);
    
        init_clock();
        init_systick();
        usb_init();

        for (int i=0; i<NLEDS; i++)
                pin_setup_output(&leds[i]);

        // UART0
        scu_pinmux(P2_1, SCU_CONF_FUNCTION1); // RXD
        scu_pinmux(P2_0, SCU_CONF_FUNCTION1); // TXD
        // UART3
        scu_pinmux(P2_4, SCU_CONF_FUNCTION2); // RXD
        scu_pinmux(P2_3, SCU_CONF_FUNCTION2); // TXD

        tracker_uart_init(115200);
        //uart_frame_recvd_cb = frame_recvd;
        uart_print("Hello world!\n");

        dac_init();
        adc_init();
        buffers_init();
        active_buffer = take_buffer();
        if (active_buffer == NULL) while (1);
        adc_start(BUFFER_SIZE, active_buffer->data, buffer_full);
        adc_set_trigger_freq(200);
        adc_set_trigger_mode(TRIGGER_AUTO);

#if 0
        feedback_init();
        feedback_start();
#endif

        // Turn all leds on and then off,
        // with a delay of 0.2s among operations.
        // This goes on indefinitely
        while (1) {
                uart_print("Hello world!\n");
#if 1
                for (int i=0; i<0xffff; i+=100) {
                        struct dac_update_t updates = {broadcast, i};
                        delay_ms(1);
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
