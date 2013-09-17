#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/timer.h>
#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/gpdma.h>

#include <stdlib.h>
#include <stdbool.h>

#include "adc.h"
#include "timer.h"
#include "clock.h"
#include "pin.h"

#define USE_DMA

static enum trigger_mode trigger_mode = TRIGGER_OFF;
static bool running = false;

static unsigned int nsamples; // length of buffer in int16_t samples
static int16_t *buffer;
static adc_buffer_done_cb buffer_done = NULL; // callback for when buffer has been filled
static unsigned int head; // index in buffer where next sample will be stored
static unsigned int decimation_count = 1;
static unsigned int decimation_factor = 1;

struct pin_t os1 = { .port = GPIO3, .pin = GPIOPIN7 };
struct pin_t os2 = { .port = GPIO3, .pin = GPIOPIN6 };
struct pin_t os3 = { .port = GPIO3, .pin = GPIOPIN5 };

struct pin_t reset = { .port = GPIO5, .pin = GPIOPIN2 };
struct pin_t range = { .port = GPIO5, .pin = GPIOPIN6 };
struct pin_t standby = { .port = GPIO5, .pin = GPIOPIN0 }; // inverted

#ifdef USE_DMA
// This is where samples get placed
int16_t last_sample[2][8] = { };
// index of last_sample buffer which is currently being filled by DMA engine
uint8_t last_sample_idx = 0;
#else
// This is where samples get placed
int16_t last_sample[8] = { };
#endif

#ifdef USE_DMA
static void configure_rx_dma()
{
        GPDMA_C0SRCADDR = (uint32_t) &SSP0_DR;
        GPDMA_C0DESTADDR = 0x0; // DESTADDR will be set in adc_set_buffers
        GPDMA_C0LLI = 0;
        GPDMA_C0CONTROL =
                  GPDMA_CCONTROL_SBSIZE(0x2)  // source burst size = 8
                | GPDMA_CCONTROL_DBSIZE(0x2)  // destination burst size = 8
                | GPDMA_CCONTROL_SWIDTH(0x1)  // halfword
                | GPDMA_CCONTROL_DWIDTH(0x1)  // halfword
                | GPDMA_CCONTROL_S(0x1)       // master 1 can access peripheral
                | GPDMA_CCONTROL_D(0x0)       // master 0 can access memory
                | GPDMA_CCONTROL_DI(0x1)      // destination increment
                ;
        GPDMA_C0CONFIG =
                  GPDMA_CCONFIG_SRCPERIPHERAL(0x9)  // SSP0 RX
                | GPDMA_CCONFIG_FLOWCNTRL(0x2)      // peripheral to memory (DMA controller)
                | GPDMA_CCONFIG_IE(0x1)
                ;
}

static int16_t dummy = 0;

static void configure_tx_dma()
{
        GPDMA_C1SRCADDR = (uint32_t) &dummy;
        GPDMA_C1DESTADDR = (uint32_t) &SSP0_DR;
        GPDMA_C1LLI = 0;
        GPDMA_C1CONTROL =
                  GPDMA_CCONTROL_SBSIZE(0x2)  // source burst size = 8
                | GPDMA_CCONTROL_DBSIZE(0x2)  // destination burst size = 8
                | GPDMA_CCONTROL_SWIDTH(0x1)  // halfword
                | GPDMA_CCONTROL_DWIDTH(0x1)  // halfword
                | GPDMA_CCONTROL_S(0x0)       // master 0 can access memory
                | GPDMA_CCONTROL_D(0x1)       // master 1 can access peripheral
                ;
        GPDMA_C1CONFIG =
                  GPDMA_CCONFIG_DESTPERIPHERAL(0xA)  // SSP0 TX
                | GPDMA_CCONFIG_FLOWCNTRL(0x1)      // memory to peripheral (DMA controller)
                | GPDMA_CCONFIG_IE(0x1)
                ;
}
#endif

void adc_init()
{
        scu_pinmux(P2_13, SCU_CONF_FUNCTION0 | SCU_GPIO_NOPULL); // BUSY
        scu_pinmux(P2_6, SCU_CONF_FUNCTION4); // RANGE
        scu_pinmux(P2_5, SCU_CONF_FUNCTION4); // FRSTDATA
        scu_pinmux(P2_2, SCU_CONF_FUNCTION4); // RESET
        pin_setup_output(&os1);
        pin_setup_output(&os2);
        pin_setup_output(&os3);
        pin_setup_output(&reset);
        pin_setup_output(&range);
        pin_setup_output(&standby);

        pin_on(&standby);
        adc_set_range(ADC_RANGE_PLUS_MINUS_5);
        adc_set_oversampling(ADC_OVERSAMPLE_NONE);
        pin_on(&reset);
        delay_ms(1);
        pin_off(&reset);

	/* use PLL1 as clock source */
	CGU_BASE_SSP0_CLK =
		  CGU_BASE_SSP0_CLK_CLK_SEL(CGU_SRC_PLL1)
		| CGU_BASE_SSP0_CLK_AUTOBLOCK;

        // ADC specified up to 20MHz
        // 204MHz / 10 == 20.4MHz
        ssp_init(SSP0_NUM,
                 SSP_DATA_16BITS,
                 SSP_FRAME_SPI,
                 SSP_CPOL_1_CPHA_1,
                 0,
                 10,
                 SSP_MODE_NORMAL,
                 SSP_MASTER,
                 SSP_SLAVE_OUT_DISABLE);
        scu_pinmux(P3_3, SCU_CONF_FUNCTION2 | SCU_SSP_IO); // SCK
        scu_pinmux(P3_6, SCU_CONF_FUNCTION2 | SCU_SSP_IO); // CS
        scu_pinmux(P3_7, SCU_CONF_FUNCTION2 | SCU_SSP_IO); // MISO
        scu_pinmux(P1_16, SCU_CONF_FUNCTION4); // T0_MAT0 = STCONV

#ifdef USE_DMA
        // enable SSP DMA
        SSP0_DMACR = SSP_DMACR_RXDMAE | SSP_DMACR_TXDMAE;
        // configure GPDMA channels
        GPDMA_CONFIG = GPDMA_CONFIG_E(1);
        GPDMA_INTTCCLEAR = 0xffffffff;
        GPDMA_INTERRCLR = 0xffffffff;
        configure_rx_dma();
        configure_tx_dma();
#endif
        
        // configure PINT0 = ADC_BUSY = GPIO1[13]
        nvic_enable_irq(NVIC_PIN_INT0_IRQ);
        SCU_PINTSEL0 = (SCU_PINTSEL0 & ~0xff) | 13 | (0x1 << 5);
        GPIO_PIN_INTERRUPT_ISEL &= ~(1 << 0); // Edge sensitive
        GPIO_PIN_INTERRUPT_IENF |= 1 << 0; // Falling edge
}

static void setup_buffer(int16_t* buf)
{
        buffer = buf;
        head = 0;
#ifdef USE_DMA
        while (GPDMA_ENBLDCHNS & 0x3);
        GPDMA_C0CONFIG &= ~GPDMA_CCONFIG_E(0x1);
        GPDMA_C1CONFIG &= ~GPDMA_CCONFIG_E(0x1);
        GPDMA_C0DESTADDR = (uint32_t) last_sample[last_sample_idx];
#endif
}

// We finished filling the buffer
int adc_flush()
{
        if (buffer == NULL) {
                return -1;
        }

        if (buffer_done) {
                int16_t *next_buffer = buffer_done(buffer, head);
                setup_buffer(next_buffer);
        } else {
                setup_buffer(NULL);
        }
        return 0;
}

void adc_start(unsigned int samples, int16_t* buf, adc_buffer_done_cb done)
{
        buffer_done = done;
        nsamples = samples;
        setup_buffer(buf);
        running = true;
}

enum trigger_mode adc_get_trigger_mode()
{
        return trigger_mode;
}

void adc_set_trigger_mode(enum trigger_mode mode)
{
        trigger_mode = mode;
        if (trigger_mode != TRIGGER_AUTO)
                TIMER0_TCR &= ~TIMER_TCR_CEN;
        else
                TIMER0_TCR |= TIMER_TCR_CEN;
}

unsigned int adc_get_decimation()
{
        return decimation_factor;
}

#define MAX(x,y) (x > y ? x : y)

int adc_set_decimation(const unsigned int _decimation_factor)
{
        decimation_factor = MAX(1, _decimation_factor);
        decimation_count = decimation_factor;
        return 0;
}

int adc_set_trigger_freq(uint32_t freq)
{
        setup_periodic_timer(TIMER0, freq);
        TIMER0_EMR |= (TIMER0_EMR & ~TIMER_EMR_EMC0_MASK) | (TIMER_EMR_EMC_TOGGLE << TIMER_EMR_EMC0_SHIFT);
        adc_set_trigger_mode(trigger_mode);
        return 0;
}

int adc_manual_trigger()
{
        if (trigger_mode != TRIGGER_MANUAL)
                return -1;

        TIMER0_EMR &= ~TIMER_EMR_EM0;
        // FIXME: delay?
        TIMER0_EMR |= TIMER_EMR_EM0;
        return 0;
}

int16_t *adc_get_active_buffer()
{
        return buffer;
}

int16_t *adc_get_last_frame()
{
#ifdef USE_DMA
        return last_sample[!last_sample_idx];
#else
        return last_sample;
#endif
}

void adc_set_oversampling(enum adc_oversampling_t os)
{
        pin_set(&os1, os & 0x1);
        pin_set(&os2, os & 0x2);
        pin_set(&os3, os & 0x4);
}

void adc_set_range(enum adc_range_t rng)
{
        pin_set(&range, rng);
}

// ADC_BUSY fell: ADC sample ready
void pin_int0_isr(void)
{
        // Clear interrupt request
        if (GPIO_PIN_INTERRUPT_IST & 0x1) {
                GPIO_PIN_INTERRUPT_FALL = 0x1;
                GPIO_PIN_INTERRUPT_RISE = 0x1;
                GPIO_PIN_INTERRUPT_IST = 0x1;

                decimation_count--;
                bool save_sample = buffer != NULL && decimation_count == 0;
                if (save_sample)
                        decimation_count = decimation_factor;

#ifdef USE_DMA
                // ensure previous transaction has completed
                while (GPDMA_ENBLDCHNS & 0x3);
                if (save_sample) {
                        for (unsigned int i=0; i<8; i++, head++)
                                buffer[head] = last_sample[last_sample_idx][i];
                }
                last_sample_idx = !last_sample_idx;
                
                GPDMA_C0CONFIG &= ~GPDMA_CCONFIG_E(0x1);
                GPDMA_C1CONFIG &= ~GPDMA_CCONFIG_E(0x1);
                GPDMA_C0DESTADDR = (uint32_t) last_sample[last_sample_idx];
                GPDMA_C0CONTROL |= GPDMA_CCONTROL_TRANSFERSIZE(8); 
                GPDMA_C1CONTROL |= GPDMA_CCONTROL_TRANSFERSIZE(8);
                GPDMA_C0CONFIG |= GPDMA_CCONFIG_E(0x1);
                GPDMA_C1CONFIG |= GPDMA_CCONFIG_E(0x1);
#else
                for (unsigned int i=0; i<8; i++) {
                        last_sample[i] = ssp_transfer(SSP0_NUM, 0);
                        if (save_sample) {
                                buffer[head] = last_sample[i];
                                head++;
                        }
                }
#endif
                        
                if (buffer != NULL && head >= nsamples)
                        adc_flush();

                increment_event_counter(adc_sample_counter);
        }
}
