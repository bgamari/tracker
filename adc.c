#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/timer.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/creg.h>

#include <stdlib.h>
#include <stdbool.h>

#include "adc.h"
#include "timer.h"
#include "pin.h"

static bool running = false;
static unsigned int nsamples;
static uint16_t *buffer;
static unsigned int head; // index in buffer where next sample will be stored
static uint16_t *inactive_buffer;
static uint16_t *last_sample;

struct pin_t os1 = { .port = GPIO3, .pin = GPIOPIN7 };
struct pin_t os2 = { .port = GPIO3, .pin = GPIOPIN6 };
struct pin_t os3 = { .port = GPIO3, .pin = GPIOPIN5 };

struct pin_t reset = { .port = GPIO5, .pin = GPIOPIN2 };
struct pin_t range = { .port = GPIO5, .pin = GPIOPIN6 };
struct pin_t standby = { .port = GPIO5, .pin = GPIOPIN0 }; // inverted

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
    pin_off(&range);
    pin_on(&reset);
    pin_off(&reset);

    uint8_t prescale = 1;
    ssp_init(SSP0_NUM,
             SSP_DATA_16BITS,
             SSP_FRAME_SPI,
             SSP_CPOL_1_CPHA_1,
             1,
             prescale, // FIXME
             SSP_MODE_NORMAL,
             SSP_MASTER,
             SSP_SLAVE_OUT_DISABLE);
    scu_pinmux(P3_3, SCU_CONF_FUNCTION3 | SCU_SSP_IO); // SCK
    scu_pinmux(P3_6, SCU_CONF_FUNCTION3 | SCU_SSP_IO); // CS
    scu_pinmux(P3_7, SCU_CONF_FUNCTION3 | SCU_SSP_IO); // MISO
    scu_pinmux(P3_8, SCU_CONF_FUNCTION3 | SCU_SSP_IO); // MOSI
    scu_pinmux(P1_16, SCU_CONF_FUNCTION4); // T0_MAT0 = STCONV

    // PINT0 = ADC_BUSY = GPIO1[13]
    nvic_enable_irq(NVIC_PIN_INT0_IRQ);
    SCU_PINTSEL0 = (SCU_PINTSEL0 & ~0xff) | 13 | (0x1 << 5);
    GPIO_PIN_INTERRUPT_ISEL &= ~(1 << 0); // Edge sensitive
    //GPIO_PIN_INTERRUPT_IENR |= 1 << 0;
    GPIO_PIN_INTERRUPT_IENF |= 1 << 0;
}

void adc_set_buffers(unsigned int length, uint16_t *buffer1, uint16_t *buffer2)
{
    nsamples = length;
    buffer = buffer1;
    head = 0;
    inactive_buffer = buffer2;
}

int adc_set_trigger_freq(uint32_t freq)
{
    setup_periodic_timer(TIMER0, 2*freq);
    TIMER0_EMR |= (TIMER0_EMR & ~TIMER_EMR_EMC0_MASK) | (TIMER_EMR_EMC_TOGGLE << TIMER_EMR_EMC0_SHIFT);
    if (running)
        TIMER0_TCR |= TIMER_TCR_CEN;
    return 0;
}

int adc_trigger_start()
{
    TIMER0_TCR |= TIMER_TCR_CEN;
    return 0;
}

int adc_trigger_stop()
{
    TIMER0_TCR &= ~TIMER_TCR_CEN;
    return 0;
}

void adc_manual_trigger()
{
    TIMER0_EMR &= ~TIMER_EMR_EM0;
    // FIXME: delay?
    TIMER0_EMR |= TIMER_EMR_EM0;
}

uint16_t *adc_get_active_buffer()
{
    return buffer;
}

uint16_t *adc_get_inactive_buffer()
{
    return inactive_buffer;
}

uint16_t *adc_get_last_sample()
{
    return last_sample;
}

void adc_set_sample_time(enum adc_sample_time_t time)
{
    pin_set(&os1, time & 0x1);
    pin_set(&os2, time & 0x2);
    pin_set(&os3, time & 0x4);
}

// ADC_BUSY fell: ADC sample ready
void pin_int0_isr(void)
{
    // Clear interrupt request
    if (GPIO_PIN_INTERRUPT_IST & 0x1) {
        GPIO_PIN_INTERRUPT_FALL = 0x1;
        GPIO_PIN_INTERRUPT_RISE = 0x1;
        GPIO_PIN_INTERRUPT_IST = 0x1;

        if (buffer == NULL) return;
        for (unsigned int i=0; i<8; i++, head++) 
            buffer[head] = ssp_transfer(SSP0_NUM, 0);
        last_sample = &buffer[head - 8];
        head = head % nsamples;
    }
}
