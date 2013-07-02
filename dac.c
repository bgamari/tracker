#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/scu.h>

#include "dac.h"
#include "pin.h"

static uint32_t cmd_buffer[8];
static unsigned int ncmds;

static struct pin_t cs    = { port : GPIO0, pin : GPIOPIN15 };
static struct pin_t nldac = { port : GPIO2, pin : GPIOPIN13 };

void fill_cmd_buffer(unsigned int n, struct dac_update_t *updates)
{
    uint32_t *tmp = cmd_buffer;
    if (n > 8) n = 8;
    for (unsigned int i=0; i<n; i++)
        tmp[i] = (updates[i].channel << 20) | (updates[i].value << 4);
    tmp[n-1] |= 1<<25; // update all registers
    ncmds = n;
}

void send_cmd_buffer() {
    for (unsigned int i=0; i<ncmds; i++) {
        pin_off(&cs);
        uint32_t tmp = cmd_buffer[i];
        for (int j=0; j<4; j++) {
            ssp_transfer(SSP1_NUM, 0xff & (tmp >> 24));
            tmp <<= 8;
            while (!(SSP1_SR & SSP_SR_TNF));
        }
        while (!(SSP1_SR & SSP_SR_TFE));
        pin_on(&cs);
    }
    ncmds = 0;
}

void set_dac(unsigned int n, struct dac_update_t *updates)
{
    // TODO: Check for existing transaction
    if (!(SSP1_SR & SSP_SR_TFE))
        while (1);

    fill_cmd_buffer(n, updates);
    send_cmd_buffer();
}

void dac_init()
{
    ncmds = 0;

    pin_setup_output(&cs);
    pin_setup_output(&nldac);

    pin_off(&nldac);
    pin_on(&cs);

    uint8_t prescale = 1;
    ssp_init(SSP1_NUM,
             SSP_DATA_8BITS,
             SSP_FRAME_SPI,
             SSP_CPOL_0_CPHA_1, 
             2,
             prescale, // FIXME
             SSP_MODE_NORMAL,
             SSP_MASTER,
             SSP_SLAVE_OUT_DISABLE);
    scu_pinmux(P1_19, SCU_CONF_FUNCTION1 | SCU_SSP_IO); // SCK
    scu_pinmux(P0_0, SCU_CONF_FUNCTION1 | SCU_SSP_IO); // MISO
    scu_pinmux(P0_1, SCU_CONF_FUNCTION1 | SCU_SSP_IO); // MOSI
    scu_pinmux(P1_20, SCU_CONF_FUNCTION0 | SCU_GPIO_NOPULL); // CS = GPIO0[15]
    scu_pinmux(P5_4, SCU_CONF_FUNCTION0 | SCU_GPIO_NOPULL); // LDAC = GPIO2[13] 

    // Enable internal reference
    cmd_buffer[0] = (1<<27) | 1;
    ncmds = 1;
    send_cmd_buffer();
}

void load_dac()
{
    pin_on(&nldac);
    pin_off(&nldac);
}

