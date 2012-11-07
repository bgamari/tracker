#include <libopencm3/stm32/f4/spi.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>

#include "dac.h"
#include "pin.h"

static uint32_t cmd_buffer[8];
static unsigned int ncmds;

// On SPI2
// Use DMA2_Stream3 for tx
// Use I2S

static struct pin_t cs    = { .port = GPIOB, .pin = GPIO12 };
static struct pin_t nldac = { .port = GPIOB, .pin = GPIO14 };
static struct pin_t nclr  = { .port = GPIOC, .pin = GPIO6 };

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
    for (int i=0; i<ncmds; i++) {
        pin_off(&cs);
        uint32_t tmp = cmd_buffer[i];
        for (int j=0; j<4; j++) {
            spi_write(SPI2, 0xff & (tmp >> 24));
            tmp <<= 8;
            while (!(SPI2_SR & SPI_SR_TXE));
        }
        //for (int i=0; i<10; i++) unused++;
        pin_on(&cs);
        //Delay(0.001);
    }
    ncmds = 0;
}

void set_dac(unsigned int n, struct dac_update_t *updates)
{
    // TODO: Check for existing transaction
    //while (!(SPI2_SR & SPI_SR_TXE));
    if (!(SPI2_SR & SPI_SR_TXE))
        while (1);

    fill_cmd_buffer(n, updates);
    send_cmd_buffer();
}

void dac_spi_init()
{
    ncmds = 0;

    RCC_APB1ENR |= RCC_APB1ENR_SPI2EN;
    pin_setup_output(&cs);
    pin_setup_output(&nclr);
    pin_setup_output(&nldac);
    gpio_set_af(GPIOB, GPIO_AF5, GPIO13 | GPIO15); // SCLK, MOSI

    pin_off(&nldac);
    pin_off(&nclr);

    spi_reset(SPI2);
    spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_2,
                    SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE, SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_set_bidirectional_transmit_only_mode(SPI2);
    spi_enable_ss_output(SPI2);
    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);
    spi_enable(SPI2);

    cmd_buffer[0] = (1<<27) | 1;
    ncmds = 1;
    send_cmd_buffer();
    ncmds = 0;
}

void i2s_init()
{
    ncmds = 0;

    RCC_APB1ENR |= RCC_APB1ENR_SPI2EN;
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO15); // SCLK MOSI
    gpio_set_af(GPIOB, GPIO_AF5, GPIO12 | GPIO13 | GPIO15); // SCLK MOSI

    pin_setup_output(&cs);
    pin_setup_output(&nclr);
    pin_setup_output(&nldac);
    pin_off(&nldac);
    pin_off(&nclr);

    SPI2_I2SCFGR = SPI_I2SCFGR_I2SMOD;
    SPI2_I2SCFGR |= 0x2 << 8; // Master, transmit
    SPI2_I2SCFGR |= 0x3 << 4; // PCM standard
    SPI2_I2SCFGR |= 0x2 << 1; // 32-bit data length
    SPI2_I2SPR = SPI_I2SPR_MCKOE | 10; // Prescaler
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void SPI2_IRQHandler() {

}
