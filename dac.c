#include "libmanyuc.h"
#include "dac.h"

static uint32_t cmd_buffer[8];
static unsigned int ncmds;

// On SPI2
// Use DMA2_Stream3 for tx
// Use I2S

static Pin_t cs, nldac, nclr;

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
    uint32_t unused;
    for (int i=0; i<ncmds; i++) {
        Pin_Off(cs);
        uint32_t tmp = cmd_buffer[i];
        for (int j=0; j<4; j++) {
            SPI2->DR = tmp >> 24;
            unused = SPI2->DR;
            tmp <<= 8;
            while (!(SPI2->SR & SPI_SR_TXE));
        }
        //for (int i=0; i<10; i++) unused++;
        Pin_On(cs);
        //Delay(0.001);
    }
    ncmds = 0;
}

void set_dac(unsigned int n, struct dac_update_t *updates)
{
    // TODO: Check for existing transaction
    //while (!(SPI2->SR & SPI_SR_TXE));

    fill_cmd_buffer(n, updates);
    send_cmd_buffer();
}

void spi_init()
{
    ncmds = 0;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    //Pin_Init(ARM_PB12, 1, Alt5);
    cs = Pin_Init(ARM_PB12, 1, Output);
    Pin_Init(ARM_PB13, 1, Alt5); // SCLK
    nldac = Pin_Init(ARM_PB14, 1, Output); // nLDAC
    Pin_Init(ARM_PB15, 1, Alt5); // MOSI
    nclr = Pin_Init(ARM_PC6, 1, Output); // nCLR

    Pin_Off(nldac);
    Pin_Off(nclr);

    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI2->CR1 |= SPI_CR1_SPE;

    cmd_buffer[0] = (1<<27) | 1;
    ncmds = 1;
    send_cmd_buffer();
    ncmds = 0;
}

void i2s_init()
{
    ncmds = 0;

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    Pin_Init(ARM_PB12, 1, Alt5);
    Pin_Init(ARM_PB13, 1, Alt5); // SCLK
    nldac = Pin_Init(ARM_PB14, 1, Output); // nLDAC
    Pin_Init(ARM_PB15, 1, Alt5); // MOSI
    nclr = Pin_Init(ARM_PC6, 1, Output); // nCLR

    Pin_Off(nldac);
    Pin_Off(nclr);

    SPI2->I2SCFGR = SPI_I2SCFGR_I2SMOD;
    SPI2->I2SCFGR |= 0x2 << 8; // Master, transmit
    SPI2->I2SCFGR |= 0x3 << 4; // PCM standard
    SPI2->I2SCFGR |= 0x2 << 1; // 32-bit data length
    SPI2->I2SPR = SPI_I2SPR_MCKOE | 10; // Prescaler
    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void SPI2_IRQHanlder() {

}
