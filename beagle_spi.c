#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>

#include "beagle_spi.h"

#define BUFFER_LEN 256

static uint8_t buffer[BUFFER_LEN];
static unsigned int offset;

enum state_t {
    IDLE, WAIT_FOR_LENGTH,
};

static unsigned int msg_length = 0;
enum state_t state = IDLE;

void beagle_spi_init()
{
    // SPI1 (Beagle)
    Pin_Init(ARM_PA4, 1, Alt5);
    Pin_Init(ARM_PA5, 1, Alt5);
    Pin_Init(ARM_PA6, 1, Alt5);
    Pin_Init(ARM_PA7, 1, Alt5);

    offset = 0;

    // Turn things on
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    NVIC_EnableIRQ(SPI1_IRQn);
    SPI1->CR2 = SPI_CR2_RXNEIE;
    SPI1->CR1 = SPI_CR1_SPE;

    // Setup DMA:
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    // TX: DMA2_Stream5
    DMA2_Stream5->CR = 3 << 25; // Channel
    DMA2_Stream5->CR |= 1 << 6; // Mem to Periph
    DMA2_Stream5->CR |= DMA_SxCR_MINC;
    DMA2_Stream5->PAR = (uint32_t) &SPI1->DR;

    // RX: DMA2_Stream0
    DMA2_Stream0->CR = 3 << 25; // Channel
    DMA2_Stream0->CR |= DMA_SxCR_MINC | DMA_SxCR_TCIE;
    DMA2_Stream0->PAR = (uint32_t) &SPI1->DR;
    DMA2_Stream0->M0AR = (uint32_t) &buffer;
}

static int read_byte(uint8_t *dest)
{
    for (unsigned int i=0; i<20; i++) {
        if (SPI1->SR & SPI_SR_RXNE) {
            *dest = SPI1->DR;
            SPI1->DR = 0x0;
            return 0;
        }
    }
    return -1;
}
    
void SPI1_IRQHandler()
{
    if (SPI1->SR & SPI_SR_RXNE) {
        uint8_t tmp;
        tmp = SPI1->DR;
        if (tmp != 0x7e) {
            SPI1->DR = 0xFF;
            return;
        } else
            SPI1->DR = 0xAA;

        if (read_byte(&tmp)) return;
        msg_length = tmp<<8;
        if (read_byte(&tmp)) return;
        msg_length |= tmp;
        DMA2_Stream0->NDTR = msg_length;
        SPI1->CR2 |= SPI_CR2_RXDMAEN;
        DMA2_Stream0->CR |= DMA_SxCR_EN;
    } else {
        SPI1->SR = 0;
    }
}

void process_cmd() {
}

void DMA2_Stream0_IRQHandler() {
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        // Transfer complete
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        SPI1->CR2 &= ~SPI_CR2_RXDMAEN;
        process_cmd();
    }
}

