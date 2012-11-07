#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/spi.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>

#include "beagle_spi.h"

#define BUFFER_LEN 256

static uint8_t buffer[BUFFER_LEN];
static unsigned int offset;

enum state_t {
    IDLE, WAIT_FOR_LENGTH,
};

enum state_t state = IDLE;

void beagle_spi_init()
{
    // SPI1 (Beagle)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO4 | GPIO5 | GPIO6 | GPIO7);

    offset = 0;

    // Turn things on
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);

    nvic_enable_irq(NVIC_SPI1_IRQ);
    spi_reset(SPI1);
    spi_enable_rx_buffer_not_empty_interrupt(SPI1);
    spi_enable(SPI1);

    // Setup DMA:
    nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM5_IRQ);

    // TX: DMA2_Stream5
    dma_stream_reset(DMA2, 5);
    dma_channel_select(DMA2, 5, 3);
    dma_set_transfer_mode(DMA2, 5, DMA_SCR_DIR_MEM2PER);
    dma_enable_memory_increment_mode(DMA2, 5);
    dma_set_peripheral_address(DMA2, 5, (uint32_t) &SPI_DR(SPI1));

    // RX: DMA2_Stream0
    dma_stream_reset(DMA2, 0);
    dma_channel_select(DMA2, 0, 3);
    dma_set_transfer_mode(DMA2, 0, DMA_SCR_DIR_PER2MEM);
    dma_enable_memory_increment_mode(DMA2, 0);
    dma_set_peripheral_address(DMA2, 0, (uint32_t) &SPI_DR(SPI1));
    dma_set_memory_address(DMA2, 0, (u32) buffer);
}

void spi1_isr()
{
}

void dma2_stream0_isr() {
    if (dma_get_interrupt_flag(DMA2, 0, DMA_ISR_TCIF)) {
        // Transfer complete
        dma_clear_interrupt_flags(DMA2, 0, DMA_ISR_TCIF);
        dma_disable_stream(DMA2, 0);
        spi_disable_rx_dma(SPI1);
        //process_cmd();
    }
}

