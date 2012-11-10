#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/f4/spi.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/rcc.h>

#include "beagle_spi.h"

static bool tx_busy = false;

static bool rx_busy = false;
static void* rx_user_data;
static completion_t rx_completion;

void beagle_spi_init()
{
    // SPI1 (Beagle)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5 | GPIO6 | GPIO7);
    gpio_set_af(GPIOA, GPIO_AF5, GPIO4 | GPIO5 | GPIO6 | GPIO7);

    // Turn things on
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SPI1EN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);

    nvic_enable_irq(NVIC_SPI1_IRQ);
    spi_reset(SPI1);
    spi_set_slave_mode(SPI1);
    spi_enable(SPI1);

    // Setup DMA:
    nvic_enable_irq(NVIC_DMA2_STREAM0_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM5_IRQ);

    // TX: DMA2_Stream5
    dma_stream_reset(DMA2, 5);
    dma_channel_select(DMA2, 5, DMA_SCR_CHSEL_3);
    dma_set_transfer_mode(DMA2, 5, DMA_SCR_DIR_MEM2PER);
    dma_enable_memory_increment_mode(DMA2, 5);
    dma_set_peripheral_address(DMA2, 5, (uint32_t) &SPI_DR(SPI1));
    dma_enable_fifo_mode(DMA2, 5);
    dma_enable_transfer_error_interrupt(DMA2, 5);
    dma_enable_transfer_complete_interrupt(DMA2, 5);
    dma_enable_fifo_error_interrupt(DMA2, 5);

    // RX: DMA2_Stream0
    dma_stream_reset(DMA2, 0);
    dma_channel_select(DMA2, 0, DMA_SCR_CHSEL_3);
    dma_set_transfer_mode(DMA2, 0, DMA_SCR_DIR_PER2MEM);
    dma_enable_memory_increment_mode(DMA2, 0);
    dma_set_peripheral_address(DMA2, 0, (uint32_t) &SPI_DR(SPI1));
    dma_enable_fifo_mode(DMA2, 0);
    dma_enable_transfer_error_interrupt(DMA2, 0);
    dma_enable_transfer_complete_interrupt(DMA2, 0);
    dma_enable_fifo_error_interrupt(DMA2, 0);
}

int beagle_spi_dma_tx(uint16_t length, char *buffer)
{
    if (tx_busy) return -1;
    tx_busy = true;
    spi_enable_tx_dma(SPI1);
    dma_set_memory_address(DMA2, 5, (u32) buffer);
    dma_set_number_of_data(DMA2, 5, length);
    dma_enable_stream(DMA2, 5);
    return 0;
}

int beagle_spi_dma_rx(uint16_t length, char *buffer,
                      completion_t rx_cb, void *user_data)
{
    if (rx_busy) return -1;
    rx_busy = true;
    rx_completion = rx_cb;
    rx_user_data = user_data;

    spi_enable_rx_dma(SPI1);
    dma_set_memory_address(DMA2, 0, (u32) buffer);
    dma_set_number_of_data(DMA2, 0, length);
    dma_enable_stream(DMA2, 0);
    return 0;
}

void dma2_stream5_isr() {
    if (dma_get_interrupt_flag(DMA2, 5, DMA_ISR_TCIF)) {
        dma_clear_interrupt_flags(DMA2, 5, DMA_ISR_TCIF);
        spi_disable_tx_dma(SPI1);
        tx_busy = false;
    }
    if (dma_get_interrupt_flag(DMA2, 5, DMA_ISR_TEIF)) {
        dma_clear_interrupt_flags(DMA2, 5, DMA_ISR_TEIF);
    }
    if (dma_get_interrupt_flag(DMA2, 5, DMA_ISR_FEIF)) {
        // This happens for some reason even when the FIFO is
        // disabled. Regardless, it doesn't seem to do any harm.
        dma_clear_interrupt_flags(DMA2, 5, DMA_ISR_FEIF);
    }
}

void dma2_stream0_isr() {
    if (dma_get_interrupt_flag(DMA2, 0, DMA_ISR_TCIF)) {
        // Transfer complete
        dma_clear_interrupt_flags(DMA2, 0, DMA_ISR_TCIF);
        spi_disable_rx_dma(SPI1);
        if (rx_completion)
            rx_completion(rx_user_data);
    }
}

