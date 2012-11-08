#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/dma.h>
#include <libopencm3/stm32/usart.h>

#include <string.h>
#include <stdlib.h>
#include "uart.h"

void (*uart_frame_recvd_cb)(unsigned int length, uint8_t *frame) = NULL;

uint8_t tx_buffer[1000] __attribute__((section (".dma_data"))) = { };
uint8_t rx_buffer[512] __attribute__((section (".dma_data"))) = { };
unsigned int rx_length;

/* Frame format:
 *   byte  -2:          0x01 (ASCII SOH)
 *   byte  -1:          N = length
 *   bytes 0 - (N-1):   data
 *   byte  N:           0x04 (ASCII EOT)
 *
 * On receiving all N bytes, an ASCII ACK or NAK character will be returned.
 */
enum rx_state_t {
    RX_IDLE,
    RX_ACTIVE,
};
volatile enum rx_state_t rx_state = RX_IDLE;

void uart_init(int baudrate)
{
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_AHB1ENR_DMA2EN);

    usart_set_baudrate(USART1, baudrate);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_enable_rx_interrupt(USART1);
    usart_enable_error_interrupt(USART1);
    usart_enable(USART1);

    nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM2_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM7_IRQ);

    // Rx DMA
    dma_stream_reset(DMA2, 2);
    dma_set_transfer_mode(DMA2, 2, DMA_SCR_DIR_PER2MEM);
    dma_channel_select(DMA2, 2, DMA_SCR_CHSEL_4);
    dma_enable_transfer_error_interrupt(DMA2, 2);
    dma_enable_transfer_complete_interrupt(DMA2, 2);
    dma_enable_memory_increment_mode(DMA2, 2);
    dma_set_peripheral_address(DMA2, 2, (u32) &USART_DR(USART1));
    dma_set_memory_address(DMA2, 2, (u32) rx_buffer);

    // Tx DMA
    dma_stream_reset(DMA2, 7);
    dma_set_transfer_mode(DMA2, 7, DMA_SCR_DIR_MEM2PER);
    dma_channel_select(DMA2, 7, DMA_SCR_CHSEL_4);
    dma_enable_transfer_error_interrupt(DMA2, 7);
    dma_enable_transfer_complete_interrupt(DMA2, 7);
    dma_enable_memory_increment_mode(DMA2, 7);
    dma_set_peripheral_address(DMA2, 7, (u32) &USART_DR(USART1));
    dma_set_memory_address(DMA2, 7, (u32) tx_buffer);

    USART1_SR = 0;
}

void uart_send_bytes(unsigned int length, uint8_t *buf)
{
    for (int i=0; i<length; i++)
        usart_send_blocking(USART1, buf[i]);
}

void uart_start_tx_from_buffer(unsigned int length, char *buf)
{
    memcpy(tx_buffer, buf, length);
    uart_start_tx(length);
}

void uart_start_tx(unsigned int length)
{
    dma_set_number_of_data(DMA2, 7, length);
    dma_enable_stream(DMA2, 7);
    USART1_SR = 0;
    usart_enable_tx_dma(USART1);
}

static void uart_tx_done()
{
    usart_disable_tx_dma(USART1);
}

static void uart_start_rx(unsigned int length)
{
    if (length == 0) {
        rx_state = RX_IDLE;
        return;
    }
    if (rx_state == RX_ACTIVE) return;
    rx_state = RX_ACTIVE;
    rx_length = length;
    dma_set_number_of_data(DMA2, 2, length);
    dma_enable_stream(DMA2, 2);
    usart_disable_rx_interrupt(USART1);
    usart_enable_rx_dma(USART1);
}

static void uart_rx_done()
{
    usart_enable_rx_interrupt(USART1);
    usart_disable_rx_dma(USART1);

    if (rx_buffer[rx_length-1] == 0x04 && uart_frame_recvd_cb) {
        usart_send_blocking(USART1, 0x06); // ACK
        uart_frame_recvd_cb(rx_length, rx_buffer);
    } else {
        usart_send_blocking(USART1, 0x15); // NAK
    }
    rx_state = RX_IDLE;
}

void dma2_stream2_isr()
{
    if (dma_get_interrupt_flag(DMA2, 2, DMA_ISR_TCIF)) {
        dma_clear_interrupt_flags(DMA2, 2, DMA_ISR_TCIF);
        uart_rx_done();
    }
    if (dma_get_interrupt_flag(DMA2, 2, DMA_ISR_TEIF)) {
        dma_clear_interrupt_flags(DMA2, 2, DMA_ISR_TEIF);
    }
}

void dma2_stream7_isr()
{
    if (dma_get_interrupt_flag(DMA2, 7, DMA_ISR_TCIF)) {
        dma_clear_interrupt_flags(DMA2, 7, DMA_ISR_TCIF);
        uart_tx_done();
    }
    if (dma_get_interrupt_flag(DMA2, 7, DMA_ISR_TEIF)) {
        dma_clear_interrupt_flags(DMA2, 7, DMA_ISR_TEIF);
    }
}

void usart1_isr()
{
    if (usart_get_flag(USART1, USART_SR_ORE)) {
        usart_recv(USART1);
        return;
    }
    if (usart_get_flag(USART1, USART_SR_RXNE)) {
        uint8_t d = (uint8_t) usart_recv(USART1);
        if (rx_state == RX_IDLE && d == 0x01) {
            uint8_t length = usart_recv_blocking(USART1);
            uart_start_rx(length);
        }
    }
}

