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
volatile enum {
    RX_IDLE,    // Waiting for Beginning-Of-Packet
    RX_START,   // Waiting for length
    RX_ACTIVE,  // Waiting for 
} rx_state = RX_IDLE;

void uart_init(int baudrate)
{
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_USART1EN);
    usart_enable(USART1);
    usart_set_baudrate(USART1, baudrate);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_enable_rx_interrupt(USART1);

    nvic_enable_irq(NVIC_USART1_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM2_IRQ);
    nvic_enable_irq(NVIC_DMA2_STREAM7_IRQ);

    // Rx DMA
    dma_stream_reset(DMA2, 2);
    dma_set_transfer_mode(DMA2, 2, DMA_SCR_DIR_PER2MEM);
    dma_channel_select(DMA2, 2, 4);
    dma_enable_transfer_error_interrupt(DMA2, 2);
    dma_enable_memory_increment_mode(DMA2, 2);
    dma_set_peripheral_address(DMA2, 2, (u32) &USART_DR(USART1));
    dma_set_memory_address(DMA2, 2, (u32) rx_buffer);

    // Tx DMA
    dma_stream_reset(DMA2, 7);
    dma_set_transfer_mode(DMA2, 7, DMA_SCR_DIR_MEM2PER);
    dma_channel_select(DMA2, 7, 4);
    dma_enable_transfer_error_interrupt(DMA2, 7);
    dma_enable_memory_increment_mode(DMA2, 7);
    dma_set_peripheral_address(DMA2, 7, (u32) &USART_DR(USART1));
    dma_set_memory_address(DMA2, 7, (u32) tx_buffer);

    dma_clear_interrupt_flags(DMA2, 2, 0xffffffff);
}

inline char uart_read_byte()
{
    return usart_recv_blocking(USART1);
}

void uart_send_byte(char data)
{
    usart_send_blocking(USART1, data);
}

void uart_send_bytes(unsigned int length, uint8_t *buf)
{
    for (int i=0; i<length; i++)
        uart_send_byte(buf[i]);
}

inline bool uart_tx_active()
{
    return DMA2_S7CR & DMA_SCR_EN;
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
    usart_enable_rx_dma(USART1);
    usart_disable_rx_interrupt(USART1);
}

static void uart_rx_done()
{
    usart_enable_rx_interrupt(USART1);
    usart_disable_rx_dma(USART1);
    rx_state = RX_IDLE;

    if (rx_buffer[rx_length-1] == 0x04 && uart_frame_recvd_cb) {
        uart_send_byte(0x06); // ACK
        uart_frame_recvd_cb(rx_length, rx_buffer);
    } else {
        uart_send_byte(0x15); // NAK
    }
}

    
void DMA2_Stream2_IRQHandler()
{
    if (dma_get_interrupt_flag(DMA2, 2, DMA_ISR_TCIF)) {
        dma_clear_interrupt_flags(DMA2, 2, DMA_ISR_TCIF);
        uart_rx_done();
    }
}

void DMA2_Stream7_IRQHandler()
{
    if (dma_get_interrupt_flag(DMA2, 7, DMA_ISR_TCIF)) {
        dma_clear_interrupt_flags(DMA2, 7, DMA_ISR_TCIF);
        uart_tx_done();
    }
}

void USART1_IRQHandler()
{
    uint8_t d = (uint8_t) usart_recv(USART1);
    if (usart_get_flag(USART1, USART_SR_RXNE)) {
        if (rx_state == RX_IDLE && d == 0x01) {
            rx_state = RX_START;
        } else if (rx_state == RX_START)
            uart_start_rx(d);
    }
}
