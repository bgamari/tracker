#include <string.h>
#include <stdlib.h>
#include "uart.h"

void (*uart_frame_recvd_cb)(unsigned int length, uint8_t *frame) = NULL;

uint8_t tx_buffer[1000] __attribute__((section (".dma_data"))) = { 0 };
uint8_t rx_buffer[512] __attribute__((section (".dma_data"))) = { 0 };
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
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = (1<<4) * PeripheralClock / 8 / 2 / baudrate;
    USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    // Rx DMA
    DMA2_Stream2->CR = 0;
    DMA2_Stream2->CR |= 0x4 << 25;
    DMA2_Stream2->CR |= 0x0 << 6; // Peripheral to Memory
    DMA2_Stream2->CR |= DMA_SxCR_MINC;
    DMA2_Stream2->CR |= DMA_SxCR_TCIE;
    DMA2_Stream2->PAR = (uint32_t) &USART1->DR;
    DMA2_Stream2->M0AR = (uint32_t) rx_buffer;
    DMA2_Stream2->NDTR = 0;

    // Tx DMA
    DMA2_Stream7->CR = 0;
    DMA2_Stream7->CR |= 0x4 << 25;
    DMA2_Stream7->CR |= 0x1 << 6; // Memory to Peripheral
    DMA2_Stream7->CR |= DMA_SxCR_MINC;
    DMA2_Stream7->CR |= DMA_SxCR_TCIE;
    DMA2_Stream7->PAR = (uint32_t) &USART1->DR;
    DMA2_Stream7->M0AR = (uint32_t) tx_buffer;
    DMA2_Stream7->NDTR = 0;

    USART1->SR = 0;
}

inline bool uart_readable()
{
    return USART1->SR & USART_SR_RXNE;
}

inline char uart_read_byte()
{
    while (!uart_readable());
    return USART1->DR;
}

bool uart_sendable()
{
    return USART1->SR & USART_SR_TXE;
}

void uart_send_byte(char data)
{
    while (!uart_sendable());
    USART1->DR = data;
}

void uart_send_bytes(unsigned int length, uint8_t *buf)
{
    for (int i=0; i<length; i++)
        uart_send_byte(buf[i]);
}

inline bool uart_tx_active()
{
    return DMA2_Stream7->CR & DMA_SxCR_EN;
}

void uart_start_tx_from_buffer(unsigned int length, uint8_t *buf)
{
    memcpy(tx_buffer, buf, length);
    uart_start_tx(length);
}

void uart_start_tx(unsigned int length)
{
    DMA2_Stream7->NDTR = length;
    DMA2_Stream7->CR |= DMA_SxCR_EN;
    USART1->SR = 0;
    USART1->CR3 |= USART_CR3_DMAT;
}

static void uart_tx_done()
{
    USART1->CR3 &= ~USART_CR3_DMAT;
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
    USART1->CR3 |= USART_CR3_DMAR;
    USART1->CR1 &= ~USART_CR1_RXNEIE;
    DMA2_Stream2->NDTR = length;
    DMA2_Stream2->CR |= DMA_SxCR_EN;
}

static void uart_rx_done()
{
    USART1->CR1 |= USART_CR1_RXNEIE;
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
    if (DMA2->LISR & DMA_LISR_TCIF2) {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
        uart_rx_done();
    }
}

void DMA2_Stream7_IRQHandler()
{
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
        uart_tx_done();
    }
}

void USART1_IRQHandler()
{
    uint32_t flags = USART1->SR;
    char d = USART1->DR;
    if (flags & USART_SR_RXNE) {
        if (rx_state == RX_IDLE && d == 0x01) {
            rx_state = RX_START;
        } else if (rx_state == RX_START)
            uart_start_rx(d);
    }
}

