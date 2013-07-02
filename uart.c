#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/uart.h>

#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "uart.h"

const bool USE_RX_DMA = true;
const bool USE_TX_DMA = true;

void (*uart_frame_recvd_cb)(unsigned int length, uint8_t *frame) = NULL;

uint8_t tx_buffer[1024] __attribute__((section (".dma_data"))) = { };
uint8_t rx_buffer[512] __attribute__((section (".dma_data"))) = { };
unsigned int rx_length, rx_tail;

/* Frame format:
 *   byte  -3:          0x01 (ASCII SOH)
 *   bytes -2           MSB of length (N)
 *   bytes -1           LSB of length
 *   bytes 0 - (N-1):   data
 *   byte  N:           0x04 (ASCII EOT)
 *
 * On receiving all N bytes, an ASCII ACK (0x06) or NAK (0x15)
 * character will be returned, followed by number of reply bytes of
 * follow (uint16). Finally, the reply data with be sent.
 */

enum rx_state_t {
    RX_IDLE,
    RX_WAIT_LENGTH1,
    RX_WAIT_LENGTH2,
    RX_ACTIVE,
    RX_DONE,
};
volatile enum rx_state_t rx_state = RX_IDLE;

volatile bool tx_busy = false;

struct rational_t {
  unsigned int num;
  unsigned int denom;
};

typedef uint32_t fixed_t;

static struct rational_t nearest_rational(fixed_t value, unsigned int max_denom)
{
  struct rational_t a;
  fixed_t error = value;
  for (unsigned int j=1; j<=max_denom; j++) {
    for (unsigned int i=0; i<j; i++) {
      fixed_t v = (i << 16) / j;
      if (abs(v - value) < error) {
        error = abs(v - value);
        a.num = i;
        a.denom = j;
      }
    }
  }

  return a;
}

uint16_t round_fixed(fixed_t a)
{
  fixed_t trunc = a & 0xffff0000;
  if ((a & 0xffff) > 0xffff/2)
    return (trunc + 0x10000) >> 16;
  else
    return trunc >> 16;
}

fixed_t fixed(uint16_t integral, uint16_t fract)
{
  return (integral << 16) | fract;
}

// TODO: Perhaps this should end up in libopencm3 (libm dependency
// could be problematic)
void uart_init_baud(uart_num_t uart_num,
                    uart_databit_t data_nb_bits,
                    uart_stopbit_t data_nb_stop,
                    uart_parity_t data_parity,
                    unsigned int pclk, unsigned int baudrate)
{
  uint16_t divisor;
  struct rational_t frac;

  if (pclk % (16 * baudrate) == 0) {
    divisor = pclk % (16 * baudrate);
    frac.num = 0;
    frac.denom = 1;
  } else {
    fixed_t fr_est = fixed(1, 0xffff/2);
    uint16_t dl_est;
    do {
      // pclk / 16 / baudrate
      fixed_t pclk_16_baud = ((pclk / baudrate) << 16) / 16;
      dl_est = round_fixed((pclk_16_baud / fr_est) << 16);
      fr_est = pclk_16_baud / dl_est;
    } while (fr_est < fixed(1, 0xffff/10) || fr_est > fixed(1, 9*0xffff/10));
    
    frac = nearest_rational(fr_est-fixed(1,0), 15);
    divisor = dl_est;
  }

  uart_init(uart_num, data_nb_bits, data_nb_stop, data_parity,
            divisor, frac.num, frac.denom);
}

void tracker_uart_init(int baudrate)
{
    uart_init_baud(UART0_NUM, UART_DATABIT_8, UART_STOPBIT_1, UART_PARITY_NONE,
                   CLK_BASE_M4, 115200);

    //uart_init(UART0_NUM, UART_DATABIT_8, UART_STOPBIT_1, UART_PARITY_NONE,
    //          75, 1, 2);

    UART_FCR(UART0) = UART_FCR_FIFO_EN | UART_FCR_TRG_LEV3;
    nvic_enable_irq(NVIC_USART0_IRQ);
    UART_IER(UART0) |= UART_IER_RBRINT_EN;
}

void uart_print(const char *buf)
{
    for (const char* c=buf; *c != 0; c++)
        uart_write(UART0_NUM, *c);
}

void uart_send_bytes(unsigned int length, const char *buf)
{
    for (unsigned int i=0; i<length; i++)
        uart_write(UART0_NUM, buf[i]);
}

static void uart_rx_done()
{
    rx_state = RX_DONE;
    if (rx_buffer[rx_length-1] == 0x04 && uart_frame_recvd_cb) {
        uart_write(UART0_NUM, 0x06); // ACK
        uart_frame_recvd_cb(rx_length, rx_buffer);
    } else {
        uart_write(UART0_NUM, 0x15); // NAK
    }
    rx_state = RX_IDLE;
}

void uart0_isr()
{
    if (UART_IIR(UART0) & UART_IIR_INTID_THRE) {
        uint8_t d = uart_read(UART0_NUM);
        if (rx_state == RX_IDLE && d == 0x01) {
            rx_state = RX_WAIT_LENGTH1;
        } else if (rx_state == RX_WAIT_LENGTH1) {
            rx_length = d << 8;
            rx_state = RX_WAIT_LENGTH2;
        } else if (rx_state == RX_WAIT_LENGTH2) {
            uint16_t length = 0;
            length |= d;

            rx_state = RX_ACTIVE;
            rx_length = length;
            rx_tail = 0;
        } else if (!USE_RX_DMA && rx_state == RX_ACTIVE) {
            rx_buffer[rx_tail] = d;
            rx_tail += 1;
            if (rx_tail == rx_length) {
                uart_rx_done();
            } else if (rx_tail >= sizeof(rx_buffer)) {
                uart_write(UART0_NUM, 0x15);
                rx_state = RX_IDLE;
            }
        } else if (rx_state != RX_IDLE)
            uart_write(UART0_NUM, 0x15);
    }
}
