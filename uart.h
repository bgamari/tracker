#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

extern void (*uart_frame_recvd_cb)(unsigned int length, uint8_t *frame);

void uart_init(int baudrate);
bool uart_tx_active();
void uart_start_tx(unsigned int length);
void uart_start_tx_from_buffer(unsigned int length, char *buf);
void uart_send_bytes(unsigned int length, char *buf);

