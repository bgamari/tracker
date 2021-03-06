#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

extern void (*uart_frame_recvd_cb)(unsigned int length, uint8_t *frame);

void tracker_uart_init(int baudrate);
void uart_send_bytes(unsigned int length, const char *buf);
void uart_print(const char *buf);

