#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "tracker.h"

extern void (*uart_frame_recvd_cb)(unsigned int length, uint8_t *frame);

void uart_init(int baudrate);
void uart_send_bytes(unsigned int length, char *buf);

