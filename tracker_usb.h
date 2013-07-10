#pragma once

#include "buffer.h"

extern bool adc_streaming;

void usb_init(void);

void send_reply(void *data, uint16_t length);

void tracker_usb_send_buffer(buffer_t* buffer);
