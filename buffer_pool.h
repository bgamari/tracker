#pragma once

#include "usb_type.h"

typedef struct _buffer_t buffer_t;

buffer_t* allocate_buffer();

void free_buffer(buffer_t* buffer);

void transmit_buffer(buffer_t* buffer,
                     const usb_endpoint_t* const endpoint,
                     unsigned int actual_length);

void receive_buffer(buffer_t* buffer,
                    const usb_endpoint_t* const endpoint);
