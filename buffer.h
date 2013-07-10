#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define BUFFER_SIZE 256

typedef struct _buffer_t buffer_t;

void buffers_init();
buffer_t* take_buffer();
void put_buffer(buffer_t* buffer);

// Everything below here is internal

#define BUFFER_SENTINEL 0xdeadbeef

struct _buffer_t {
        uint16_t data[BUFFER_SIZE];
#ifdef BUFFER_SENTINEL
        uint32_t sentinel;
#endif
        bool allocated;
};

static inline void buffer_check_sentinel(buffer_t* buffer)
{
#ifdef BUFFER_SENTINEL
        if (buffer->sentinel != BUFFER_SENTINEL) while (1);
#endif
}

static inline buffer_t* buffer_from_pointer(void* ptr)
{
        buffer_t* buffer = ptr;
        buffer_check_sentinel(buffer);
        return buffer;
}
