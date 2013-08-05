#include "buffer.h"

buffer_t buffers[16] = {};
const unsigned int n_buffers = sizeof(buffers) / sizeof(buffers[0]);

void buffers_init() {
        for (unsigned int i=0; i < n_buffers; i++) {
                buffers[i].sentinel = BUFFER_SENTINEL;
                buffers[i].allocated = false;
        }
}

buffer_t* take_buffer()
{
        for (unsigned int i=0; i < n_buffers; i++) {
                if (!buffers[i].allocated) {
                        buffers[i].allocated = true;
                        return &buffers[i];
                }
        }
        return NULL;
}

void put_buffer(buffer_t* buffer)
{
        if (!buffer->allocated) while(1);
        buffer->allocated = false;
}

