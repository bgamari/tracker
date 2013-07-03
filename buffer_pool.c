#include <stddef.h>
#include <assert.h>

#include "buffer_pool.h"
#include "usb.h"
#include "usb_type.h"

#include <libopencm3/lpc43xx/usb.h>
#include <libopencm3/cm3/cortex.h>

#define USB_ENDPOINT_INDEX(endpoint_address) (((endpoint_address & 0xF) * 2) + ((endpoint_address >> 7) & 1))

enum buffer_status {
        FREE,      /* currently unused */
        ALLOCATED, /* allocated but transfer hasn't been started */
        DONE,      /* transfer finished */
};

struct _buffer_t {
        struct _buffer_t *next; // for use in linked list
        enum buffer_status status;
        unsigned int actual_length;
        uint8_t buffer[512];
};

buffer_t buffers[6];
const unsigned int n_buffers = sizeof(buffers) / sizeof(buffers[0]);

/* head of linked list of currently pending buffers */
buffer_t *endpoint_heads[12] = {};

buffer_t* allocate_buffer()
{
        for (unsigned int i = 0; i < n_buffers; i++) {
                if (buffers[i].status == FREE) {
                        buffers[i].status = ALLOCATED;
                        return &buffers[i];
                }
        }
        return NULL;
}

void free_buffer(buffer_t* buffer)
{
        buffer->status = FREE;
}

