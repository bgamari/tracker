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
        PENDING,   /* waiting for transfer to finish */
        DONE,      /* transfer finished */
};

struct _buffer_t {
        struct _buffer_t *next; // for use in linked list
        enum buffer_status status;
        unsigned int actual_length;
        usb_transfer_descriptor_t td __attribute__((aligned(64)));
        void (*completion_cb)(buffer_t*, unsigned int);
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

static bool usb_endpoint_is_in(const uint_fast8_t endpoint_address) {
	return (endpoint_address & 0x80) ? true : false;
}

static void start_transfer(buffer_t* buffer,
                           const usb_endpoint_t* const endpoint)
{
        const uint_fast8_t index = USB_ENDPOINT_INDEX(endpoint->address);
        usb_transfer_descriptor_t* const td = &buffer->td;

        // Setup transfer descriptor
        td->next_dtd_pointer = USB_TD_NEXT_DTD_POINTER_TERMINATE;
        td->total_bytes
                = USB_TD_DTD_TOKEN_TOTAL_BYTES(buffer->actual_length)
                | USB_TD_DTD_TOKEN_MULTO(0);
        for (unsigned int i = 0; i < 5; i++)
                td->buffer_pointer_page[i] = (uint32_t) &buffer->buffer[i*0x1000];

        // Start transfer
        //FIXME: cc_disable_interrupts();
        if (endpoint_heads[index] == NULL) {
                // There is no transfer currently underway
                buffer->next = NULL;
                usb_endpoint_prime(endpoint, td);
        } else {
                buffer_t* i = endpoint_heads[index];
                while (i->next != NULL)
                        i = i->next;
                i->next = buffer;
                usb_endpoint_append_td(endpoint, td, &i->td);
        }

        buffer->status = PENDING;
        //cc_enable_interrupts();
}
  
void transmit_buffer(buffer_t* buffer,
                     const usb_endpoint_t* const endpoint,
                     unsigned int actual_length)
{
        assert(usb_endpoint_is_in(endpoint->address));
        buffer->actual_length = actual_length;
        start_transfer(buffer, endpoint);
}

void receive_buffer(buffer_t* buffer,
                    const usb_endpoint_t* const endpoint)
{
        assert(!usb_endpoint_is_in(endpoint->address));
        start_transfer(buffer, endpoint);
}

void transfer_complete(const usb_endpoint_t* const endpoint)
{
        const uint_fast8_t index = USB_ENDPOINT_INDEX(endpoint->address);
        buffer_t *buffer = endpoint_heads[index];
        endpoint_heads[index] = buffer->next;
        unsigned int transferred = buffer->actual_length - buffer->td.total_bytes;
        if (buffer->completion_cb)
                buffer->completion_cb(buffer, transferred);
        else if (usb_endpoint_is_in(endpoint->address))
                buffer->status = FREE;
}
