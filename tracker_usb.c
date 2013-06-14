#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/gpio.h>

#include <stddef.h>

#include "tracker_usb.h"
#include "usb.h"
#include "usb_type.h"
#include "usb_request.h"
#include "usb_descriptor.h"
#include "usb_standard_request.h"

uint8_t* const usb_bulk_buffer = (uint8_t*)0x20004000;
static volatile uint32_t usb_bulk_buffer_offset = 0;
static const uint32_t usb_bulk_buffer_mask = 32768 - 1;

usb_transfer_descriptor_t usb_td_bulk[2] ATTR_ALIGNED(64);
const uint_fast8_t usb_td_bulk_count = sizeof(usb_td_bulk) / sizeof(usb_td_bulk[0]);

static void usb_init_buffers_bulk() {
	usb_td_bulk[0].next_dtd_pointer = USB_TD_NEXT_DTD_POINTER_TERMINATE;
	usb_td_bulk[0].total_bytes
		= USB_TD_DTD_TOKEN_TOTAL_BYTES(16384)
		| USB_TD_DTD_TOKEN_MULTO(0)
		;
	usb_td_bulk[0].buffer_pointer_page[0] = (uint32_t)&usb_bulk_buffer[0x0000];
	usb_td_bulk[0].buffer_pointer_page[1] = (uint32_t)&usb_bulk_buffer[0x1000];
	usb_td_bulk[0].buffer_pointer_page[2] = (uint32_t)&usb_bulk_buffer[0x2000];
	usb_td_bulk[0].buffer_pointer_page[3] = (uint32_t)&usb_bulk_buffer[0x3000];
	usb_td_bulk[0].buffer_pointer_page[4] = (uint32_t)&usb_bulk_buffer[0x4000];

	usb_td_bulk[1].next_dtd_pointer = USB_TD_NEXT_DTD_POINTER_TERMINATE;
	usb_td_bulk[1].total_bytes
		= USB_TD_DTD_TOKEN_TOTAL_BYTES(16384)
		| USB_TD_DTD_TOKEN_MULTO(0)
		;
	usb_td_bulk[1].buffer_pointer_page[0] = (uint32_t)&usb_bulk_buffer[0x4000];
	usb_td_bulk[1].buffer_pointer_page[1] = (uint32_t)&usb_bulk_buffer[0x5000];
	usb_td_bulk[1].buffer_pointer_page[2] = (uint32_t)&usb_bulk_buffer[0x6000];
	usb_td_bulk[1].buffer_pointer_page[3] = (uint32_t)&usb_bulk_buffer[0x7000];
	usb_td_bulk[1].buffer_pointer_page[4] = (uint32_t)&usb_bulk_buffer[0x8000];
}

void usb_endpoint_schedule_no_int(
	const usb_endpoint_t* const endpoint,
	usb_transfer_descriptor_t* const td
) {
	// Ensure that endpoint is ready to be primed.
	// It may have been flushed due to an aborted transaction.
	// TODO: This should be preceded by a flush?
	while( usb_endpoint_is_ready(endpoint) );

	// Configure a transfer.
	td->total_bytes =
		  USB_TD_DTD_TOKEN_TOTAL_BYTES(16384)
		/*| USB_TD_DTD_TOKEN_IOC*/
		| USB_TD_DTD_TOKEN_MULTO(0)
		| USB_TD_DTD_TOKEN_STATUS_ACTIVE
		;
	
	usb_endpoint_prime(endpoint, td);
}

usb_configuration_t usb_configuration_high_speed = {
	.number = 1,
	.speed = USB_SPEED_HIGH,
	.descriptor = usb_descriptor_configuration_high_speed,
};

usb_configuration_t usb_configuration_full_speed = {
	.number = 1,
	.speed = USB_SPEED_FULL,
	.descriptor = usb_descriptor_configuration_full_speed,
};

usb_configuration_t* usb_configurations[] = {
	&usb_configuration_high_speed,
	&usb_configuration_full_speed,
	0,
};

usb_device_t usb_device = {
	.descriptor = usb_descriptor_device,
	.configurations = &usb_configurations,
	.configuration = 0,
};

usb_endpoint_t usb_endpoint_control_out;
usb_endpoint_t usb_endpoint_control_in;

usb_endpoint_t usb_endpoint_control_out = {
	.address = 0x00,
	.device = &usb_device,
	.in = &usb_endpoint_control_in,
	.out = &usb_endpoint_control_out,
	.setup_complete = usb_setup_complete,
	.transfer_complete = usb_control_out_complete,
};

usb_endpoint_t usb_endpoint_control_in = {
	.address = 0x80,
	.device = &usb_device,
	.in = &usb_endpoint_control_in,
	.out = &usb_endpoint_control_out,
	.setup_complete = 0,
	.transfer_complete = usb_control_in_complete,
};

usb_endpoint_t usb_endpoint_bulk_in = {
	.address = 0x81,
	.device = &usb_device,
	.in = &usb_endpoint_bulk_in,
	.out = 0,
	.setup_complete = 0,
	.transfer_complete = 0,
};

usb_endpoint_t usb_endpoint_bulk_out = {
	.address = 0x02,
	.device = &usb_device,
	.in = 0,
	.out = &usb_endpoint_bulk_out,
	.setup_complete = 0,
	.transfer_complete = 0,
};

static const usb_request_handler_fn vendor_request_handler[] = {
	NULL,
};

static const uint32_t vendor_request_handler_count =
	sizeof(vendor_request_handler) / sizeof(vendor_request_handler[0]);

usb_request_status_t usb_vendor_request(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
	usb_request_status_t status = USB_REQUEST_STATUS_STALL;
	
	if( endpoint->setup.request < vendor_request_handler_count ) {
		usb_request_handler_fn handler = vendor_request_handler[endpoint->setup.request];
		if( handler ) {
			status = handler(endpoint, stage);
		}
	}
	
	return status;
}

const usb_request_handlers_t usb_request_handlers = {
	.standard = usb_standard_request,
	.class = 0,
	.vendor = usb_vendor_request,
	.reserved = 0,
};

void usb_configuration_changed(
	usb_device_t* const device
) {
        if( device->configuration ) {
                gpio_set(PORT_LED1_3, PIN_LED1);
        } else {
                gpio_clear(PORT_LED1_3, PIN_LED1);
	}
};

void usb_init(void)
{
  usb_peripheral_reset();
  usb_device_init(0, &usb_device);
  usb_init_buffers_bulk();
  usb_endpoint_init(&usb_endpoint_control_out);
  usb_endpoint_init(&usb_endpoint_control_in);
  nvic_set_priority(NVIC_USB0_IRQ, 255);
  usb_run(&usb_device);
}
