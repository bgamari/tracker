#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/gpio.h>

#include <stddef.h>

#include "tracker_usb.h"
#include "commands.h"
#include "hackrf_usb/usb.h"
#include "hackrf_usb/usb_type.h"
#include "hackrf_usb/usb_queue.h"
#include "hackrf_usb/usb_request.h"
#include "hackrf_usb/usb_descriptor.h"
#include "hackrf_usb/usb_standard_request.h"

bool adc_streaming = false;
struct cmd_frame_t cmd_frame;

uint8_t command_buffer[512] = "hello world";

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
USB_DEFINE_QUEUE(usb_endpoint_control_out, 4);

usb_endpoint_t usb_endpoint_control_in = {
	.address = 0x80,
	.device = &usb_device,
	.in = &usb_endpoint_control_in,
	.out = &usb_endpoint_control_out,
	.setup_complete = 0,
	.transfer_complete = usb_control_in_complete,
};
USB_DEFINE_QUEUE(usb_endpoint_control_in, 4);

usb_endpoint_t usb_endpoint_bulk_cmd_in = {
	.address = 0x81,
	.device = &usb_device,
	.in = &usb_endpoint_bulk_cmd_in,
	.out = 0,
	.setup_complete = 0,
	.transfer_complete = usb_queue_transfer_complete,
};
USB_DEFINE_QUEUE(usb_endpoint_bulk_cmd_in, 4);

usb_endpoint_t usb_endpoint_bulk_cmd_out = {
	.address = 0x02,
	.device = &usb_device,
	.in = 0,
	.out = &usb_endpoint_bulk_cmd_out,
	.setup_complete = 0,
	.transfer_complete = usb_queue_transfer_complete,
};
USB_DEFINE_QUEUE(usb_endpoint_bulk_cmd_out, 4);

usb_endpoint_t usb_endpoint_bulk_data_in = {
	.address = 0x83,
	.device = &usb_device,
	.in = &usb_endpoint_bulk_data_in,
	.out = 0,
	.setup_complete = 0,
	.transfer_complete = usb_queue_transfer_complete,
};
USB_DEFINE_QUEUE(usb_endpoint_bulk_data_in, 4);

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

static void start_command_transfer(void);

static void command_transfer_completed(
        void* user_data,
        unsigned int transferred
) {
        process_cmd((struct cmd_frame_t*) command_buffer);
        start_command_transfer();
}

static void start_command_transfer()
{
        usb_transfer_schedule_block(&usb_endpoint_bulk_cmd_out,
                                    command_buffer,
                                    sizeof(command_buffer),
                                    command_transfer_completed, NULL);
}

void send_reply(void *data, uint16_t length)
{
        usb_transfer_schedule_block(&usb_endpoint_bulk_cmd_in, data, length,
                                    NULL, NULL);
}

static void buffer_sent(void* user_data, unsigned int transferred)
{
        buffer_t* buffer = buffer_from_pointer(user_data);
        put_buffer(buffer);
}

void tracker_usb_send_buffer(buffer_t* buffer)
{
        int ret = usb_transfer_schedule(&usb_endpoint_bulk_data_in,
                                        buffer->data, BUFFER_SIZE,
                                        buffer_sent, buffer);
                                        
        if (ret == 0) {
                // success
                return;
        } if (ret == -1) {
                // No transfers available
                put_buffer(buffer);
        } else {
                while (1);
        }
}

void usb_configuration_changed(
	usb_device_t* const device
) {
        if (device->configuration == 0) {
            usb_endpoint_disable(&usb_endpoint_bulk_cmd_in);
            usb_endpoint_disable(&usb_endpoint_bulk_cmd_out);
            usb_endpoint_disable(&usb_endpoint_bulk_data_in);
        } else {
            usb_endpoint_init(&usb_endpoint_bulk_cmd_out);
            usb_endpoint_init(&usb_endpoint_bulk_cmd_in);
            usb_endpoint_init(&usb_endpoint_bulk_data_in);
            start_command_transfer();
        }
}

void usb_init(void)
{
        usb_queue_init(&usb_endpoint_control_in_queue);
        usb_queue_init(&usb_endpoint_control_out_queue);
        usb_queue_init(&usb_endpoint_bulk_cmd_in_queue);
        usb_queue_init(&usb_endpoint_bulk_cmd_out_queue);
        usb_queue_init(&usb_endpoint_bulk_data_in_queue);

        usb_set_configuration_changed_cb(usb_configuration_changed);
        usb_peripheral_reset();
        usb_device_init(0, &usb_device);
        usb_endpoint_init(&usb_endpoint_control_out);
        usb_endpoint_init(&usb_endpoint_control_in);
        nvic_set_priority(NVIC_USB0_IRQ, 255);
        usb_run(&usb_device);
}
