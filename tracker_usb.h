#pragma once

void usb_init(void);

void send_reply(void *data, uint16_t length);

void tracker_usb_send_buffer(void *data, uint16_t length);
