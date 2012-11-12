#pragma once

enum dac_channel_t {
    channel_a = 0, channel_b, channel_c, channel_d,
    channel_e, channel_f, channel_g, channel_h,
    broadcast = 0xf,
};

struct dac_update_t {
    enum dac_channel_t channel;
    uint16_t value;
};

void dac_init();
void set_dac(unsigned int n, struct dac_update_t *updates);

