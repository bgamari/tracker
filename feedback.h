#pragma once

enum feedback_mode_t {
    NO_FEEDBACK = 0,
    PSD_FEEDBACK,
    STAGE_FEEDBACK,
};

void feedback_init();
enum feedback_mode_t feedback_get_mode();
void feedback_set_mode(enum feedback_mode_t mode);

void feedback_set_loop_freq(unsigned int freq);

extern signed int psd_fb_gains[PSD_INPUTS][STAGE_OUTPUTS];
extern signed int psd_fb_setpoint[STAGE_OUTPUTS];

extern signed int stage_fb_gains[STAGE_INPUTS][STAGE_OUTPUTS];
extern signed int stage_fb_setpoint[STAGE_OUTPUTS];

extern signed int max_error;
extern signed int output_gains[STAGE_OUTPUTS];

