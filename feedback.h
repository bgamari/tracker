#pragma once

#include "tracker.h"
#include "pid.h"
#include "types.h"

enum feedback_mode_t {
    NO_FEEDBACK = 0,
    PSD_FEEDBACK,
    STAGE_FEEDBACK,
};

void feedback_init();
enum feedback_mode_t feedback_get_mode();
void feedback_set_mode(enum feedback_mode_t mode);

void feedback_set_loop_freq(unsigned int freq);

int feedback_set_position(uint16_t setpoint[3]);

extern fixed16_t psd_fb_gains[PSD_INPUTS][STAGE_OUTPUTS];
extern signed int psd_fb_setpoint[PSD_INPUTS];

extern fixed16_t stage_fb_gains[STAGE_INPUTS][STAGE_OUTPUTS];
extern signed int stage_fb_setpoint[STAGE_INPUTS];

extern signed int max_error;
extern struct pi_channel stage_outputs[STAGE_OUTPUTS];

#define MAX_EXCITATION_LENGTH 1024
struct excitation_buffer {
        unsigned int length;
        unsigned int offset;
        int16_t samples[MAX_EXCITATION_LENGTH];
};

extern struct excitation_buffer excitations[STAGE_OUTPUTS];
