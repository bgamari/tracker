#pragma once

#include "tracker.h"
#include "pid.h"
#include "types.h"

enum feedback_mode_t {
        NO_FEEDBACK = 0, // open loop
        PSD_FEEDBACK,    // linear feedback on PSD input
        STAGE_FEEDBACK,  // linear feedback on stage input
        SEARCH_FEEDBACK, // maximize PSD sum signal
        COARSE_FEEDBACK, // "if above setpoint step left, if below step right"
};

void feedback_init();
enum feedback_mode_t feedback_get_mode();
void feedback_set_mode(enum feedback_mode_t mode);

unsigned int feedback_get_loop_freq();
void feedback_set_loop_freq(unsigned int freq);

int feedback_set_position(uint16_t setpoint[3]);

// PSD feedback parameters
extern fixed16_t psd_fb_gains[PSD_INPUTS][STAGE_OUTPUTS];
extern signed int psd_fb_setpoint[PSD_INPUTS];

// stage feedback parameters
extern fixed16_t stage_fb_gains[STAGE_INPUTS][STAGE_OUTPUTS];
extern signed int stage_fb_setpoint[STAGE_INPUTS];

// search feedback parameters
extern fixed16_t search_obj_gains[PSD_INPUTS];
extern uint16_t search_fb_step[STAGE_OUTPUTS];
extern uint16_t search_obj_thresh;

// coarse feedback parameters
struct coarse_fb_channel {
        int16_t step_high[STAGE_OUTPUTS];
        int16_t step_low[STAGE_OUTPUTS];
        uint16_t tol;
};

extern struct coarse_fb_channel coarse_fb_channels[PSD_INPUTS];

// other feedback
extern signed int max_error;
extern struct pi_channel stage_outputs[STAGE_OUTPUTS];

// on-the-fly excitation
#define MAX_EXCITATION_LENGTH 1024
struct excitation_buffer {
        unsigned int length;
        unsigned int offset;
        int16_t samples[MAX_EXCITATION_LENGTH];
};

extern struct excitation_buffer excitations[STAGE_OUTPUTS];
