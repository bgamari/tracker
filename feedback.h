#pragma once

#include "tracker.h"

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

// 16.16 fixed point
typedef int32_t fixed16_t;

extern fixed16_t psd_fb_gains[PSD_INPUTS][STAGE_OUTPUTS];
extern signed int psd_fb_setpoint[STAGE_OUTPUTS];

extern fixed16_t stage_fb_gains[STAGE_INPUTS][STAGE_OUTPUTS];
extern signed int stage_fb_setpoint[STAGE_OUTPUTS];

extern signed int max_error;
extern fixed16_t output_gains[STAGE_OUTPUTS];

struct excitation_buffer {
  unsigned int length;
  unsigned int offset;
  int16_t* samples;
};

extern struct excitation_buffer excitations[STAGE_OUTPUTS];
