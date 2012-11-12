#pragma once

void feedback_init();
void feedback_start();
void feedback_stop();

void feedback_set_loop_freq(unsigned int freq);

extern signed int feedback_gains[STAGE_INPUTS][STAGE_OUTPUTS];

