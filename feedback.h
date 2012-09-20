#pragma once

#include "libmanyuc.h"

void feedback_init();
void feedback_start();
void feedback_stop();

#define SENSOR_INPUTS 4
#define STAGE_INPUTS 3
#define STAGE_OUTPUTS 3

extern signed int feedback_gains[STAGE_INPUTS][STAGE_OUTPUTS];

