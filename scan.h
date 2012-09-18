#pragma once

#include "adc.h"

struct raster_scan_t {
    uint16_t center_x, center_y;
    uint16_t step_x, step_y;
    unsigned int size_x, size_y;
    unsigned int freq;
};

void raster_scan(struct raster_scan_t *scan);

