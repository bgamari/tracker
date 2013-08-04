#pragma once

#include "adc.h"

struct raster_scan_t {
    uint16_t center_x, center_y;
    uint16_t step_x, step_y;
    uint16_t size_x, size_y;
    uint16_t freq;
};

void raster_scan(struct raster_scan_t *scan);

