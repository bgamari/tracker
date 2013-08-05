#pragma once

#define PATH_MAX_POINTS 80

int enqueue_points(uint16_t* points, unsigned int npts);

void clear_path();

int start_path(unsigned int freq);

bool is_path_running();
