#pragma once

#define PATH_MAX_POINTS 80

int enqueue_points(uint16_t* points, unsigned int npts);

void clear_path();

// Note that synchronous_trigger==true will likely break feedback horribly
int start_path(unsigned int freq, bool synchronous_trigger);

bool is_path_running();
