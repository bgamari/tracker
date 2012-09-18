#pragma once

#include <stdbool.h>

struct event_t {
  bool fired;
};

void init_event(struct event_t *event);
void event_fire(struct event_t *event);
void event_wait(struct event_t *event);

