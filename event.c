#include "event.h"

void init_event(struct event_t *event)
{
  event->fired = false;
}

void event_fire(struct event_t *event)
{
  event->fired = true;
}

void event_wait(struct event_t *event)
{
  while (!event->fired)
    __asm__("wfi\n");
  event->fired = false;
}

