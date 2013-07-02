#pragma once

#include <libopencm3/lpc43xx/gpio.h>

struct pin_t {
    uint32_t port;
    uint16_t pin;
};

static inline void pin_setup_output(struct pin_t *pin)
{
    GPIO_DIR(pin->port) |= pin->pin; 
}

static inline void pin_setup_input(struct pin_t *pin)
{
    GPIO_DIR(pin->port) &= ~pin->pin; 
}

static inline void pin_off(struct pin_t *pin)
{
    gpio_clear(pin->port, pin->pin);
}

static inline void pin_on(struct pin_t *pin)
{
    gpio_set(pin->port, pin->pin);
}

static inline void pin_set(struct pin_t *pin, bool on)
{
    if (on)
        pin_on(pin);
    else
        pin_off(pin);
}

