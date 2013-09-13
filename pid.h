#pragma once

#include "types.h"

#define PI_MAX_HISTORY 32

// Proportional-integral feedback loop
struct pi_channel {
        // parameters:
        fixed16_t p_gain, i_gain;
        unsigned int tau;
        // state:
        int32_t history[PI_MAX_HISTORY];
        unsigned int tail;  // index of last filled entry in history
        int32_t sum;  // running sum of history
};

static inline void pi_reset(struct pi_channel* const pi)
{
        for (unsigned int i=0; i<PI_MAX_HISTORY; i++)
                pi->history[i] = 0;
        pi->sum = 0;
        pi->tail = 0;
}

static inline uint8_t pi_get_tau(struct pi_channel* const pi)
{
        return pi->tau;
}

static inline int pi_set_tau(struct pi_channel* const pi, uint8_t tau)
{
        if (tau > PI_MAX_HISTORY)
                return -1;
        pi->tau = tau;
        pi_reset(pi);
        return 0;
}

static inline void pi_add_sample(struct pi_channel* const pi, int32_t s)
{
        if (pi->tau == 0)
                pi->tail = 0;
        else
                pi->tail = (pi->tail + 1) % pi->tau;
        int32_t* tail = &pi->history[pi->tail];
        pi->sum = pi->sum - *tail + s;
        *tail = s;
}

static inline int32_t pi_get_response(const struct pi_channel* const pi)
{
        int32_t resp = 0;
        resp += (pi->p_gain * pi->history[pi->tail]) >> 16;
        resp += (pi->i_gain * pi->sum) >> 16;
        return resp;
}
