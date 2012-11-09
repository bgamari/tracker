#pragma once

#include <stdint.h>
#include "scan.h"

struct echo_cmd_t {
    uint8_t length;
    uint8_t data[];
} __attribute__((packed));

struct set_gains_cmd_t {
    signed int feedback_gains[3][3];
} __attribute__((packed));

struct run_scan_cmd_t {
    struct raster_scan_t raster_scan;
} __attribute__((packed));

enum cmd_t {
    CMD_ECHO = 0,
    CMD_RUN_SCAN,
    CMD_SET_GAINS,
    CMD_START_FEEDBACK,
    CMD_STOP_FEEDBACK,
    CMD_SET_FEEDBACK_FREQ,
    CMD_SET_ADC_FREQ,
};

struct cmd_frame_t {
    uint8_t cmd;
    union {
        struct echo_cmd_t echo;
        struct set_gains_cmd_t set_gains;
        struct run_scan_cmd_t run_scan;
        uint32_t set_feedback_freq;
        uint32_t set_adc_freq;
    };
} __attribute__((packed));

