#pragma once

#include <stdint.h>

#include "scan.h"
#include "feedback.h"

struct echo_cmd_t {
    uint8_t length;
    uint8_t data[];
} __attribute__((packed));

struct run_scan_cmd_t {
    struct raster_scan_t raster_scan;
} __attribute__((packed));

enum cmd_t {
    CMD_ECHO                  = 0x0,
    CMD_RUN_SCAN              = 0x2,

    CMD_SET_STAGE_GAINS       = 0x10,
    CMD_SET_STAGE_SETPOINT    = 0x11,

    CMD_SET_PSD_GAINS         = 0x12,
    CMD_SET_PSD_SETPOINT      = 0x13,

    CMD_SET_MAX_ERROR         = 0x14,
    CMD_SET_OUTPUT_GAINS      = 0x15,

    CMD_SET_ADC_FREQ          = 0x16,

    CMD_SET_FEEDBACK_FREQ     = 0x17,
    CMD_SET_FEEDBACK_MODE     = 0x18,
};

struct cmd_frame_t {
    uint8_t cmd;
    union {
        struct echo_cmd_t echo;
        struct echo_cmd_t spi_echo;
        signed int set_stage_gains[3][3];
        signed int set_stage_setpoint[3];
        signed int set_psd_gains[4][3];
        signed int set_psd_setpoint[3];
        unsigned int set_max_error;
        signed int set_output_gains[3];
        struct run_scan_cmd_t run_scan;
        uint32_t set_feedback_freq;
        uint32_t set_adc_freq;
        enum feedback_mode_t set_feedback_mode;
    };
} __attribute__((packed));

void process_cmd(struct cmd_frame_t *cmd);
