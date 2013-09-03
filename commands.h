#pragma once

#include <stdint.h>

#include "feedback.h"
#include "adc.h"
#include "path.h"

struct echo_cmd {
    uint8_t length;
    uint8_t data[];
} __attribute__((packed));

#define RESET_MAGIC 0xdeadbeef

struct enqueue_points {
    uint8_t npts;
    uint16_t points[80][3];
} __attribute__((packed));

struct start_path {
    uint32_t freq;
    uint8_t synchronous_adc;
} __attribute__((packed));

struct set_excitation {
    uint8_t channel;
    uint8_t length;
    uint16_t samples[];
} __attribute__((packed));

enum cmd_t {
    CMD_ECHO                  = 0x0,
    CMD_RESET                 = 0x1,

    CMD_SET_STAGE_GAINS       = 0x10,
    CMD_SET_STAGE_SETPOINT    = 0x11,

    CMD_SET_PSD_GAINS         = 0x12,
    CMD_SET_PSD_SETPOINT      = 0x13,

    CMD_SET_MAX_ERROR         = 0x14,
    CMD_SET_OUTPUT_GAINS      = 0x15,
    CMD_SET_EXCITATION        = 0x16,

    CMD_SET_ADC_FREQ          = 0x20,
    CMD_SET_ADC_TRIGGER_MODE  = 0x21,
    CMD_START_ADC_STREAM      = 0x22,
    CMD_STOP_ADC_STREAM       = 0x23,

    CMD_SET_FEEDBACK_FREQ     = 0x30,
    CMD_SET_FEEDBACK_MODE     = 0x31,
    CMD_SET_RAW_POS           = 0x32,

    CMD_CLEAR_PATH            = 0x40,
    CMD_ENQUEUE_POINTS        = 0x41,
    CMD_START_PATH            = 0x42,
};

struct cmd_frame_t {
    uint8_t cmd;
    union {
        struct echo_cmd echo;
        uint32_t reset_magic;
        int32_t set_stage_gains[3][3];
        int32_t set_stage_setpoint[3];
        int32_t set_psd_gains[4][3];
        int32_t set_psd_setpoint[4];
        uint32_t set_max_error;
        int32_t set_output_gains[3];
        struct set_excitation set_excitation;
        uint32_t set_feedback_freq;  // In Hz
        uint32_t set_adc_freq;
        enum trigger_mode set_adc_trigger_mode;
        enum feedback_mode_t set_feedback_mode;
        uint16_t set_raw_pos[3];
        struct enqueue_points enqueue_points;
        struct start_path start_path;
    };
} __attribute__((packed));

void process_cmd(struct cmd_frame_t *cmd);
