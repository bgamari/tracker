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
        uint16_t total_length; // total length of excitation
        uint16_t offset;  // offset within excitation buffer to write this packet at
        uint8_t length;  // number of samples provided in this packet
        uint16_t samples[];
} __attribute__((packed));

enum cmd_t {
        CMD_ECHO                  = 0x0,
        CMD_RESET                 = 0x1,
        CMD_GET_EVENT_COUNTERS    = 0x2,

        CMD_GET_STAGE_GAINS       = 0x10,
        CMD_SET_STAGE_GAINS       = 0x11,
        CMD_GET_STAGE_SETPOINT    = 0x12,
        CMD_SET_STAGE_SETPOINT    = 0x13,

        CMD_GET_PSD_GAINS         = 0x14,
        CMD_SET_PSD_GAINS         = 0x15,
        CMD_GET_PSD_SETPOINT      = 0x16,
        CMD_SET_PSD_SETPOINT      = 0x17,

        CMD_GET_MAX_ERROR         = 0x18,
        CMD_SET_MAX_ERROR         = 0x19,
        CMD_GET_OUTPUT_GAINS      = 0x1a,
        CMD_SET_OUTPUT_GAINS      = 0x1b,
        CMD_GET_OUTPUT_TAUS       = 0x1c,
        CMD_SET_OUTPUT_TAUS       = 0x1d,
        CMD_SET_EXCITATION        = 0x1e,

        CMD_GET_ADC_FREQ          = 0x20,
        CMD_SET_ADC_FREQ          = 0x21,
        CMD_GET_ADC_TRIGGER_MODE  = 0x22,
        CMD_SET_ADC_TRIGGER_MODE  = 0x23,
        CMD_GET_ADC_DECIMATION    = 0x24,
        CMD_SET_ADC_DECIMATION    = 0x25,

        CMD_START_ADC_STREAM      = 0x2a,
        CMD_STOP_ADC_STREAM       = 0x2b,
        CMD_FLUSH_ADC_STREAM      = 0x2c,

        CMD_GET_FEEDBACK_FREQ     = 0x30,
        CMD_SET_FEEDBACK_FREQ     = 0x31,
        CMD_GET_FEEDBACK_MODE     = 0x32,
        CMD_SET_FEEDBACK_MODE     = 0x33,
        CMD_SET_RAW_POS           = 0x34,

        CMD_CLEAR_PATH            = 0x40,
        CMD_ENQUEUE_POINTS        = 0x41,
        CMD_START_PATH            = 0x42,

        CMD_GET_SEARCH_STEP       = 0x43,
        CMD_SET_SEARCH_STEP       = 0x44,
        CMD_GET_SEARCH_OBJ_GAINS  = 0x45,
        CMD_SET_SEARCH_OBJ_GAINS  = 0x46,
        CMD_GET_SEARCH_OBJ_THRESH = 0x47,
        CMD_SET_SEARCH_OBJ_THRESH = 0x48,

        CMD_GET_COARSE_FB_PARAMS  = 0X49,
        CMD_SET_COARSE_FB_PARAMS  = 0X4a,
};

struct cmd_frame_t {
        uint8_t cmd;
        union {
                struct echo_cmd echo;
                uint32_t reset_magic;
                fixed16_t set_stage_gains[3][3];
                int32_t set_stage_setpoint[3];
                fixed16_t set_psd_gains[4][3];
                int32_t set_psd_setpoint[4];
                uint32_t set_max_error;
                fixed16_t set_output_gains[STAGE_OUTPUTS][2];
                uint8_t set_output_taus[STAGE_OUTPUTS];
                struct set_excitation set_excitation;
                uint32_t set_feedback_freq;  // in Hz
                uint32_t set_adc_freq;  // in Hz
                uint32_t set_adc_trigger_mode; // enum trigger_mode
                uint32_t set_adc_decimation;
                uint32_t set_feedback_mode; // enum feedback_mode
                uint16_t set_raw_pos[3];
                struct enqueue_points enqueue_points;
                struct start_path start_path;
                uint16_t set_search_step[3];
                fixed16_t set_search_obj_gains[PSD_INPUTS];
                uint16_t set_search_obj_thresh;
                struct coarse_fb_channel set_coarse_fb_params[3];
        };
} __attribute__((packed));

void process_cmd(struct cmd_frame_t *cmd);
