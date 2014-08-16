#pragma once

// 24.8 fixed point
typedef int32_t fixed8_t;
// 16.16 fixed point
typedef int32_t fixed16_t;
// 8.24 fixed point
typedef int32_t fixed24_t;

// A set of samples from the ADC
typedef int16_t adc_frame_t[8];

typedef fixed24_t adc_avg_frame_t[8];
