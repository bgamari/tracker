#pragma once

// 24.8 fixed point
typedef int32_t fixed8_t;
// 16.16 fixed point
typedef int32_t fixed16_t;
// 8.24 fixed point
typedef int32_t fixed24_t;

// A set of samples from the ADC
//   frame[0] = stage x
//   frame[1] = stage y
//   frame[2] = stage z
//   frame[3] = sum x
//   frame[4] = diff x
//   frame[5] = sum y
//   frame[6] = diff y
//   frame[7] = unused
typedef int16_t adc_frame_t[8];

typedef fixed24_t adc_avg_frame_t[8];
