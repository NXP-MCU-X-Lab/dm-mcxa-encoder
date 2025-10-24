// app_encoder.h
#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdint.h>

// Configuration: electrical sin/cos cycles per mechanical revolution
#define ENCODER_ELEC_CYCLES_PER_REV 4
// Output resolution in bits for quantized mechanical angle
#define ENCODER_RESOLUTION_BITS 14
// Filter and gating configuration
#define ENCODER_MIN_MAG_THRESHOLD 0.6f      // Magnitude gating threshold [0..1]

typedef struct {
    uint16_t sin_raw;
    uint16_t cos_raw;
    float angle_deg;     // Mechanical angle [0, 360)
    float elec_angle_deg;// Electrical angle per cycle [0, 360)
    int32_t turns;       // Mechanical turn count
    uint16_t angle_counts;// Quantized mechanical angle (N-bit)
    float magnitude;     // Signal quality indicator
    float sin_norm;      // Normalized sin (-1 ~ +1)
    float cos_norm;      // Normalized cos (-1 ~ +1)
} encoder_result_t;

void encoder_init(void);
void encoder_calibrate(uint16_t sin_min, uint16_t sin_max, 
                       uint16_t cos_min, uint16_t cos_max);
void encoder_process(uint16_t adc_sin, uint16_t adc_cos, encoder_result_t *result);

// Zero/Direction configuration (additive APIs; existing external interfaces unchanged)
void encoder_set_zero_deg(float zero_deg);   // Set zero offset (deg)
void encoder_tare_zero(void);               // Set zero to current mechanical angle
void encoder_clear_zero(void);              // Clear zero offset
void encoder_set_direction(int8_t dir);     // +1 normal, -1 invert

#endif
