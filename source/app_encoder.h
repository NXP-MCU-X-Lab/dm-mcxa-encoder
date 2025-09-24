// app_encoder.h
#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdint.h>

typedef struct {
    uint16_t sin_raw;
    uint16_t cos_raw;
    float angle_deg;     // 0 ~ 360
    float magnitude;     // Signal quality indicator
    float sin_norm;      // Normalized sin (-1 ~ +1)
    float cos_norm;      // Normalized cos (-1 ~ +1)
} encoder_result_t;

void ENCODER_Init(void);
void ENCODER_Calibrate(uint16_t sin_min, uint16_t sin_max, 
                       uint16_t cos_min, uint16_t cos_max);
void ENCODER_Process(uint16_t adc_sin, uint16_t adc_cos, encoder_result_t *result);

#endif
