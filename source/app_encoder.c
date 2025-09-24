// app_encoder.c
#include "app_encoder.h"
#include <math.h>
#include <stdio.h>

#define RAD_TO_DEG  57.295779513f

// Calibration parameters (will be updated by auto-calibration)
static float sin_center = 44540.0f;
static float sin_amplitude = 14332.5f;
static float cos_center = 45522.0f;
static float cos_amplitude = 13971.5f;


void ENCODER_Init(void)
{
    // Reserved for auto-calibration
}

void ENCODER_Calibrate(uint16_t sin_min, uint16_t sin_max, 
                       uint16_t cos_min, uint16_t cos_max)
{
    sin_center = (sin_max + sin_min) / 2.0f;
    sin_amplitude = (sin_max - sin_min) / 2.0f;
    cos_center = (cos_max + cos_min) / 2.0f;
    cos_amplitude = (cos_max - cos_min) / 2.0f;
    
    printf("\nCalibration Applied:\n");
    printf("  SIN: min=%5u max=%5u center=%.1f amp=%.1f\n", 
           sin_min, sin_max, sin_center, sin_amplitude);
    printf("  COS: min=%5u max=%5u center=%.1f amp=%.1f\n",
           cos_min, cos_max, cos_center, cos_amplitude);
}


void ENCODER_Process(uint16_t adc_sin, uint16_t adc_cos, encoder_result_t *result)
{
    result->sin_raw = adc_sin;
    result->cos_raw = adc_cos;
    
    // Normalize to [-1, 1] with individual scaling
    float sin_norm = ((float)adc_sin - sin_center) / sin_amplitude;
    float cos_norm = ((float)adc_cos - cos_center) / cos_amplitude;
    
    // Clamp to valid range (handle ADC noise)
    if (sin_norm > 1.0f) sin_norm = 1.0f;
    if (sin_norm < -1.0f) sin_norm = -1.0f;
    if (cos_norm > 1.0f) cos_norm = 1.0f;
    if (cos_norm < -1.0f) cos_norm = -1.0f;
    
    // Calculate signal magnitude (for diagnostic)
    result->magnitude = sqrtf(sin_norm*sin_norm + cos_norm*cos_norm);
    result->sin_norm = sin_norm;
    result->cos_norm = cos_norm;
    
    // Calculate angle
    float angle_rad = atan2f(sin_norm, cos_norm);
    
    // Convert to [0, 360]
    result->angle_deg = angle_rad * RAD_TO_DEG;
    if (result->angle_deg < 0.0f) {
        result->angle_deg += 360.0f;
    }
}
