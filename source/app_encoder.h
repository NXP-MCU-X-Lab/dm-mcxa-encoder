// app_encoder.h - Updated
#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdint.h>

// ========== Configuration ==========
// Electrical sin/cos cycles per mechanical revolution
#define ENCODER_ELEC_CYCLES_PER_REV 4

// Output resolution in bits for quantized mechanical angle
#define ENCODER_RESOLUTION_BITS 16

// Signal quality gating threshold [0..1]
#define ENCODER_MIN_MAG_THRESHOLD 0.6f

// ========== Data Structures ==========
typedef struct {
    // Raw ADC values
    uint16_t sin_raw;
    uint16_t cos_raw;
    
    // Normalized signals [-1, 1]
    float sin_norm;
    float cos_norm;
    
    // Signal quality
    float magnitude;            // Signal magnitude [0, ~1.414]
    
    // Angle outputs
    float elec_angle_deg;       // Electrical angle per cycle [0, 360)
    float angle_deg;            // Mechanical angle [0, 360)
    int32_t turns;              // Mechanical turn count
    uint16_t angle_counts;      // Quantized mechanical angle (N-bit)
} encoder_result_t;

// ========== Public API ==========

/**
 * @brief Initialize encoder module
 *        Resets all state variables
 */
void encoder_init(void);

/**
 * @brief Apply calibration parameters from auto-calibration
 * @param sin_min Minimum ADC value for sin channel
 * @param sin_max Maximum ADC value for sin channel
 * @param cos_min Minimum ADC value for cos channel
 * @param cos_max Maximum ADC value for cos channel
 * @note This function resets internal state
 */
void encoder_calibrate(uint16_t sin_min, uint16_t sin_max, 
                       uint16_t cos_min, uint16_t cos_max);

/**
 * @brief Set absolute zero position
 * @param zero_deg Zero position in degrees [0, 360)
 */
void encoder_set_zero_deg(float zero_deg);

/**
 * @brief Set current position as zero (tare function)
 */
void encoder_tare_zero(void);

/**
 * @brief Clear zero offset (reset to 0°)
 */
void encoder_clear_zero(void);

/**
 * @brief Set rotation direction
 * @param dir +1 for normal, -1 for inverted
 */
void encoder_set_direction(int8_t dir);

/**
 * @brief Process ADC samples and calculate angle
 * @param adc_sin Sin channel ADC value
 * @param adc_cos Cos channel ADC value
 * @param result Pointer to result structure
 * @note Call at constant sampling rate (e.g., 1kHz)
 */
void encoder_process(uint16_t adc_sin, uint16_t adc_cos, encoder_result_t *result);

#endif // APP_ENCODER_H
