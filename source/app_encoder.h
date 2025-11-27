// app_encoder.h - Fixed version
#ifndef APP_ENCODER_H
#define APP_ENCODER_H

/* Encoder processing API for inductive sensor signals.
 * Defines configuration macros, result/calibration structures,
 * and functions to initialize, calibrate, and compute angles.
 */

#include <stdint.h>

// ========== Configuration ==========
// Electrical sin/cos cycles per mechanical revolution
#define ENCODER_ELEC_CYCLES_PER_REV 4

// Output resolution in bits for quantized mechanical angle
#define ENCODER_RESOLUTION_BITS 16

// Signal quality gating threshold [0..1]
// Below this magnitude, output is frozen and radial normalization is skipped.
#define ENCODER_MIN_MAG_THRESHOLD 0.6f

// Sampling period (seconds) of encoder_process calls (adjust to your system)
#define ENCODER_SAMPLE_PERIOD_S 0.0001f

// Simple alpha-beta filter parameters for angle tracking
#define ENCODER_AB_ALPHA 0.01f
#define ENCODER_AB_BETA  (ENCODER_AB_ALPHA * ENCODER_AB_ALPHA / 2.0f)

// Max allowed mechanical angular speed for rate limiting (deg/s)
#define ENCODER_OMEGA_MAX_DPS 7200.0f

// Spike clamp threshold on angle innovation (deg)
#define ENCODER_SPIKE_THRESHOLD_DEG 20.0f

// Output deadband in counts (N-bit resolution); small changes are held
#define ENCODER_OUTPUT_DEADBAND_COUNTS 2

// Angle deadband in degrees; small angle changes are held
#define ENCODER_ANGLE_DEADBAND_DEG 0.1f  // ~0.1° deadband

// ========== Data Structures ==========
typedef struct {
    // Raw ADC values
    uint16_t sin_raw;
    uint16_t cos_raw;
    
    // Normalized signals [-1, 1]
    float sin_norm;
    float cos_norm;
    
    // Signal quality
    float magnitude;            // Transformed magnitude before radial normalization (non-negative)
    
    // Angle outputs
    float elec_angle_deg;       // Electrical angle per cycle [0, 360)
    float angle_deg;            // Mechanical angle [0, 360) after zero/direction
    int32_t turns;              // Mechanical turn count
    uint16_t angle_counts;      // Quantized mechanical angle (N-bit)
    float speed_dps;            // Mechanical angular speed (deg/s)
    float speed_rpm;            // Mechanical speed (RPM)
} encoder_result_t;

typedef struct {
    float sin_center;           // DC offset for sin channel
    float cos_center;           // DC offset for cos channel
    float transform[2][2];      // 2x2 gain matrix mapping centered raw -> normalized sin/cos
} encoder_calibration_t;

// ========== Public API ==========

/**
 * @brief Initialize encoder module.
 *        Resets all internal state variables (phase, turns, filters, zero, direction).
 */
void encoder_init(void);

/**
 * @brief Compute and apply min/max-based calibration.
 * @param sin_min Minimum ADC value for sin channel
 * @param sin_max Maximum ADC value for sin channel
 * @param cos_min Minimum ADC value for cos channel
 * @param cos_max Maximum ADC value for cos channel
 * @note Internally builds a diagonal 2x2 transform from amplitudes and calls encoder_apply_calibration().
 *       Resets internal state.
 */
void encoder_calibrate(uint16_t sin_min, uint16_t sin_max, 
                       uint16_t cos_min, uint16_t cos_max);

/**
 * @brief Apply full calibration parameters (offsets + 2x2 transform).
 * @param cal Pointer to calibration structure.
 * @note Validates the transform; on invalid values, falls back to identity matrix.
 *       Resets internal state.
 */
void encoder_apply_calibration(const encoder_calibration_t *cal);

/**
 * @brief Set absolute zero position.
 * @param zero_deg Zero position in degrees; internally normalized to [0, 360).
 * @note This is a display zero - does not affect turn counting.
 */
void encoder_set_zero_deg(float zero_deg);

/**
 * @brief Set current mechanical position as zero (tare function).
 * @note Uses last output mechanical angle.
 */
void encoder_tare_zero(void);

/**
 * @brief Clear zero offset (reset to 0 deg).
 */
void encoder_clear_zero(void);

/**
 * @brief Set rotation direction.
 * @param dir +1 for normal, -1 for inverted (non-negative treated as +1).
 */
void encoder_set_direction(int8_t dir);

/**
 * @brief Process ADC samples and calculate angles.
 * @param adc_sin Sin channel ADC value.
 * @param adc_cos Cos channel ADC value.
 * @param result Pointer to result structure.
 * @note Uses magnitude gating; below threshold, output is held. Expects a constant sampling rate
 *       for stable adaptive filtering (e.g., 1 kHz).
 */
void encoder_process(uint16_t adc_sin, uint16_t adc_cos, encoder_result_t *result);

/*
 * Industrial support helpers
 * - ABS: single-turn counts (N-bit, per ENCODER_RESOLUTION_BITS)
 * - ABM: multi-turn 24-bit counter (wraps at 2^24), with reset/tare support
 * - ENID: encoder ID (1 byte)
 * - SF/ALMC: status/alarm bytes (stubs; integrate with product diagnostics as needed)
 */
uint16_t encoder_get_abs_counts(void);
uint32_t encoder_get_abm_counts24(void);
void     encoder_reset_abm(void);

void     encoder_set_id(uint8_t id);
uint8_t  encoder_get_id(void);
uint8_t  encoder_get_status(void);
uint8_t  encoder_get_alarm(void);

extern encoder_calibration_t s_calibration;

#endif // APP_ENCODER_H
