// app_encoder.c
#include "app_encoder.h"
#include <math.h>
#include <stdio.h>

#define RAD_TO_DEG  57.295779513f

// Calibration parameters (will be updated by auto-calibration)
static float sin_center = 33833.0f;
static float sin_amplitude = 14389.5f;
static float cos_center = 32979.0f;
static float cos_amplitude = 14158.5f;

// Unified amplitude for ellipse correction
static float unified_amplitude = 14389.5f;

// State for mechanical angle tracking across multiple electrical cycles
static int initialized = 0;            // electrical tracking init
static int32_t turn_count = 0;         // mechanical turns
static float last_mech_angle = 0.0f;   // before zero/direction

// Zero and direction configuration
static float zero_offset_deg = 0.0f;  // subtract after direction
static int8_t dir_sign = +1;          // +1 normal, -1 invert

// Phase-unwrapping state
static int prev_vec_initialized = 0;
static float prev_elec_angle = 0.0f;  // use angle difference instead of dot/cross
static float accum_elec_deg = 0.0f;   // cumulative electrical angle (deg)

static inline float normalize_deg(float a)
{
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f) a += 360.0f;
    return a;
}

void encoder_init(void)
{
    initialized = 0;
    turn_count = 0;
    zero_offset_deg = 0.0f;
    dir_sign = +1;
    prev_vec_initialized = 0;
    accum_elec_deg = 0.0f;
    last_mech_angle = 0.0f;
}

void encoder_calibrate(uint16_t sin_min, uint16_t sin_max, 
                       uint16_t cos_min, uint16_t cos_max)
{
    sin_center = (sin_max + sin_min) / 2.0f;
    sin_amplitude = (sin_max - sin_min) / 2.0f;
    cos_center = (cos_max + cos_min) / 2.0f;
    cos_amplitude = (cos_max - cos_min) / 2.0f;
    
    // Use unified amplitude (larger one) to avoid phase distortion
    unified_amplitude = (sin_amplitude > cos_amplitude) ? sin_amplitude : cos_amplitude;
    
    // Reset filters/state to avoid stale values after calibration
    initialized = 0;
    prev_vec_initialized = 0;
    accum_elec_deg = 0.0f;
    turn_count = 0;
    last_mech_angle = 0.0f;

    printf("\nCalibration Applied:\n");
    printf("  SIN: min=%5u max=%5u center=%.1f amp=%.1f\n", 
           sin_min, sin_max, sin_center, sin_amplitude);
    printf("  COS: min=%5u max=%5u center=%.1f amp=%.1f\n",
           cos_min, cos_max, cos_center, cos_amplitude);
    printf("  Unified amplitude: %.1f\n", unified_amplitude);
}

void encoder_set_zero_deg(float zero_deg)
{
    zero_offset_deg = normalize_deg(zero_deg);
}

void encoder_tare_zero(void)
{
    // set zero to last mechanical angle
    zero_offset_deg = normalize_deg(last_mech_angle);
}

void encoder_clear_zero(void)
{
    zero_offset_deg = 0.0f;
}

void encoder_set_direction(int8_t dir)
{
    dir_sign = (dir >= 0) ? +1 : -1;
}

void encoder_process(uint16_t adc_sin, uint16_t adc_cos, encoder_result_t *result)
{
    result->sin_raw = adc_sin;
    result->cos_raw = adc_cos;
    
    // Step 1: Remove DC offset
    float sin_centered = (float)adc_sin - sin_center;
    float cos_centered = (float)adc_cos - cos_center;
    
    // Step 2: Normalize with unified amplitude to avoid phase distortion
    float sin_norm = sin_centered / unified_amplitude;
    float cos_norm = cos_centered / unified_amplitude;
    
    // Clamp to valid range (handle ADC noise)
    if (sin_norm > 1.0f) sin_norm = 1.0f;
    if (sin_norm < -1.0f) sin_norm = -1.0f;
    if (cos_norm > 1.0f) cos_norm = 1.0f;
    if (cos_norm < -1.0f) cos_norm = -1.0f;

    // No filtering, use directly
    float filt_sin = sin_norm;
    float filt_cos = cos_norm;
    
    // Calculate signal magnitude (for diagnostic)
    float mag = sqrtf(filt_sin*filt_sin + filt_cos*filt_cos);
    result->magnitude = mag;
    result->sin_norm = filt_sin;
    result->cos_norm = filt_cos;

    // Gating: if magnitude too low, freeze angle update
    static float prev_mech_angle_out = 0.0f;
    if (mag < ENCODER_MIN_MAG_THRESHOLD) {
        result->elec_angle_deg = 0.0f; // not reliable
        result->angle_deg = prev_mech_angle_out;
        result->turns = turn_count;
        const uint32_t full_scale = 1u << ENCODER_RESOLUTION_BITS;
        uint32_t counts = (uint32_t)(result->angle_deg * (float)full_scale / 360.0f + 0.5f);
        if (counts >= full_scale) counts = 0;
        result->angle_counts = (uint16_t)counts;
        return;
    }
    
    // Instant electrical angle (for diagnostics)
    float elec_angle_deg_inst = atan2f(filt_sin, filt_cos) * RAD_TO_DEG;
    if (elec_angle_deg_inst < 0.0f) elec_angle_deg_inst += 360.0f;
    result->elec_angle_deg = elec_angle_deg_inst;

    // Use angle difference instead of dot/cross to avoid ±180° flip at high speed
    if (!prev_vec_initialized) {
        prev_elec_angle = elec_angle_deg_inst;
        accum_elec_deg = elec_angle_deg_inst; // anchor at current
        prev_vec_initialized = 1;
    } else {
        // Calculate angle increment
        float delta_elec = elec_angle_deg_inst - prev_elec_angle;
        
        // Handle 360°?0° and 0°?360° wraparound
        if (delta_elec > 180.0f) {
            delta_elec -= 360.0f;
        } else if (delta_elec < -180.0f) {
            delta_elec += 360.0f;
        }
        
        accum_elec_deg += delta_elec;
        prev_elec_angle = elec_angle_deg_inst;
    }

    // Mechanical total angle in degrees (can exceed 360 or be negative)
    float mech_total_deg = accum_elec_deg / (float)ENCODER_ELEC_CYCLES_PER_REV;
    
    // Correctly handle negative angles for turn counting
    turn_count = (int32_t)floorf(mech_total_deg / 360.0f);
    
    // Mechanical angle within [0,360)
    float mech_angle_deg = mech_total_deg - (float)(turn_count * 360);
    // Ensure within [0, 360) range
    if (mech_angle_deg < 0.0f) {
        mech_angle_deg += 360.0f;
    }
    
    last_mech_angle = mech_angle_deg;

    // Apply direction first, then subtract zero offset
    float mech_angle_out = (dir_sign > 0) ? mech_angle_deg : (360.0f - mech_angle_deg);
    mech_angle_out = mech_angle_out - zero_offset_deg;
    mech_angle_out = normalize_deg(mech_angle_out);

    result->angle_deg = mech_angle_out;
    result->turns = turn_count;

    // Round to nearest instead of truncation
    const uint32_t full_scale = 1u << ENCODER_RESOLUTION_BITS;
    uint32_t counts = (uint32_t)(mech_angle_out * (float)full_scale / 360.0f + 0.5f);
    if (counts >= full_scale) counts = 0; // Wrap to 0
    result->angle_counts = (uint16_t)counts;

    prev_mech_angle_out = mech_angle_out;
}
