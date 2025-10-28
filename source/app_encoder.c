// app_encoder.c - Optimized version
#include "app_encoder.h"
#include <math.h>
#include <stdio.h>

#define RAD_TO_DEG  57.295779513f

// ========== Calibration Parameters ==========
static float sin_center = 33833.0f;
static float sin_amplitude = 14389.5f;
static float cos_center = 32979.0f;
static float cos_amplitude = 14158.5f;
static float unified_amplitude = 14389.5f;

// ========== Zero and Direction ==========
static float zero_offset_deg = 0.0f;
static int8_t dir_sign = +1;

// ========== Phase Unwrapping State ==========
typedef struct {
    uint8_t initialized;
    float prev_elec_angle;      // Previous electrical angle
    float accum_elec_deg;       // Accumulated electrical angle
    int32_t turn_count;         // Mechanical turn count
    float prev_output_angle;    // Previous output angle (for filtering and gating)
} encoder_state_t;

static encoder_state_t state = {0};

// ========== Inline Utility Functions ==========
static inline float normalize_deg(float a)
{
    // Optimized: single modulo is faster than while loop
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

static inline float clamp_f(float val, float min, float max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static inline float angle_diff_deg(float a, float b)
{
    // Calculate shortest angle difference [-180, 180)
    float d = a - b;
    if (d > 180.0f) d -= 360.0f;
    else if (d < -180.0f) d += 360.0f;
    return d;
}

// ========== Public Interface ==========
void encoder_init(void)
{
    state.initialized = 0;
    state.prev_elec_angle = 0.0f;
    state.accum_elec_deg = 0.0f;
    state.turn_count = 0;
    state.prev_output_angle = 0.0f;
    zero_offset_deg = 0.0f;
    dir_sign = +1;
}

void encoder_calibrate(uint16_t sin_min, uint16_t sin_max, 
                       uint16_t cos_min, uint16_t cos_max)
{
    sin_center = (sin_max + sin_min) * 0.5f;
    sin_amplitude = (sin_max - sin_min) * 0.5f;
    cos_center = (cos_max + cos_min) * 0.5f;
    cos_amplitude = (cos_max - cos_min) * 0.5f;
    
    // Use larger amplitude to avoid phase distortion
    unified_amplitude = (sin_amplitude > cos_amplitude) ? sin_amplitude : cos_amplitude;
    
    // Reset state after calibration
    encoder_init();

    printf("\nCalibration Applied:\n");
    printf("  SIN: center=%.1f amp=%.1f\n", sin_center, sin_amplitude);
    printf("  COS: center=%.1f amp=%.1f\n", cos_center, cos_amplitude);
    printf("  Unified amp: %.1f\n", unified_amplitude);
}

void encoder_set_zero_deg(float zero_deg)
{
    zero_offset_deg = normalize_deg(zero_deg);
}

void encoder_tare_zero(void)
{
    zero_offset_deg = state.prev_output_angle;
}

void encoder_clear_zero(void)
{
    zero_offset_deg = 0.0f;
}

void encoder_set_direction(int8_t dir)
{
    dir_sign = (dir >= 0) ? +1 : -1;
}

// ========== Core Processing Function ==========
void encoder_process(uint16_t adc_sin, uint16_t adc_cos, encoder_result_t *result)
{
    // ========== Step 1: Normalization ==========
    float sin_norm = ((float)adc_sin - sin_center) / unified_amplitude;
    float cos_norm = ((float)adc_cos - cos_center) / unified_amplitude;
    
    // Soft clipping to handle ADC noise and avoid atan2 anomalies
    sin_norm = clamp_f(sin_norm, -1.0f, 1.0f);
    cos_norm = clamp_f(cos_norm, -1.0f, 1.0f);
    
    // ========== Step 2: Signal Quality Check ==========
    float mag = sqrtf(sin_norm * sin_norm + cos_norm * cos_norm);
    
    // Fill diagnostic information
    result->sin_raw = adc_sin;
    result->cos_raw = adc_cos;
    result->sin_norm = sin_norm;
    result->cos_norm = cos_norm;
    result->magnitude = mag;
    
    // Signal too weak: freeze output
    if (mag < ENCODER_MIN_MAG_THRESHOLD) {
        result->elec_angle_deg = 0.0f;
        result->angle_deg = state.prev_output_angle;
        result->turns = state.turn_count;
        
        const uint32_t full_scale = 1u << ENCODER_RESOLUTION_BITS;
        uint32_t counts = (uint32_t)(result->angle_deg * (float)full_scale / 360.0f + 0.5f);
        result->angle_counts = (uint16_t)(counts & (full_scale - 1));
        return;
    }
    
    // ========== Step 3: Electrical Angle Calculation ==========
    float elec_angle_deg = atan2f(sin_norm, cos_norm) * RAD_TO_DEG;
    if (elec_angle_deg < 0.0f) elec_angle_deg += 360.0f;
    
    result->elec_angle_deg = elec_angle_deg;
    
    // ========== Step 4: Phase Unwrapping (Accumulated Electrical Angle) ==========
    if (!state.initialized) {
        state.prev_elec_angle = elec_angle_deg;
        state.accum_elec_deg = elec_angle_deg;
        state.prev_output_angle = elec_angle_deg / (float)ENCODER_ELEC_CYCLES_PER_REV;
        state.initialized = 1;
    } else {
        // Calculate delta (handle 360° wrap-around)
        float delta_elec = angle_diff_deg(elec_angle_deg, state.prev_elec_angle);
        state.accum_elec_deg += delta_elec * (float)dir_sign;
        state.prev_elec_angle = elec_angle_deg;
    }
    
    // ========== Step 5: Mechanical Angle Calculation ==========
    float mech_total_deg = state.accum_elec_deg / (float)ENCODER_ELEC_CYCLES_PER_REV;
    state.turn_count = (int32_t)floorf(mech_total_deg / 360.0f);
    float mech_angle = mech_total_deg - (float)(state.turn_count * 360);
    if (mech_angle < 0.0f) mech_angle += 360.0f;
    
    // ========== Step 6: Apply Direction and Zero Offset ==========
    mech_angle -= zero_offset_deg;
    mech_angle = normalize_deg(mech_angle);
    
    // ========== Step 7: Velocity-Adaptive Jitter Suppression ==========
    #define BASE_DEADBAND     0.03f   // Base deadband for static
    #define MAX_DEADBAND      0.05f   // Max deadband for low speed
    #define FILTER_ALPHA      0.1f    
    #define VELOCITY_SCALE    0.1f     // Velocity sensitivity

    static float prev_delta = 0.0f;  // Add to encoder_state_t

    float delta = angle_diff_deg(mech_angle, state.prev_output_angle);
    float abs_delta = fabsf(delta);

    // Estimate velocity (simple derivative)
    float velocity = fabsf(delta - prev_delta);
    prev_delta = delta;

    // Adaptive deadband: smaller when moving fast
    float adaptive_deadband = BASE_DEADBAND + (MAX_DEADBAND - BASE_DEADBAND) * expf(-velocity / VELOCITY_SCALE);

    if (abs_delta < adaptive_deadband) {
        // Inside adaptive deadband: hold
        mech_angle = state.prev_output_angle;
    } else {
        // Outside deadband: filter and update
        mech_angle = normalize_deg(state.prev_output_angle + FILTER_ALPHA * delta);
        state.prev_output_angle = mech_angle;
    }
    
    // ========== Step 8: Output ==========
    result->angle_deg = mech_angle;
    result->turns = state.turn_count;
    
    // Quantize to N-bit counts
    const uint32_t full_scale = 1u << ENCODER_RESOLUTION_BITS;
    uint32_t counts = (uint32_t)(mech_angle * (float)full_scale / 360.0f + 0.5f);
    result->angle_counts = counts & (full_scale - 1);
}
