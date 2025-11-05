// app_encoder.c - Optimized version
#include "app_encoder.h"
#include "mau_atan2.h"
#include <math.h>
#include <stdio.h>

#define RAD_TO_DEG  57.295779513f

/* Inductive encoder core processing:
 * - Normalize raw ADC sin/cos via calibration
 * - Gate by signal magnitude and compute electrical angle
 * - Unwrap phase and convert to mechanical angle/turns
 * - Apply direction/zero and light jitter suppression
 * - Quantize to N-bit angle for transmission
 */

// ========== Calibration Parameters ==========
static encoder_calibration_t s_calibration = {
    .sin_center = 33486.0f,
    .cos_center = 33783.0f,
    .transform = {
        {0.000063, 0.000019},
        {-0.000020, 0.000068}
    }
};

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
    float prev_delta;           // Previous delta for velocity estimate
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
    static int s_mau_ready = 0;
    if (!s_mau_ready) {
        mau_config_t cfg;
        MAU_GetDefaultConfig(&cfg);
        MAU_Init(MAU0, &cfg);
        s_mau_ready = 1;
    }

    state.initialized = 0;
    state.prev_elec_angle = 0.0f;
    state.accum_elec_deg = 0.0f;
    state.turn_count = 0;
    state.prev_output_angle = 0.0f;
    state.prev_delta = 0.0f;
    zero_offset_deg = 0.0f;
    dir_sign = +1;
}

void encoder_calibrate(uint16_t sin_min, uint16_t sin_max, 
                       uint16_t cos_min, uint16_t cos_max)
{
    float sin_center = (sin_max + sin_min) * 0.5f;
    float sin_amp = (sin_max - sin_min) * 0.5f;
    float cos_center = (cos_max + cos_min) * 0.5f;
    float cos_amp = (cos_max - cos_min) * 0.5f;

    const float min_amp = 100.0f;
    if (sin_amp < min_amp) sin_amp = min_amp;
    if (cos_amp < min_amp) cos_amp = min_amp;

    encoder_calibration_t cal = {
        .sin_center = sin_center,
        .cos_center = cos_center,
        .transform = {
            {1.0f / sin_amp, 0.0f},
            {0.0f, 1.0f / cos_amp}
        }
    };

    encoder_apply_calibration(&cal);

    printf("\nCalibration Applied (Min/Max):\n");
    printf("  SIN: center=%.1f amp=%.1f\n", sin_center, sin_amp);
    printf("  COS: center=%.1f amp=%.1f\n", cos_center, cos_amp);
}

void encoder_apply_calibration(const encoder_calibration_t *cal)
{
    if (!cal) {
        return;
    }

    // Copy and validate the incoming calibration transform; if any non-finite
    // value is detected, fall back to identity matrix instead of zero to avoid
    // degeneracy.
    encoder_calibration_t tmp = *cal;
    int invalid = 0;
    for (int r = 0; r < 2; r++) {
        for (int c = 0; c < 2; c++) {
            if (!isfinite(tmp.transform[r][c])) {
                invalid = 1;
            }
        }
    }
    if (invalid) {
        // Fallback to identity is safer and prevents transform degeneracy
        tmp.transform[0][0] = 1.0f; tmp.transform[0][1] = 0.0f;
        tmp.transform[1][0] = 0.0f; tmp.transform[1][1] = 1.0f;
    }

    s_calibration = tmp;

    encoder_init();

    printf("\nApplied encoder calibration:\n");
    printf("  Center: sin=%.3f cos=%.3f\n", s_calibration.sin_center, s_calibration.cos_center);
    printf("  Transform:\n");
    printf("    [%.6f %.6f]\n", s_calibration.transform[0][0], s_calibration.transform[0][1]);
    printf("    [%.6f %.6f]\n", s_calibration.transform[1][0], s_calibration.transform[1][1]);
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
    float sin_centered = (float)adc_sin - s_calibration.sin_center;
    float cos_centered = (float)adc_cos - s_calibration.cos_center;
    float sin_t = s_calibration.transform[0][0] * sin_centered +
                  s_calibration.transform[0][1] * cos_centered;
    float cos_t = s_calibration.transform[1][0] * sin_centered +
                  s_calibration.transform[1][1] * cos_centered;
    
    // Compute magnitude for quality gating and radial normalization
    float mag_raw = sqrtf(sin_t * sin_t + cos_t * cos_t);
    
    // ========== Step 2: Signal Quality Check ==========
    float mag = mag_raw;
    
    // Fill diagnostic information
    result->sin_raw = adc_sin;
    result->cos_raw = adc_cos;
    // For sufficiently strong signal, apply radial normalization to unit vector
    // (avoids distortion that component clamping would introduce)
    float sin_norm = sin_t;
    float cos_norm = cos_t;
    if (mag_raw >= ENCODER_MIN_MAG_THRESHOLD) {
        sin_norm = sin_t / mag_raw;
        cos_norm = cos_t / mag_raw;
    }
    result->sin_norm = sin_norm;
    result->cos_norm = cos_norm;
    result->magnitude = mag;
    
    // Signal too weak: freeze output
    if (mag < ENCODER_MIN_MAG_THRESHOLD) {
        // Hold electrical and mechanical angles to avoid semantic jumps under weak signal
        result->elec_angle_deg = state.prev_elec_angle;
        result->angle_deg = state.prev_output_angle;
        result->turns = state.turn_count;
        
        const uint32_t full_scale = 1u << ENCODER_RESOLUTION_BITS;
        uint32_t counts = (uint32_t)(result->angle_deg * (float)full_scale / 360.0f + 0.5f);
        result->angle_counts = (uint16_t)(counts & (full_scale - 1));
        return;
    }
    
    // ========== Step 3: Electrical Angle Calculation ==========
    /* Use MAU hardware-accelerated atan2 (float precision) */
    float elec_angle_deg = mau_atan2f(sin_norm, cos_norm) * RAD_TO_DEG;
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
    int32_t turns = (int32_t)(mech_total_deg / 360.0f); // truncate toward zero
    float mech_angle = mech_total_deg - (float)turns * 360.0f;
    if (mech_angle < 0.0f) {
        mech_angle += 360.0f;
    }
    state.turn_count = turns;
    
    // ========== Step 6: Apply Direction and Zero Offset ==========
    mech_angle -= zero_offset_deg;
    mech_angle = normalize_deg(mech_angle);
    
    // ========== Step 7: Velocity-Adaptive Jitter Suppression ==========
    #define BASE_DEADBAND     0.03f   // Base deadband for static
    #define MAX_DEADBAND      0.04f   // Max deadband for low speed
    #define FILTER_ALPHA      0.1f    
    #define VELOCITY_SCALE    0.1f     // Velocity sensitivity

    float delta = angle_diff_deg(mech_angle, state.prev_output_angle);
    float abs_delta = fabsf(delta);

    // Estimate velocity (simple derivative)
    float velocity = fabsf(delta - state.prev_delta);
    state.prev_delta = delta;

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
