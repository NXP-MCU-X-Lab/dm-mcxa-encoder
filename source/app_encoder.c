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
    float omega_est;            // Estimated mechanical angular speed (deg/s)
    float published_angle_deg;  // Last published (deadband-filtered) angle
    uint16_t published_counts;  // Last published quantized counts
} encoder_state_t;

static encoder_state_t state = {0};

/* Industrial support: ID/Status/Alarm and ABM offset */
static uint8_t s_encoder_id = 0x17;      /* default ENID */
static uint8_t s_status     = 0x00;      /* SF (stub; integrate diagnostics) */
static uint8_t s_alarm      = 0x00;      /* ALMC (stub; integrate diagnostics) */
static int32_t s_abm_offset = 0;         /* ABM tare offset (turns) */

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
    state.omega_est = 0.0f;
    state.published_angle_deg = 0.0f;
    state.published_counts = 0u;
    zero_offset_deg = 0.0f;
    dir_sign = +1;
    /* Optional: keep ABM offset across init; set to 0 for deterministic startup */
    s_abm_offset = 0;
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
    
    // ========== Step 7: Simple Alpha-Beta Filter for Robust Angle Tracking ==========
    // Prediction
    float theta_pred = normalize_deg(state.prev_output_angle + state.omega_est * ENCODER_SAMPLE_PERIOD_S);
    // Innovation (wrapped)
    float e_raw = angle_diff_deg(mech_angle, theta_pred);
    // Clamp spikes
    float e = clamp_f(e_raw, -ENCODER_SPIKE_THRESHOLD_DEG, ENCODER_SPIKE_THRESHOLD_DEG);
    // Update omega with rate limiting
    float omega_new = state.omega_est + (ENCODER_AB_BETA / ENCODER_SAMPLE_PERIOD_S) * e;
    omega_new = clamp_f(omega_new, -ENCODER_OMEGA_MAX_DPS, ENCODER_OMEGA_MAX_DPS);
    state.omega_est = omega_new;
    // Update filtered angle
    float theta_f = normalize_deg(theta_pred + ENCODER_AB_ALPHA * e);
    state.prev_delta = e; // keep a lightweight record of last innovation
    state.prev_output_angle = theta_f;

    // ========== Step 8: Output with simple deadband freeze (counts space) ==========
    const uint32_t full_scale = 1u << ENCODER_RESOLUTION_BITS;
    uint32_t desired_counts = (uint32_t)(state.prev_output_angle * (float)full_scale / 360.0f + 0.5f) & (full_scale - 1);
    int diff = (int)desired_counts - (int)state.published_counts;
    // Wrap difference to shortest path on ring
    if (diff > (int)(full_scale/2)) diff -= (int)full_scale;
    else if (diff < -(int)(full_scale/2)) diff += (int)full_scale;

    uint16_t final_counts;
    float final_angle_deg;
    if ((unsigned)abs(diff) <= ENCODER_OUTPUT_DEADBAND_COUNTS) {
        // Inside deadband: hold last published values
        final_counts = state.published_counts;
        final_angle_deg = state.published_angle_deg;
    } else {
        // Outside deadband: publish new filtered angle
        final_counts = (uint16_t)desired_counts;
        final_angle_deg = state.prev_output_angle;
        state.published_counts = final_counts;
        state.published_angle_deg = final_angle_deg;
    }

    result->angle_deg = final_angle_deg;
    result->turns = state.turn_count;
    result->angle_counts = final_counts;
}

/* ===== Industrial support helpers ===== */
uint16_t encoder_get_abs_counts(void)
{
    return state.published_counts;
}

uint32_t encoder_get_abm_counts24(void)
{
    /* Wrap to 24-bit */
    int32_t abm = state.turn_count + s_abm_offset;
    uint32_t u = (uint32_t)abm & 0xFFFFFFU;
    return u;
}

void encoder_reset_abm(void)
{
    /* Tare ABM to zero at current turn count */
    s_abm_offset = -state.turn_count;
}

void encoder_set_id(uint8_t id)
{
    s_encoder_id = id;
}

uint8_t encoder_get_id(void)
{
    return s_encoder_id;
}

uint8_t encoder_get_status(void)
{
    return s_status;
}

uint8_t encoder_get_alarm(void)
{
    return s_alarm;
}
