#include "app_encoder.h"
#include "mau_atan2.h"
#include "app_perf.h"
#include "app_sampler.h"
#include <math.h>
#include <stdio.h>

#define RAD_TO_DEG  57.295779513f

/* Key fixes:
 * 1. Mechanical turn calculation uses floorf() instead of truncation
 * 2. Direction only affects output display, not accumulation
 * 3. Zero offset applied at output stage (display zero)
 * 4. Speed estimation decays when signal is weak
 * 5. Optimized normalize_deg for common cases
 */

// ========== Calibration Parameters ==========
encoder_calibration_t s_calibration = {
    .sin_center = 26346.650,
    .cos_center = 20356.039,
    .transform = {
        {0.000074, -0.000024},
        {0.000027, 0.000082}
    }
};

// ========== Zero and Direction ==========
static float zero_offset_deg = 0.0f;
static int8_t dir_sign = +1;

// ========== Phase Unwrapping State ==========
typedef struct {
    uint8_t initialized;
    float prev_elec_angle;
    float accum_elec_deg;
    int32_t turn_count;
    float filtered_angle;
    float published_angle;
    float omega_est;
    uint16_t published_counts;
    uint8_t bypass_deadband;
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
    // Optimized for common case: small deviations from [0, 360)
    if (a >= 360.0f) {
        if (a < 720.0f) return a - 360.0f;
        a = fmodf(a, 360.0f);
    } else if (a < 0.0f) {
        if (a >= -360.0f) return a + 360.0f;
        a = fmodf(a, 360.0f);
        if (a < 0.0f) a += 360.0f;
    }
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
    state.accum_elec_deg = 0.0;
    state.turn_count = 0;
    state.filtered_angle = 0.0f;
    state.published_angle = 0.0f;
    state.omega_est = 0.0f;
    state.published_counts = 0u;
    state.bypass_deadband = 0;
    zero_offset_deg = 0.0f;
    dir_sign = +1;
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
        // Fallback to identity
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
    float raw_mech = (float)(state.accum_elec_deg / (double)ENCODER_ELEC_CYCLES_PER_REV);
    int32_t t = (int32_t)floorf(raw_mech / 360.0);
    zero_offset_deg = raw_mech - (float)t * 360.0f;
    
    // Force exact zero output
    state.published_angle = 0.0f;
    state.published_counts = 0u;
    state.bypass_deadband = 1;
}

void encoder_clear_zero(void)
{
    zero_offset_deg = 0.0f;
    state.bypass_deadband = 1;
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
    
    float mag_raw = sqrtf(sin_t * sin_t + cos_t * cos_t);
    
    // ========== Step 2: Signal Quality Check ==========
    result->sin_raw = adc_sin;
    result->cos_raw = adc_cos;
    result->magnitude = mag_raw;
    
    float sin_norm = sin_t;
    float cos_norm = cos_t;
    if (mag_raw >= ENCODER_MIN_MAG_THRESHOLD) {
        sin_norm = sin_t / mag_raw;
        cos_norm = cos_t / mag_raw;
    }
    result->sin_norm = sin_norm;
    result->cos_norm = cos_norm;
    
    // Signal too weak: freeze output and decay speed estimate
    if (mag_raw < ENCODER_MIN_MAG_THRESHOLD) {
        state.omega_est *= 0.95f;  // Decay speed estimate
        
        // Hold previous output
        result->elec_angle_deg = state.prev_elec_angle;
        result->angle_deg = state.filtered_angle;
        result->turns = state.turn_count;
        result->angle_counts = state.published_counts;
        result->speed_dps = state.omega_est;
        result->speed_rpm = state.omega_est * (1.0f / 360.0f) * 60.0f;
        return;
    }
    
    // ========== Step 3: Electrical Angle Calculation ==========
    uint32_t c0 = sampler_get_timer_count();
    float elec_angle_deg = mau_atan2f(sin_norm, cos_norm) * RAD_TO_DEG;
    uint32_t c1 = sampler_get_timer_count();
    perf_record_atan2_cycles(c1 - c0);
    if (elec_angle_deg < 0.0f) elec_angle_deg += 360.0f;
    
    result->elec_angle_deg = elec_angle_deg;
    
    // ========== Step 4: Phase Unwrapping (Accumulated Electrical Angle) ==========
    if (!state.initialized) {
        state.prev_elec_angle = elec_angle_deg;
        state.accum_elec_deg = elec_angle_deg;
        state.turn_count = 0;
        state.initialized = 1;
    } else {
        float delta_elec = angle_diff_deg(elec_angle_deg, state.prev_elec_angle);
        state.accum_elec_deg += delta_elec;
        
        // 周期性归一化，避免浮点精度损失
        float elec_per_rev = 360.0f * ENCODER_ELEC_CYCLES_PER_REV;
        if (state.accum_elec_deg >= elec_per_rev) {
            state.accum_elec_deg -= elec_per_rev;
            state.turn_count++;
        } else if (state.accum_elec_deg < 0.0f) {
            state.accum_elec_deg += elec_per_rev;
            state.turn_count--;
        }
        
        state.prev_elec_angle = elec_angle_deg;
    }
    
    // ========== Step 5: Mechanical Angle Calculation (FIXED) ==========
    // Use floorf for correct negative angle handling
    float mech_angle = state.accum_elec_deg / (float)ENCODER_ELEC_CYCLES_PER_REV;
    int32_t turns = state.turn_count;
        
    state.turn_count = turns;
    
    // ========== Step 6: Alpha-Beta Filter for Robust Angle Tracking ==========
    // Prediction
    float theta_pred = normalize_deg(state.filtered_angle + state.omega_est * ENCODER_SAMPLE_PERIOD_S);
    
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
    state.filtered_angle = theta_f;
    
    // ========== Step 7: Apply Direction and Zero Offset (FIXED) ==========
    // Apply direction: invert angle if needed
    float display_angle = theta_f;
    if (dir_sign < 0) {
        display_angle = 360.0f - display_angle;
    }
    
    // Apply zero offset (display zero, doesn't affect turn count)
    display_angle -= zero_offset_deg;
    display_angle = normalize_deg(display_angle);
    
    // ========== Step 8: Angle Deadband Filter ==========
    float final_angle_deg;

    if (state.bypass_deadband) {
        // Bypass deadband for immediate update (e.g., after tare/clear)
        final_angle_deg = display_angle;
        state.published_angle = display_angle;
        state.bypass_deadband = 0;
    } else {
        // Normal deadband filtering
        float angle_diff = angle_diff_deg(display_angle, state.published_angle);
        
        if (fabsf(angle_diff) <= ENCODER_ANGLE_DEADBAND_DEG) {
            final_angle_deg = state.published_angle;
        } else {
            final_angle_deg = display_angle;
            state.published_angle = display_angle;
        }
    }
        
    // ========== Step 9: Quantize with Deadband ==========
    const uint32_t full_scale = 1u << ENCODER_RESOLUTION_BITS;
    uint32_t desired_counts = (uint32_t)(final_angle_deg * (float)full_scale / 360.0f + 0.5f) & (full_scale - 1);
    
    int diff = (int)desired_counts - (int)state.published_counts;
    // Wrap difference to shortest path on ring
    if (diff > (int)(full_scale/2)) diff -= (int)full_scale;
    else if (diff < -(int)(full_scale/2)) diff += (int)full_scale;

    uint16_t final_counts;
    if ((unsigned)abs(diff) <= ENCODER_OUTPUT_DEADBAND_COUNTS) {
        // Inside deadband: hold last published value
        final_counts = state.published_counts;
    } else {
        // Outside deadband: publish new value
        final_counts = (uint16_t)desired_counts;
        state.published_counts = final_counts;
    }

    result->angle_deg = final_angle_deg;
    result->turns = state.turn_count;
    result->angle_counts = final_counts;
    result->speed_dps = state.omega_est;
    result->speed_rpm = state.omega_est * (1.0f / 360.0f) * 60.0f;
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
