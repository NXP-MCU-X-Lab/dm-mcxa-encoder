#include "app_encoder_v2.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>

#define V2_ENCODER_PI (3.14159265358979323846f)
#define V2_ENCODER_ADC_RAIL_LOW (64U)
#define V2_ENCODER_ADC_RAIL_HIGH (65471U)
#define V2_ENCODER_MIN_RAW_AMPLITUDE (256.0f)
#define V2_ENCODER_MIN_NORMALIZED_MAG (0.15f)
#define V2_ENCODER_TRACK_PHASE_TOLERANCE_DEG (45.0f)
#define V2_ENCODER_DEFAULT_RAW_CENTER (32768.0f)
#define V2_ENCODER_DEFAULT_RAW_AMPLITUDE (16384.0f)
#define V2_ENCODER_MIN_ROUGH_AMPLITUDE (64.0f)

typedef struct _v2_encoder_solver_candidate
{
    uint8_t mapping;
    int8_t dir16;
    int8_t dir15;
    float p16;
    float p15;
    float coarse;
    float angle;
    float phase16_error;
    float phase15_error;
    float error;
} v2_encoder_solver_candidate_t;

static bool is_finite_float(float value)
{
    return ((value == value) && (fabsf(value) < FLT_MAX));
}

static float wrap_deg(float deg)
{
    while (deg >= 360.0f)
    {
        deg -= 360.0f;
    }

    while (deg < 0.0f)
    {
        deg += 360.0f;
    }

    return deg;
}

static float signed_angle_error_deg(float actual, float expected)
{
    float diff = wrap_deg(actual - expected);

    if (diff > 180.0f)
    {
        diff -= 360.0f;
    }

    return diff;
}

static float max_float(float a, float b)
{
    return (a > b) ? a : b;
}

static uint32_t angle_to_counts(float angle_deg)
{
    uint32_t counts = (uint32_t)(((wrap_deg(angle_deg) * (float)V2_ENCODER_COUNTS_PER_REV) / 360.0f) + 0.5f);

    if (counts >= V2_ENCODER_COUNTS_PER_REV)
    {
        counts -= V2_ENCODER_COUNTS_PER_REV;
    }

    return counts;
}

static bool adc_is_rail(uint16_t raw)
{
    return ((raw <= V2_ENCODER_ADC_RAIL_LOW) || (raw >= V2_ENCODER_ADC_RAIL_HIGH));
}

static void set_identity_track_cal(v2_encoder_track_calibration_t *track)
{
    if (track == NULL)
    {
        return;
    }

    track->center_sin = V2_ENCODER_DEFAULT_RAW_CENTER;
    track->center_cos = V2_ENCODER_DEFAULT_RAW_CENTER;
    track->transform[0][0] = 1.0f / V2_ENCODER_DEFAULT_RAW_AMPLITUDE;
    track->transform[0][1] = 0.0f;
    track->transform[1][0] = 0.0f;
    track->transform[1][1] = 1.0f / V2_ENCODER_DEFAULT_RAW_AMPLITUDE;
}

static void init_track_stats(v2_encoder_track_cal_stats_t *stats)
{
    stats->count = 0U;
    stats->rail_count = 0U;
    stats->min_sin = UINT16_MAX;
    stats->max_sin = 0U;
    stats->min_cos = UINT16_MAX;
    stats->max_cos = 0U;
    stats->sum_sin = 0.0;
    stats->sum_cos = 0.0;
    stats->sum_sin2 = 0.0;
    stats->sum_cos2 = 0.0;
    stats->sum_sincos = 0.0;
}

static void init_rough_track(v2_encoder_rough_track_state_t *track)
{
    if (track == NULL)
    {
        return;
    }

    track->initialized = false;
    track->min_sin = UINT16_MAX;
    track->max_sin = 0U;
    track->min_cos = UINT16_MAX;
    track->max_cos = 0U;
}

static void update_min_max(uint16_t raw, uint16_t *min_raw, uint16_t *max_raw)
{
    if (raw < *min_raw)
    {
        *min_raw = raw;
    }

    if (raw > *max_raw)
    {
        *max_raw = raw;
    }
}

static void update_rough_track(v2_encoder_rough_track_state_t *track, uint16_t sin_raw, uint16_t cos_raw)
{
    if (track == NULL)
    {
        return;
    }

    if (!track->initialized)
    {
        track->initialized = true;
        track->min_sin = sin_raw;
        track->max_sin = sin_raw;
        track->min_cos = cos_raw;
        track->max_cos = cos_raw;
        return;
    }

    update_min_max(sin_raw, &track->min_sin, &track->max_sin);
    update_min_max(cos_raw, &track->min_cos, &track->max_cos);
}

static void accumulate_track_stats(v2_encoder_track_cal_stats_t *stats, uint16_t sin_raw, uint16_t cos_raw)
{
    if (sin_raw < stats->min_sin)
    {
        stats->min_sin = sin_raw;
    }

    if (sin_raw > stats->max_sin)
    {
        stats->max_sin = sin_raw;
    }

    if (cos_raw < stats->min_cos)
    {
        stats->min_cos = cos_raw;
    }

    if (cos_raw > stats->max_cos)
    {
        stats->max_cos = cos_raw;
    }

    stats->count++;
    if (adc_is_rail(sin_raw) || adc_is_rail(cos_raw))
    {
        stats->rail_count++;
    }

    stats->sum_sin += (double)sin_raw;
    stats->sum_cos += (double)cos_raw;
    stats->sum_sin2 += ((double)sin_raw * (double)sin_raw);
    stats->sum_cos2 += ((double)cos_raw * (double)cos_raw);
    stats->sum_sincos += ((double)sin_raw * (double)cos_raw);
}

static bool build_track_calibration(const v2_encoder_track_cal_stats_t *stats,
                                    v2_encoder_track_calibration_t *track,
                                    uint32_t weak_status,
                                    uint32_t *status)
{
    const float range_sin = 0.5f * (float)(stats->max_sin - stats->min_sin);
    const float range_cos = 0.5f * (float)(stats->max_cos - stats->min_cos);

    if ((stats->count < 64U) || (range_sin < V2_ENCODER_MIN_RAW_AMPLITUDE) ||
        (range_cos < V2_ENCODER_MIN_RAW_AMPLITUDE))
    {
        if (status != NULL)
        {
            *status |= weak_status;
        }
        return false;
    }

    if (stats->rail_count > (stats->count / 16U))
    {
        if (status != NULL)
        {
            *status |= V2_ENCODER_STATUS_ADC_RAIL;
        }
        return false;
    }

    track->center_sin = 0.5f * ((float)stats->min_sin + (float)stats->max_sin);
    track->center_cos = 0.5f * ((float)stats->min_cos + (float)stats->max_cos);
    track->transform[0][0] = 1.0f / range_sin;
    track->transform[0][1] = 0.0f;
    track->transform[1][0] = 0.0f;
    track->transform[1][1] = 1.0f / range_cos;

    if (!is_finite_float(track->transform[0][0]) || !is_finite_float(track->transform[1][1]))
    {
        if (status != NULL)
        {
            *status |= V2_ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    return true;
}

static bool transform_track(const v2_encoder_track_calibration_t *track,
                            uint16_t sin_raw,
                            uint16_t cos_raw,
                            float *phase_deg,
                            float *mag)
{
    const float sin_centered = (float)sin_raw - track->center_sin;
    const float cos_centered = (float)cos_raw - track->center_cos;
    const float sin_corr = (track->transform[0][0] * sin_centered) + (track->transform[0][1] * cos_centered);
    const float cos_corr = (track->transform[1][0] * sin_centered) + (track->transform[1][1] * cos_centered);
    const float local_mag = sqrtf((sin_corr * sin_corr) + (cos_corr * cos_corr));
    const float local_phase = wrap_deg(atan2f(sin_corr, cos_corr) * (180.0f / V2_ENCODER_PI));

    if (!is_finite_float(local_mag) || !is_finite_float(local_phase))
    {
        return false;
    }

    if (phase_deg != NULL)
    {
        *phase_deg = local_phase;
    }

    if (mag != NULL)
    {
        *mag = local_mag;
    }

    return true;
}

static bool transform_rough_track(const v2_encoder_rough_track_state_t *track,
                                  uint16_t sin_raw,
                                  uint16_t cos_raw,
                                  float *phase_deg,
                                  float *mag)
{
    float center_sin = V2_ENCODER_DEFAULT_RAW_CENTER;
    float center_cos = V2_ENCODER_DEFAULT_RAW_CENTER;
    float amp_sin = V2_ENCODER_DEFAULT_RAW_AMPLITUDE;
    float amp_cos = V2_ENCODER_DEFAULT_RAW_AMPLITUDE;
    float sin_corr;
    float cos_corr;
    float local_mag;
    float local_phase;

    if ((track != NULL) && track->initialized)
    {
        center_sin = 0.5f * ((float)track->min_sin + (float)track->max_sin);
        center_cos = 0.5f * ((float)track->min_cos + (float)track->max_cos);
        amp_sin = 0.5f * (float)(track->max_sin - track->min_sin);
        amp_cos = 0.5f * (float)(track->max_cos - track->min_cos);

        if (amp_sin < V2_ENCODER_MIN_ROUGH_AMPLITUDE)
        {
            amp_sin = V2_ENCODER_DEFAULT_RAW_AMPLITUDE;
        }

        if (amp_cos < V2_ENCODER_MIN_ROUGH_AMPLITUDE)
        {
            amp_cos = V2_ENCODER_DEFAULT_RAW_AMPLITUDE;
        }
    }

    sin_corr = ((float)sin_raw - center_sin) / amp_sin;
    cos_corr = ((float)cos_raw - center_cos) / amp_cos;
    local_mag = sqrtf((sin_corr * sin_corr) + (cos_corr * cos_corr));
    local_phase = wrap_deg(atan2f(sin_corr, cos_corr) * (180.0f / V2_ENCODER_PI));

    if (!is_finite_float(local_mag) || !is_finite_float(local_phase))
    {
        return false;
    }

    if (phase_deg != NULL)
    {
        *phase_deg = local_phase;
    }

    if (mag != NULL)
    {
        *mag = local_mag;
    }

    return true;
}

static void hold_last_angle(v2_encoder_state_t *state, v2_encoder_result_t *result)
{
    if ((state != NULL) && state->has_valid_angle)
    {
        result->angle_deg = state->last_angle_deg;
        result->angle_counts = state->last_angle_counts;
    }
    else
    {
        result->angle_deg = 0.0f;
        result->angle_counts = 0U;
    }
}

static void init_diag(v2_encoder_diag_t *diag)
{
    if (diag == NULL)
    {
        return;
    }

    diag->mapping = V2_ENCODER_MAPPING_A1_16_A2_15;
    diag->dir16 = 1;
    diag->dir15 = 1;
    diag->angle_deg = 0.0f;
    diag->coarse_angle_deg = 0.0f;
    diag->phase16_error_deg = 360.0f;
    diag->phase15_error_deg = 360.0f;
    diag->phase_a1_deg = 0.0f;
    diag->phase_a2_deg = 0.0f;
}

static uint32_t a1_weak_status(void)
{
    return (V2_ENCODER_CONFIG_MAPPING == V2_ENCODER_MAPPING_A1_16_A2_15) ?
               V2_ENCODER_STATUS_TRACK16_WEAK :
               V2_ENCODER_STATUS_TRACK15_WEAK;
}

static uint32_t a2_weak_status(void)
{
    return (V2_ENCODER_CONFIG_MAPPING == V2_ENCODER_MAPPING_A1_16_A2_15) ?
               V2_ENCODER_STATUS_TRACK15_WEAK :
               V2_ENCODER_STATUS_TRACK16_WEAK;
}

static bool solve_vernier_variant(float phase_a1,
                                  float phase_a2,
                                  float zero_a1,
                                  float zero_a2,
                                  uint8_t mapping,
                                  int8_t dir16,
                                  int8_t dir15,
                                  v2_encoder_solver_candidate_t *candidate)
{
    float raw16;
    float raw15;
    float zero16;
    float zero15;

    if (candidate == NULL)
    {
        return false;
    }

    if (mapping == V2_ENCODER_MAPPING_A1_16_A2_15)
    {
        raw16 = phase_a1;
        raw15 = phase_a2;
        zero16 = zero_a1;
        zero15 = zero_a2;
    }
    else
    {
        raw16 = phase_a2;
        raw15 = phase_a1;
        zero16 = zero_a2;
        zero15 = zero_a1;
    }

    candidate->mapping = mapping;
    candidate->dir16 = dir16;
    candidate->dir15 = dir15;
    candidate->p16 = wrap_deg((raw16 - zero16) * (float)dir16);
    candidate->p15 = wrap_deg((raw15 - zero15) * (float)dir15);
    candidate->coarse = wrap_deg(candidate->p16 - candidate->p15);

    /* Nonius/Vernier fine refinement.
     * Coarse (p16 - p15) selects which of 16 electrical cycles p16 belongs to;
     * p16 then supplies high-resolution sub-cycle angle. Resolution improves ~16x
     * over raw coarse because p16's atan2 noise is divided by the cycle count. */
    {
        float predicted16 = wrap_deg(candidate->coarse * (float)V2_ENCODER_TRACK16_CYCLES);
        float fine_delta = signed_angle_error_deg(candidate->p16, predicted16);
        candidate->angle = wrap_deg(candidate->coarse + fine_delta / (float)V2_ENCODER_TRACK16_CYCLES);
    }

    /* p16 error is near zero by construction (angle was reconstructed from p16).
     * p15 error is the cross-track consistency check that drives TRACK_MISMATCH. */
    candidate->phase16_error = fabsf(signed_angle_error_deg(
        wrap_deg(candidate->angle * (float)V2_ENCODER_TRACK16_CYCLES), candidate->p16));
    candidate->phase15_error = fabsf(signed_angle_error_deg(
        wrap_deg(candidate->angle * (float)V2_ENCODER_TRACK15_CYCLES), candidate->p15));
    candidate->error = max_float(candidate->phase16_error, candidate->phase15_error);

    return (is_finite_float(candidate->angle) && is_finite_float(candidate->error));
}

static bool solve_best_vernier(float phase_a1,
                               float phase_a2,
                               float zero_a1,
                               float zero_a2,
                               v2_encoder_solver_candidate_t *best)
{
    if (best == NULL)
    {
        return false;
    }

    return solve_vernier_variant(phase_a1,
                                 phase_a2,
                                 zero_a1,
                                 zero_a2,
                                 V2_ENCODER_CONFIG_MAPPING,
                                 V2_ENCODER_CONFIG_DIR16,
                                 V2_ENCODER_CONFIG_DIR15,
                                 best);
}

void v2_encoder_calibration_set_defaults(v2_encoder_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return;
    }

    set_identity_track_cal(&calibration->a1);
    set_identity_track_cal(&calibration->a2);
    calibration->phase_a1_zero_deg = 0.0f;
    calibration->phase_a2_zero_deg = 0.0f;
    calibration->valid = false;
}

void v2_encoder_cal_stats_init(v2_encoder_cal_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    init_track_stats(&stats->a1);
    init_track_stats(&stats->a2);
}

void v2_encoder_cal_stats_accumulate(v2_encoder_cal_stats_t *stats, const v2_encoder_raw_sample_t *sample)
{
    if ((stats == NULL) || (sample == NULL))
    {
        return;
    }

    accumulate_track_stats(&stats->a1, sample->a1_sin_raw, sample->a1_cos_raw);
    accumulate_track_stats(&stats->a2, sample->a2_sin_raw, sample->a2_cos_raw);
}

bool v2_encoder_cal_stats_build(const v2_encoder_cal_stats_t *stats,
                                v2_encoder_calibration_t *calibration,
                                uint32_t *status)
{
    uint32_t local_status = V2_ENCODER_STATUS_OK;
    bool ok;

    if (status != NULL)
    {
        *status = V2_ENCODER_STATUS_OK;
    }

    if ((stats == NULL) || (calibration == NULL))
    {
        if (status != NULL)
        {
            *status = V2_ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    v2_encoder_calibration_set_defaults(calibration);

    ok = build_track_calibration(&stats->a1,
                                 &calibration->a1,
                                 a1_weak_status(),
                                 &local_status);
    ok = build_track_calibration(&stats->a2,
                                 &calibration->a2,
                                 a2_weak_status(),
                                 &local_status) &&
         ok;

    calibration->valid = ok;

    if (status != NULL)
    {
        *status = ok ? V2_ENCODER_STATUS_OK : (local_status | V2_ENCODER_STATUS_CAL_FAILED);
    }

    return ok;
}

bool v2_encoder_capture_zero(v2_encoder_calibration_t *calibration,
                             const v2_encoder_raw_sample_t *sample,
                             uint32_t *status)
{
    float phase_a1;
    float phase_a2;
    float mag_a1;
    float mag_a2;
    uint32_t local_status = V2_ENCODER_STATUS_OK;

    if (status != NULL)
    {
        *status = V2_ENCODER_STATUS_OK;
    }

    if ((calibration == NULL) || (sample == NULL) || !calibration->valid)
    {
        if (status != NULL)
        {
            *status = V2_ENCODER_STATUS_NOT_CALIBRATED | V2_ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    if (adc_is_rail(sample->a1_sin_raw) || adc_is_rail(sample->a1_cos_raw) ||
        adc_is_rail(sample->a2_sin_raw) || adc_is_rail(sample->a2_cos_raw))
    {
        local_status |= V2_ENCODER_STATUS_ADC_RAIL;
    }

    if (!transform_track(&calibration->a1, sample->a1_sin_raw, sample->a1_cos_raw, &phase_a1, &mag_a1))
    {
        local_status |= a1_weak_status() | V2_ENCODER_STATUS_CAL_FAILED;
    }

    if (!transform_track(&calibration->a2, sample->a2_sin_raw, sample->a2_cos_raw, &phase_a2, &mag_a2))
    {
        local_status |= a2_weak_status() | V2_ENCODER_STATUS_CAL_FAILED;
    }

    if (mag_a1 < V2_ENCODER_MIN_NORMALIZED_MAG)
    {
        local_status |= a1_weak_status();
    }

    if (mag_a2 < V2_ENCODER_MIN_NORMALIZED_MAG)
    {
        local_status |= a2_weak_status();
    }

    if (local_status != V2_ENCODER_STATUS_OK)
    {
        if (status != NULL)
        {
            *status = local_status;
        }
        return false;
    }

    calibration->phase_a1_zero_deg = phase_a1;
    calibration->phase_a2_zero_deg = phase_a2;

    return true;
}

void v2_encoder_state_init(v2_encoder_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->last_angle_deg = 0.0f;
    state->last_angle_counts = 0U;
    state->has_valid_angle = false;
    init_rough_track(&state->rough_a1);
    init_rough_track(&state->rough_a2);
}

void v2_encoder_process_with_diag(v2_encoder_state_t *state,
                                  const v2_encoder_calibration_t *calibration,
                                  const v2_encoder_raw_sample_t *sample,
                                  v2_encoder_result_t *result,
                                  v2_encoder_diag_t *diag)
{
    float phase_a1 = 0.0f;
    float phase_a2 = 0.0f;
    float mag_a1 = 0.0f;
    float mag_a2 = 0.0f;
    float zero_a1 = 0.0f;
    float zero_a2 = 0.0f;
    v2_encoder_solver_candidate_t best;
    uint32_t status = V2_ENCODER_STATUS_OK;
    bool have_calibration = false;
    bool publish_angle = false;

    if (result == NULL)
    {
        return;
    }

    result->angle_deg = 0.0f;
    result->angle_counts = 0U;
    result->phase16_deg = 0.0f;
    result->phase15_deg = 0.0f;
    result->coarse_deg = 0.0f;
    result->mag16 = 0.0f;
    result->mag15 = 0.0f;
    result->status = V2_ENCODER_STATUS_OK;
    init_diag(diag);

    if (sample == NULL)
    {
        result->status = V2_ENCODER_STATUS_NOT_CALIBRATED | V2_ENCODER_STATUS_HOLD_LAST;
        hold_last_angle(state, result);
        return;
    }

    have_calibration = ((calibration != NULL) && calibration->valid);
    if (!have_calibration)
    {
        status |= V2_ENCODER_STATUS_NOT_CALIBRATED;
    }

    if (adc_is_rail(sample->a1_sin_raw) || adc_is_rail(sample->a1_cos_raw) ||
        adc_is_rail(sample->a2_sin_raw) || adc_is_rail(sample->a2_cos_raw))
    {
        status |= V2_ENCODER_STATUS_ADC_RAIL;
    }

    if (have_calibration)
    {
        zero_a1 = calibration->phase_a1_zero_deg;
        zero_a2 = calibration->phase_a2_zero_deg;

        if (!transform_track(&calibration->a1,
                             sample->a1_sin_raw,
                             sample->a1_cos_raw,
                             &phase_a1,
                             &mag_a1))
        {
            status |= a1_weak_status() | V2_ENCODER_STATUS_CAL_FAILED;
        }

        if (!transform_track(&calibration->a2,
                             sample->a2_sin_raw,
                             sample->a2_cos_raw,
                             &phase_a2,
                             &mag_a2))
        {
            status |= a2_weak_status() | V2_ENCODER_STATUS_CAL_FAILED;
        }

        if (mag_a1 < V2_ENCODER_MIN_NORMALIZED_MAG)
        {
            status |= a1_weak_status();
        }

        if (mag_a2 < V2_ENCODER_MIN_NORMALIZED_MAG)
        {
            status |= a2_weak_status();
        }
    }
    else
    {
        if (state != NULL)
        {
            update_rough_track(&state->rough_a1, sample->a1_sin_raw, sample->a1_cos_raw);
            update_rough_track(&state->rough_a2, sample->a2_sin_raw, sample->a2_cos_raw);
        }

        if (!transform_rough_track((state != NULL) ? &state->rough_a1 : NULL,
                                   sample->a1_sin_raw,
                                   sample->a1_cos_raw,
                                   &phase_a1,
                                   &mag_a1))
        {
            status |= a1_weak_status() | V2_ENCODER_STATUS_CAL_FAILED;
        }

        if (!transform_rough_track((state != NULL) ? &state->rough_a2 : NULL,
                                   sample->a2_sin_raw,
                                   sample->a2_cos_raw,
                                   &phase_a2,
                                   &mag_a2))
        {
            status |= a2_weak_status() | V2_ENCODER_STATUS_CAL_FAILED;
        }
    }

    if (diag != NULL)
    {
        diag->phase_a1_deg = phase_a1;
        diag->phase_a2_deg = phase_a2;
    }

    if ((status & ~(V2_ENCODER_STATUS_NOT_CALIBRATED)) == V2_ENCODER_STATUS_OK)
    {
        if (solve_best_vernier(phase_a1, phase_a2, zero_a1, zero_a2, &best))
        {
            if (best.mapping == V2_ENCODER_MAPPING_A1_16_A2_15)
            {
                result->mag16 = mag_a1;
                result->mag15 = mag_a2;
            }
            else
            {
                result->mag16 = mag_a2;
                result->mag15 = mag_a1;
            }

            result->phase16_deg = best.p16;
            result->phase15_deg = best.p15;
            result->coarse_deg = best.coarse;

            if (diag != NULL)
            {
                diag->mapping = best.mapping;
                diag->dir16 = best.dir16;
                diag->dir15 = best.dir15;
                diag->angle_deg = best.angle;
                diag->coarse_angle_deg = best.coarse;
                diag->phase16_error_deg = best.phase16_error;
                diag->phase15_error_deg = best.phase15_error;
            }

            if (best.error > V2_ENCODER_TRACK_PHASE_TOLERANCE_DEG)
            {
                status |= V2_ENCODER_STATUS_TRACK_MISMATCH;
            }
        }
        else
        {
            status |= V2_ENCODER_STATUS_CAL_FAILED;
        }
    }

    publish_angle = (((status & ~(V2_ENCODER_STATUS_NOT_CALIBRATED)) == V2_ENCODER_STATUS_OK) &&
                     (have_calibration || (V2_ENCODER_ROUGH_ANGLE_ENABLE != 0U)));

    if (publish_angle)
    {
        result->angle_deg = (diag != NULL) ? diag->angle_deg : best.angle;
        result->angle_counts = angle_to_counts(result->angle_deg);

        if (state != NULL)
        {
            state->last_angle_deg = result->angle_deg;
            state->last_angle_counts = result->angle_counts;
            state->has_valid_angle = true;
        }
    }
    else
    {
        status |= V2_ENCODER_STATUS_HOLD_LAST;
        hold_last_angle(state, result);
    }

    result->status = status;
}

void v2_encoder_process(v2_encoder_state_t *state,
                        const v2_encoder_calibration_t *calibration,
                        const v2_encoder_raw_sample_t *sample,
                        v2_encoder_result_t *result)
{
    v2_encoder_process_with_diag(state, calibration, sample, result, NULL);
}
