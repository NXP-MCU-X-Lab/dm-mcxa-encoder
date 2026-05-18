#include "app_encoder.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>

#define ENCODER_PI (3.14159265358979323846f)
#define ENCODER_ADC_RAIL_LOW (64U)
#define ENCODER_ADC_RAIL_HIGH (65471U)
#define ENCODER_MIN_RAW_AMPLITUDE (256.0f)
#define ENCODER_MIN_NORMALIZED_MAG (0.15f)
#define ENCODER_TRACK_PHASE_TOLERANCE_DEG (45.0f)
#define ENCODER_DEFAULT_RAW_CENTER (32768.0f)
#define ENCODER_DEFAULT_RAW_AMPLITUDE (16384.0f)
#define ENCODER_FINE_BRANCH_STEP_DEG (360.0f / (float)ENCODER_TRACK16_CYCLES)
#define ENCODER_BRANCH_SLIP_STEP_DEG (0.5f * ENCODER_FINE_BRANCH_STEP_DEG)
#define ENCODER_BRANCH_ERROR_MARGIN_DEG (5.0f)
#define ENCODER_RUNTIME_TRIM_MAG_MIN (0.75f)
#define ENCODER_RUNTIME_TRIM_MAG_MAX (1.25f)
#define ENCODER_RUNTIME_TRIM_BIN_COUNT (16U)
#define ENCODER_RUNTIME_TRIM_FULL_COVERAGE_MASK (0xFFFFUL)
#define ENCODER_RUNTIME_TRIM_MIN_WINDOW_SAMPLES (512U)
#define ENCODER_RUNTIME_TRIM_ALPHA (0.125f)

typedef struct _encoder_solver_candidate
{
    float p16;
    float p15;
    float coarse;
    float angle;
    float phase16_error;
    float phase15_error;
    float error;
} encoder_solver_candidate_t;

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
    uint32_t counts = (uint32_t)(((wrap_deg(angle_deg) * (float)ENCODER_COUNTS_PER_REV) / 360.0f) + 0.5f);

    if (counts >= ENCODER_COUNTS_PER_REV)
    {
        counts -= ENCODER_COUNTS_PER_REV;
    }

    return counts;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

static float filter_published_angle(encoder_state_t *state, float raw_angle_deg)
{
    const float alpha = ENCODER_FILTER_ALPHA;
    float filtered;

    if (state == NULL)
    {
        return wrap_deg(raw_angle_deg);
    }

    if (!state->filter_initialized)
    {
        state->filtered_angle_deg = wrap_deg(raw_angle_deg);
        state->filter_initialized = true;
        return state->filtered_angle_deg;
    }

    filtered = wrap_deg(state->filtered_angle_deg +
                        (signed_angle_error_deg(raw_angle_deg, state->filtered_angle_deg) * alpha));
    state->filtered_angle_deg = filtered;
    return filtered;
}

static bool adc_is_rail(uint16_t raw)
{
    return ((raw <= ENCODER_ADC_RAIL_LOW) || (raw >= ENCODER_ADC_RAIL_HIGH));
}

static void set_identity_track_cal(encoder_track_calibration_t *track)
{
    if (track == NULL)
    {
        return;
    }

    track->center_sin = ENCODER_DEFAULT_RAW_CENTER;
    track->center_cos = ENCODER_DEFAULT_RAW_CENTER;
    track->t00 = 1.0f / ENCODER_DEFAULT_RAW_AMPLITUDE;
    track->t10 = 0.0f;
    track->t11 = 1.0f / ENCODER_DEFAULT_RAW_AMPLITUDE;
}

static void init_track_stats(encoder_track_cal_stats_t *stats)
{
    *stats = (encoder_track_cal_stats_t){0};
    stats->min_sin = UINT16_MAX;
    stats->min_cos = UINT16_MAX;
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

static void reset_trim_window(encoder_runtime_trim_t *trim)
{
    if (trim == NULL)
    {
        return;
    }

    trim->window_count = 0U;
    trim->coverage_mask = 0U;
    trim->a1_sin_min = UINT16_MAX;
    trim->a1_sin_max = 0U;
    trim->a1_cos_min = UINT16_MAX;
    trim->a1_cos_max = 0U;
    trim->a2_sin_min = UINT16_MAX;
    trim->a2_sin_max = 0U;
    trim->a2_cos_min = UINT16_MAX;
    trim->a2_cos_max = 0U;
}

static float update_trim_delta(float current_delta, float desired_delta)
{
    float step = (desired_delta - current_delta) * ENCODER_RUNTIME_TRIM_ALPHA;

    step = clamp_float(step,
                       -ENCODER_RUNTIME_TRIM_STEP_LIMIT_COUNTS,
                       ENCODER_RUNTIME_TRIM_STEP_LIMIT_COUNTS);

    return clamp_float(current_delta + step,
                       -ENCODER_RUNTIME_TRIM_TOTAL_LIMIT_COUNTS,
                       ENCODER_RUNTIME_TRIM_TOTAL_LIMIT_COUNTS);
}

static void accumulate_track_stats(encoder_track_cal_stats_t *stats, uint16_t sin_raw, uint16_t cos_raw)
{
    const double sin_value = (double)sin_raw;
    const double cos_value = (double)cos_raw;
    const double sin2 = sin_value * sin_value;
    const double cos2 = cos_value * cos_value;
    const double sincos = sin_value * cos_value;

    update_min_max(sin_raw, &stats->min_sin, &stats->max_sin);
    update_min_max(cos_raw, &stats->min_cos, &stats->max_cos);

    stats->count++;
    if (adc_is_rail(sin_raw) || adc_is_rail(cos_raw))
    {
        stats->rail_count++;
    }

    stats->sum_sin += sin_value;
    stats->sum_cos += cos_value;
    stats->sum_sin2 += sin2;
    stats->sum_cos2 += cos2;
    stats->sum_sincos += sincos;
    stats->sum_sin3 += sin2 * sin_value;
    stats->sum_cos3 += cos2 * cos_value;
    stats->sum_sin2cos += sin2 * cos_value;
    stats->sum_sincos2 += sin_value * cos2;
    stats->sum_sin4 += sin2 * sin2;
    stats->sum_cos4 += cos2 * cos2;
    stats->sum_sin3cos += sin2 * sincos;
    stats->sum_sincos3 += sincos * cos2;
    stats->sum_sin2cos2 += sin2 * cos2;
}

/* Gauss elimination with partial pivoting, 3x3 only. */
static bool solve_3x3(double matrix[3][3], double vector[3], double result[3])
{
    double aug[3][4];
    uint32_t row;
    uint32_t col;

    for (row = 0U; row < 3U; row++)
    {
        for (col = 0U; col < 3U; col++)
        {
            aug[row][col] = matrix[row][col];
        }
        aug[row][3] = vector[row];
    }

    for (col = 0U; col < 3U; col++)
    {
        uint32_t pivot = col;
        double pivot_abs = fabs(aug[col][col]);

        for (row = col + 1U; row < 3U; row++)
        {
            const double value_abs = fabs(aug[row][col]);
            if (value_abs > pivot_abs)
            {
                pivot = row;
                pivot_abs = value_abs;
            }
        }

        if (pivot_abs < 1.0e-30)
        {
            return false;
        }

        if (pivot != col)
        {
            for (row = col; row < 4U; row++)
            {
                const double tmp = aug[col][row];
                aug[col][row] = aug[pivot][row];
                aug[pivot][row] = tmp;
            }
        }

        {
            const double div = aug[col][col];
            for (row = col; row < 4U; row++)
            {
                aug[col][row] /= div;
            }
        }

        for (row = 0U; row < 3U; row++)
        {
            if (row != col)
            {
                const double factor = aug[row][col];
                uint32_t k;
                for (k = col; k < 4U; k++)
                {
                    aug[row][k] -= factor * aug[col][k];
                }
            }
        }
    }

    result[0] = aug[0][3];
    result[1] = aug[1][3];
    result[2] = aug[2][3];

    return true;
}

/* Heydemann-style ellipse correction:
 *   center is taken from min/max (density-robust)
 *   shape is an algebraic LSQ fit of a*x^2 + b*xy + c*y^2 = 1 on centered samples.
 *   Algebraic residual is zero for any sample on the true ellipse, so uneven sample
 *   density along the rotor angle does not bias the fit. */
static bool build_track_calibration(const encoder_track_cal_stats_t *stats,
                                    encoder_track_calibration_t *track,
                                    uint32_t weak_status,
                                    uint32_t *status)
{
    const float range_sin = 0.5f * (float)(stats->max_sin - stats->min_sin);
    const float range_cos = 0.5f * (float)(stats->max_cos - stats->min_cos);

    if ((stats->count < 64U) || (range_sin < ENCODER_MIN_RAW_AMPLITUDE) ||
        (range_cos < ENCODER_MIN_RAW_AMPLITUDE))
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
            *status |= ENCODER_STATUS_ADC_RAIL;
        }
        return false;
    }

    const float center_sin = 0.5f * ((float)stats->min_sin + (float)stats->max_sin);
    const float center_cos = 0.5f * ((float)stats->min_cos + (float)stats->max_cos);
    const double n = (double)stats->count;
    const double cx = (double)center_sin;
    const double cy = (double)center_cos;
    const double cx2 = cx * cx;
    const double cy2 = cy * cy;
    const double cx3 = cx2 * cx;
    const double cy3 = cy2 * cy;
    const double cx4 = cx2 * cx2;
    const double cy4 = cy2 * cy2;

    const double sx2 = stats->sum_sin2 - (2.0 * cx * stats->sum_sin) + (n * cx2);
    const double sy2 = stats->sum_cos2 - (2.0 * cy * stats->sum_cos) + (n * cy2);
    const double sxy = stats->sum_sincos - (cx * stats->sum_cos) - (cy * stats->sum_sin) + (n * cx * cy);
    const double sx4 = stats->sum_sin4 - (4.0 * cx * stats->sum_sin3) +
                       (6.0 * cx2 * stats->sum_sin2) - (4.0 * cx3 * stats->sum_sin) + (n * cx4);
    const double sy4 = stats->sum_cos4 - (4.0 * cy * stats->sum_cos3) +
                       (6.0 * cy2 * stats->sum_cos2) - (4.0 * cy3 * stats->sum_cos) + (n * cy4);
    const double sx3y = stats->sum_sin3cos - (cy * stats->sum_sin3) -
                        (3.0 * cx * stats->sum_sin2cos) +
                        (3.0 * cx * cy * stats->sum_sin2) +
                        (3.0 * cx2 * stats->sum_sincos) -
                        (3.0 * cx2 * cy * stats->sum_sin) -
                        (cx3 * stats->sum_cos) + (n * cx3 * cy);
    const double sxy3 = stats->sum_sincos3 - (cx * stats->sum_cos3) -
                        (3.0 * cy * stats->sum_sincos2) +
                        (3.0 * cx * cy * stats->sum_cos2) +
                        (3.0 * cy2 * stats->sum_sincos) -
                        (3.0 * cx * cy2 * stats->sum_cos) -
                        (cy3 * stats->sum_sin) + (n * cx * cy3);
    const double sx2y2 = stats->sum_sin2cos2 -
                         (2.0 * cy * stats->sum_sin2cos) +
                         (cy2 * stats->sum_sin2) -
                         (2.0 * cx * stats->sum_sincos2) +
                         (4.0 * cx * cy * stats->sum_sincos) -
                         (2.0 * cx * cy2 * stats->sum_sin) +
                         (cx2 * stats->sum_cos2) -
                         (2.0 * cx2 * cy * stats->sum_cos) +
                         (n * cx2 * cy2);
    double matrix[3][3];
    double vector[3];
    double conic[3];
    double m00;
    double m01;
    double m11;
    double det_m;
    double t00_sq;
    double t00;
    double t10;
    double t11;

    matrix[0][0] = sx4;
    matrix[0][1] = sx3y;
    matrix[0][2] = sx2y2;
    matrix[1][0] = sx3y;
    matrix[1][1] = sx2y2;
    matrix[1][2] = sxy3;
    matrix[2][0] = sx2y2;
    matrix[2][1] = sxy3;
    matrix[2][2] = sy4;
    vector[0] = sx2;
    vector[1] = sxy;
    vector[2] = sy2;

    if (!solve_3x3(matrix, vector, conic))
    {
        if (status != NULL)
        {
            *status |= ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    m00 = conic[0];
    m01 = 0.5 * conic[1];
    m11 = conic[2];
    det_m = (m00 * m11) - (m01 * m01);

    if ((m00 <= 0.0) || (m11 <= 0.0) || (det_m <= 0.0))
    {
        if (status != NULL)
        {
            *status |= ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    /* Lower-triangular Cholesky: T^T*T = [[m00, m01], [m01, m11]]. */
    t11 = sqrt(m11);
    t10 = m01 / t11;
    t00_sq = m00 - (t10 * t10);

    if (t00_sq <= 0.0)
    {
        if (status != NULL)
        {
            *status |= ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    t00 = sqrt(t00_sq);

    if (!is_finite_float((float)t00) || !is_finite_float((float)t10) || !is_finite_float((float)t11))
    {
        if (status != NULL)
        {
            *status |= ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    track->center_sin = center_sin;
    track->center_cos = center_cos;
    track->t00 = (float)t00;
    track->t10 = (float)t10;
    track->t11 = (float)t11;

    return true;
}

static bool transform_track(const encoder_track_calibration_t *track,
                            uint16_t sin_raw,
                            uint16_t cos_raw,
                            float *phase_deg,
                            float *mag)
{
    const float sin_centered = (float)sin_raw - track->center_sin;
    const float cos_centered = (float)cos_raw - track->center_cos;
    const float sin_corr = track->t00 * sin_centered;
    const float cos_corr = (track->t10 * sin_centered) + (track->t11 * cos_centered);
    const float local_mag = sqrtf((sin_corr * sin_corr) + (cos_corr * cos_corr));
    const float local_phase = wrap_deg(atan2f(sin_corr, cos_corr) * (180.0f / ENCODER_PI));

    if (phase_deg != NULL)
    {
        *phase_deg = 0.0f;
    }
    if (mag != NULL)
    {
        *mag = 0.0f;
    }

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

static void hold_last_angle(encoder_state_t *state, encoder_result_t *result)
{
    if ((state != NULL) && state->has_valid_angle)
    {
        result->angle_deg = state->last_angle_deg;
        result->angle_deg_raw = state->last_angle_raw_deg;
        result->angle_deg_filtered = state->last_angle_deg;
        result->angle_counts = state->last_angle_counts;
    }
    else
    {
        result->angle_deg = 0.0f;
        result->angle_deg_raw = 0.0f;
        result->angle_deg_filtered = 0.0f;
        result->angle_counts = 0U;
    }
}

static void init_diag(encoder_diag_t *diag)
{
    if (diag == NULL)
    {
        return;
    }

    diag->angle_deg = 0.0f;
    diag->coarse_angle_deg = 0.0f;
    diag->phase16_error_deg = 360.0f;
    diag->phase15_error_deg = 360.0f;
    diag->phase_a1_deg = 0.0f;
    diag->phase_a2_deg = 0.0f;
}

static void update_candidate_error(encoder_solver_candidate_t *candidate)
{
    if (candidate == NULL)
    {
        return;
    }

    candidate->phase16_error = fabsf(signed_angle_error_deg(
        wrap_deg(candidate->angle * (float)ENCODER_TRACK16_CYCLES), candidate->p16));
    candidate->phase15_error = fabsf(signed_angle_error_deg(
        wrap_deg(candidate->angle * (float)ENCODER_TRACK15_CYCLES), candidate->p15));
    candidate->error = max_float(candidate->phase16_error, candidate->phase15_error);
}

/* Vernier branch-slip guard: if the angle jumped by more than half a fine cycle
 * in a single sample, search ±8 fine branches and pick the one closest to the
 * previous angle. Prevents single-frame ADC glitches from flipping branches. */
static void stabilize_vernier_branch(const encoder_state_t *state, encoder_solver_candidate_t *candidate)
{
    encoder_solver_candidate_t adjusted;
    float current_step;
    float best_distance;
    int32_t branch;

    if ((state == NULL) || !state->has_valid_angle || (candidate == NULL))
    {
        return;
    }

    current_step = fabsf(signed_angle_error_deg(candidate->angle, state->last_angle_deg));
    if (current_step <= ENCODER_BRANCH_SLIP_STEP_DEG)
    {
        return;
    }

    adjusted = *candidate;
    best_distance = current_step;

    for (branch = -8; branch <= 8; branch++)
    {
        encoder_solver_candidate_t trial = *candidate;
        const float trial_distance =
            fabsf(signed_angle_error_deg(wrap_deg(candidate->angle +
                                                  ((float)branch * ENCODER_FINE_BRANCH_STEP_DEG)),
                                         state->last_angle_deg));

        if (trial_distance >= best_distance)
        {
            continue;
        }

        trial.angle = wrap_deg(candidate->angle + ((float)branch * ENCODER_FINE_BRANCH_STEP_DEG));
        update_candidate_error(&trial);
        if ((trial.error <= ENCODER_TRACK_PHASE_TOLERANCE_DEG) &&
            (trial.error <= (candidate->error + ENCODER_BRANCH_ERROR_MARGIN_DEG)))
        {
            adjusted = trial;
            best_distance = trial_distance;
        }
    }

    *candidate = adjusted;
}

/* Vernier solve: 16-cycle track gives high resolution, 15-cycle track resolves
 * which of the 16 electrical periods we are in. coarse = wrap(p16 - p15)
 * picks the cycle; p16 supplies sub-cycle fine angle. */
static bool solve_vernier(float phase_a1,
                          float phase_a2,
                          float zero_a1,
                          float zero_a2,
                          encoder_solver_candidate_t *candidate)
{
    float predicted16;
    float fine_delta;

    if (candidate == NULL)
    {
        return false;
    }

    candidate->p16 = wrap_deg(phase_a1 - zero_a1);
    candidate->p15 = wrap_deg(phase_a2 - zero_a2);
    candidate->coarse = wrap_deg(candidate->p16 - candidate->p15);

    predicted16 = wrap_deg(candidate->coarse * (float)ENCODER_TRACK16_CYCLES);
    fine_delta = signed_angle_error_deg(candidate->p16, predicted16);
    candidate->angle = wrap_deg(candidate->coarse + fine_delta / (float)ENCODER_TRACK16_CYCLES);

    update_candidate_error(candidate);

    return (is_finite_float(candidate->angle) && is_finite_float(candidate->error));
}

void encoder_calibration_set_defaults(encoder_calibration_t *calibration)
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

void encoder_cal_stats_init(encoder_cal_stats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    init_track_stats(&stats->a1);
    init_track_stats(&stats->a2);
}

void encoder_cal_stats_accumulate(encoder_cal_stats_t *stats, const encoder_raw_sample_t *sample)
{
    if ((stats == NULL) || (sample == NULL))
    {
        return;
    }

    accumulate_track_stats(&stats->a1, sample->a1_sin_raw, sample->a1_cos_raw);
    accumulate_track_stats(&stats->a2, sample->a2_sin_raw, sample->a2_cos_raw);
}

bool encoder_cal_stats_build(const encoder_cal_stats_t *stats,
                             encoder_calibration_t *calibration,
                             uint32_t *status)
{
    uint32_t local_status = ENCODER_STATUS_OK;
    bool ok;

    if (status != NULL)
    {
        *status = ENCODER_STATUS_OK;
    }

    if ((stats == NULL) || (calibration == NULL))
    {
        if (status != NULL)
        {
            *status = ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    encoder_calibration_set_defaults(calibration);

    ok = build_track_calibration(&stats->a1, &calibration->a1,
                                 ENCODER_STATUS_TRACK16_WEAK, &local_status);
    ok = build_track_calibration(&stats->a2, &calibration->a2,
                                 ENCODER_STATUS_TRACK15_WEAK, &local_status) &&
         ok;

    calibration->valid = ok;

    if (status != NULL)
    {
        *status = ok ? ENCODER_STATUS_OK : (local_status | ENCODER_STATUS_CAL_FAILED);
    }

    return ok;
}

bool encoder_capture_zero(encoder_calibration_t *calibration,
                          const encoder_raw_sample_t *sample,
                          uint32_t *status)
{
    float phase_a1 = 0.0f;
    float phase_a2 = 0.0f;
    float mag_a1 = 0.0f;
    float mag_a2 = 0.0f;
    uint32_t local_status = ENCODER_STATUS_OK;

    if (status != NULL)
    {
        *status = ENCODER_STATUS_OK;
    }

    if ((calibration == NULL) || (sample == NULL) || !calibration->valid)
    {
        if (status != NULL)
        {
            *status = ENCODER_STATUS_NOT_CALIBRATED | ENCODER_STATUS_CAL_FAILED;
        }
        return false;
    }

    if (adc_is_rail(sample->a1_sin_raw) || adc_is_rail(sample->a1_cos_raw) ||
        adc_is_rail(sample->a2_sin_raw) || adc_is_rail(sample->a2_cos_raw))
    {
        local_status |= ENCODER_STATUS_ADC_RAIL;
    }

    if (!transform_track(&calibration->a1, sample->a1_sin_raw, sample->a1_cos_raw,
                         &phase_a1, &mag_a1))
    {
        local_status |= ENCODER_STATUS_TRACK16_WEAK | ENCODER_STATUS_CAL_FAILED;
    }

    if (!transform_track(&calibration->a2, sample->a2_sin_raw, sample->a2_cos_raw,
                         &phase_a2, &mag_a2))
    {
        local_status |= ENCODER_STATUS_TRACK15_WEAK | ENCODER_STATUS_CAL_FAILED;
    }

    if (mag_a1 < ENCODER_MIN_NORMALIZED_MAG)
    {
        local_status |= ENCODER_STATUS_TRACK16_WEAK;
    }

    if (mag_a2 < ENCODER_MIN_NORMALIZED_MAG)
    {
        local_status |= ENCODER_STATUS_TRACK15_WEAK;
    }

    if (local_status != ENCODER_STATUS_OK)
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

void encoder_state_init(encoder_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    *state = (encoder_state_t){0};
}

void encoder_runtime_trim_init(encoder_runtime_trim_t *trim)
{
    if (trim == NULL)
    {
        return;
    }

    *trim = (encoder_runtime_trim_t){0};
    trim->enabled = true;
    trim->freeze_reason = ENCODER_RUNTIME_TRIM_FREEZE_COVERAGE;
    reset_trim_window(trim);
}

void encoder_runtime_trim_reset(encoder_runtime_trim_t *trim)
{
    bool enabled;

    if (trim == NULL)
    {
        return;
    }

    enabled = trim->enabled;
    encoder_runtime_trim_init(trim);
    trim->enabled = enabled;
}

void encoder_runtime_trim_apply(const encoder_calibration_t *factory,
                                const encoder_runtime_trim_t *trim,
                                encoder_calibration_t *effective)
{
    if ((factory == NULL) || (effective == NULL))
    {
        return;
    }

    *effective = *factory;
    if ((trim == NULL) || !trim->enabled)
    {
        return;
    }

    effective->a1.center_sin += trim->a1_center_sin_delta;
    effective->a1.center_cos += trim->a1_center_cos_delta;
    effective->a2.center_sin += trim->a2_center_sin_delta;
    effective->a2.center_cos += trim->a2_center_cos_delta;
}

void encoder_runtime_trim_update(encoder_runtime_trim_t *trim,
                                 const encoder_calibration_t *factory,
                                 const encoder_raw_sample_t *sample,
                                 const encoder_result_t *result)
{
    uint32_t bin;
    float a1_sin_center;
    float a1_cos_center;
    float a2_sin_center;
    float a2_cos_center;

    if ((trim == NULL) || (sample == NULL) || (result == NULL))
    {
        return;
    }

    if (!trim->enabled)
    {
        trim->active = false;
        trim->freeze_reason = ENCODER_RUNTIME_TRIM_FREEZE_DISABLED;
        reset_trim_window(trim);
        return;
    }

    if ((factory == NULL) || !factory->valid)
    {
        trim->active = false;
        trim->freeze_reason = ENCODER_RUNTIME_TRIM_FREEZE_NOT_CALIBRATED;
        reset_trim_window(trim);
        return;
    }

    if (result->status != ENCODER_STATUS_OK)
    {
        trim->active = false;
        trim->freeze_reason = ENCODER_RUNTIME_TRIM_FREEZE_STATUS;
        reset_trim_window(trim);
        return;
    }

    if ((result->mag16 < ENCODER_RUNTIME_TRIM_MAG_MIN) ||
        (result->mag16 > ENCODER_RUNTIME_TRIM_MAG_MAX) ||
        (result->mag15 < ENCODER_RUNTIME_TRIM_MAG_MIN) ||
        (result->mag15 > ENCODER_RUNTIME_TRIM_MAG_MAX))
    {
        trim->active = false;
        trim->freeze_reason = ENCODER_RUNTIME_TRIM_FREEZE_MAGNITUDE;
        reset_trim_window(trim);
        return;
    }

    if (adc_is_rail(sample->a1_sin_raw) || adc_is_rail(sample->a1_cos_raw) ||
        adc_is_rail(sample->a2_sin_raw) || adc_is_rail(sample->a2_cos_raw))
    {
        trim->active = false;
        trim->freeze_reason = ENCODER_RUNTIME_TRIM_FREEZE_STATUS;
        reset_trim_window(trim);
        return;
    }

    update_min_max(sample->a1_sin_raw, &trim->a1_sin_min, &trim->a1_sin_max);
    update_min_max(sample->a1_cos_raw, &trim->a1_cos_min, &trim->a1_cos_max);
    update_min_max(sample->a2_sin_raw, &trim->a2_sin_min, &trim->a2_sin_max);
    update_min_max(sample->a2_cos_raw, &trim->a2_cos_min, &trim->a2_cos_max);

    bin = (uint32_t)((wrap_deg(result->angle_deg_raw) *
                      (float)ENCODER_RUNTIME_TRIM_BIN_COUNT) /
                     360.0f);
    if (bin >= ENCODER_RUNTIME_TRIM_BIN_COUNT)
    {
        bin = ENCODER_RUNTIME_TRIM_BIN_COUNT - 1U;
    }
    trim->coverage_mask |= (1UL << bin);
    trim->window_count++;
    trim->accepted_samples++;

    if ((trim->window_count < ENCODER_RUNTIME_TRIM_MIN_WINDOW_SAMPLES) ||
        (trim->coverage_mask != ENCODER_RUNTIME_TRIM_FULL_COVERAGE_MASK))
    {
        trim->active = trim->update_count > 0U;
        trim->freeze_reason = trim->active ? ENCODER_RUNTIME_TRIM_FREEZE_NONE :
                                             ENCODER_RUNTIME_TRIM_FREEZE_COVERAGE;
        return;
    }

    a1_sin_center = 0.5f * ((float)trim->a1_sin_min + (float)trim->a1_sin_max);
    a1_cos_center = 0.5f * ((float)trim->a1_cos_min + (float)trim->a1_cos_max);
    a2_sin_center = 0.5f * ((float)trim->a2_sin_min + (float)trim->a2_sin_max);
    a2_cos_center = 0.5f * ((float)trim->a2_cos_min + (float)trim->a2_cos_max);

    trim->a1_center_sin_delta =
        update_trim_delta(trim->a1_center_sin_delta,
                          a1_sin_center - factory->a1.center_sin);
    trim->a1_center_cos_delta =
        update_trim_delta(trim->a1_center_cos_delta,
                          a1_cos_center - factory->a1.center_cos);
    trim->a2_center_sin_delta =
        update_trim_delta(trim->a2_center_sin_delta,
                          a2_sin_center - factory->a2.center_sin);
    trim->a2_center_cos_delta =
        update_trim_delta(trim->a2_center_cos_delta,
                          a2_cos_center - factory->a2.center_cos);

    trim->active = true;
    trim->freeze_reason = ENCODER_RUNTIME_TRIM_FREEZE_NONE;
    trim->update_count++;
    reset_trim_window(trim);
}

void encoder_process(encoder_state_t *state,
                     const encoder_calibration_t *calibration,
                     const encoder_raw_sample_t *sample,
                     encoder_result_t *result,
                     encoder_diag_t *diag)
{
    float phase_a1 = 0.0f;
    float phase_a2 = 0.0f;
    float mag_a1 = 0.0f;
    float mag_a2 = 0.0f;
    encoder_solver_candidate_t best;
    uint32_t status = ENCODER_STATUS_OK;

    if (result == NULL)
    {
        return;
    }

    result->angle_deg = 0.0f;
    result->angle_deg_raw = 0.0f;
    result->angle_deg_filtered = 0.0f;
    result->angle_counts = 0U;
    result->phase16_deg = 0.0f;
    result->phase15_deg = 0.0f;
    result->coarse_deg = 0.0f;
    result->mag16 = 0.0f;
    result->mag15 = 0.0f;
    result->status = ENCODER_STATUS_OK;
    init_diag(diag);

    if (sample == NULL)
    {
        result->status = ENCODER_STATUS_NOT_CALIBRATED | ENCODER_STATUS_HOLD_LAST;
        hold_last_angle(state, result);
        return;
    }

    if ((calibration == NULL) || !calibration->valid)
    {
        status |= ENCODER_STATUS_NOT_CALIBRATED | ENCODER_STATUS_HOLD_LAST;
        hold_last_angle(state, result);
        result->status = status;
        return;
    }

    if (adc_is_rail(sample->a1_sin_raw) || adc_is_rail(sample->a1_cos_raw) ||
        adc_is_rail(sample->a2_sin_raw) || adc_is_rail(sample->a2_cos_raw))
    {
        status |= ENCODER_STATUS_ADC_RAIL;
    }

    if (!transform_track(&calibration->a1, sample->a1_sin_raw, sample->a1_cos_raw,
                         &phase_a1, &mag_a1))
    {
        status |= ENCODER_STATUS_TRACK16_WEAK | ENCODER_STATUS_CAL_FAILED;
    }

    if (!transform_track(&calibration->a2, sample->a2_sin_raw, sample->a2_cos_raw,
                         &phase_a2, &mag_a2))
    {
        status |= ENCODER_STATUS_TRACK15_WEAK | ENCODER_STATUS_CAL_FAILED;
    }

    if (mag_a1 < ENCODER_MIN_NORMALIZED_MAG)
    {
        status |= ENCODER_STATUS_TRACK16_WEAK;
    }

    if (mag_a2 < ENCODER_MIN_NORMALIZED_MAG)
    {
        status |= ENCODER_STATUS_TRACK15_WEAK;
    }

    if (diag != NULL)
    {
        diag->phase_a1_deg = phase_a1;
        diag->phase_a2_deg = phase_a2;
    }

    if (status == ENCODER_STATUS_OK)
    {
        if (solve_vernier(phase_a1, phase_a2,
                          calibration->phase_a1_zero_deg, calibration->phase_a2_zero_deg,
                          &best))
        {
            stabilize_vernier_branch(state, &best);

            result->mag16 = mag_a1;
            result->mag15 = mag_a2;
            result->phase16_deg = best.p16;
            result->phase15_deg = best.p15;
            result->coarse_deg = best.coarse;

            if (diag != NULL)
            {
                diag->angle_deg = best.angle;
                diag->coarse_angle_deg = best.coarse;
                diag->phase16_error_deg = best.phase16_error;
                diag->phase15_error_deg = best.phase15_error;
            }

            if (best.error > ENCODER_TRACK_PHASE_TOLERANCE_DEG)
            {
                status |= ENCODER_STATUS_TRACK_MISMATCH;
            }
        }
        else
        {
            status |= ENCODER_STATUS_CAL_FAILED;
        }
    }

    if (status == ENCODER_STATUS_OK)
    {
        const float raw_angle_deg = best.angle;
        result->angle_deg_raw = raw_angle_deg;
        result->angle_deg_filtered = filter_published_angle(state, raw_angle_deg);
        result->angle_deg = result->angle_deg_filtered;
        result->angle_counts = angle_to_counts(result->angle_deg);

        if (state != NULL)
        {
            state->last_angle_deg = result->angle_deg;
            state->last_angle_raw_deg = result->angle_deg_raw;
            state->last_angle_counts = result->angle_counts;
            state->has_valid_angle = true;
        }
    }
    else
    {
        status |= ENCODER_STATUS_HOLD_LAST;
        hold_last_angle(state, result);
    }

    result->status = status;
}
