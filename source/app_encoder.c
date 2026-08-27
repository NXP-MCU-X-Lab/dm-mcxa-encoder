#include "app_encoder.h"
#include "app_adc.h"

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
/* Acceptance band for trimming: exactly the set of magnitudes the gain authority
 * can still pull back to 1.0. Gain is multiplicative -- a delta d scales T by
 * (1 + d) -- so the band is not symmetric about 1.0. With a limit of L, a
 * magnitude m is correctable when 1/(1 + L) <= m <= 1/(1 - L).
 *
 * Getting this wrong in either direction is a trap. Narrower than the authority
 * (the original 0.75..1.25 against +/-0.5) leaves half of it unreachable: the
 * estimator switches itself off precisely when the amplitude has drifted far
 * enough to need it, and can never climb back. Wider than the authority admits
 * magnitudes it cannot fix, so the AGC parks against its clamp. The rail and
 * WEAK checks alongside are what actually reject unusable samples. */
#define ENCODER_RUNTIME_TRIM_MAG_MIN (1.0f / (1.0f + ENCODER_RUNTIME_TRIM_GAIN_TOTAL_LIMIT))
#define ENCODER_RUNTIME_TRIM_MAG_MAX (1.0f / (1.0f - ENCODER_RUNTIME_TRIM_GAIN_TOTAL_LIMIT))
#define ENCODER_RUNTIME_TRIM_BIN_COUNT (16U)
#define ENCODER_RUNTIME_TRIM_FULL_COVERAGE_MASK (0xFFFFUL)
#define ENCODER_RUNTIME_TRIM_MIN_WINDOW_SAMPLES (512U)
#define ENCODER_RUNTIME_TRIM_ALPHA (0.125f)
/* 5 s at 10 kHz. Long enough that ordinary rough running never re-acquires,
 * short enough that a board which locked on bad data heals itself instead of
 * needing a reflash. */
#define ENCODER_RUNTIME_TRIM_RELOCK_SAMPLES (50000U)
/* Windows that still take the measured centre outright rather than creeping.
 * The first lock is necessarily approximate -- harmonic distortion makes the
 * waveform asymmetric, so the min/max midpoint sits a few hundred counts off the
 * true centre even across whole cycles. Creeping that away at the drift step
 * limit would take hundreds of revolutions; a few more snap windows close it in
 * a few. Only once those are spent is the remaining error actually drift. */
#define ENCODER_RUNTIME_TRIM_SNAP_SOLVES (4U)

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

/* Rotor-angle binned mag window: each bin holds the latest mag sampled while
 * the rotor was in that angular sector. Once every bin has been visited at
 * least once, we publish the mean across the window so the displayed mag does
 * not depend on where the rotor happened to be parked at power-up. The mean
 * is also the AGC feedback signal that drives gain trim toward unity. */
static float mag_window_update(encoder_mag_window_t *win, float raw_mag, float angle_deg)
{
    uint32_t bin;

    if (win == NULL)
    {
        return raw_mag;
    }

    bin = (uint32_t)((wrap_deg(angle_deg) * (float)ENCODER_MAG_WINDOW_BINS) / 360.0f);
    if (bin >= ENCODER_MAG_WINDOW_BINS)
    {
        bin = ENCODER_MAG_WINDOW_BINS - 1U;
    }

    win->sum -= win->bins[bin];
    win->bins[bin] = raw_mag;
    win->sum += raw_mag;
    win->coverage_mask |= ((uint32_t)1U << bin);
    if (win->coverage_mask == ENCODER_MAG_WINDOW_FULL_MASK)
    {
        win->full_revolution_seen = true;
    }

    if (!win->full_revolution_seen)
    {
        return raw_mag;
    }

    return win->sum / (float)ENCODER_MAG_WINDOW_BINS;
}


/* Type-II tracking observer (software PLL) — canonical resolver / inductive
 * encoder output stage. Closed-loop characteristic eq: s^2 + Kp*s + Ki,
 * Kp = 2*zeta*wn, Ki = wn^2. Precomputed at compile time for the fixed
 * 10 kHz ADC sample rate to keep the hot path branch-free. */
#define ENCODER_TRACKING_DT      (1.0f / (float)ADC_SAMPLE_RATE_HZ)
#define ENCODER_TRACKING_WN      (2.0f * ENCODER_PI * ENCODER_TRACKING_BW_HZ)
#define ENCODER_TRACKING_KP      (2.0f * ENCODER_TRACKING_ZETA * ENCODER_TRACKING_WN)
#define ENCODER_TRACKING_KI      (ENCODER_TRACKING_WN * ENCODER_TRACKING_WN)

static float filter_published_angle(encoder_state_t *state, float raw_angle_deg)
{
    float error;
    float predicted_angle;

    if (state == NULL)
    {
        return wrap_deg(raw_angle_deg);
    }

    if (!state->filter_initialized)
    {
        state->filtered_angle_deg = wrap_deg(raw_angle_deg);
        state->tracking_velocity_dps = 0.0f;
        state->filter_initialized = true;
        return state->filtered_angle_deg;
    }

    predicted_angle = wrap_deg(state->filtered_angle_deg +
                               (state->tracking_velocity_dps * ENCODER_TRACKING_DT));
    error = signed_angle_error_deg(raw_angle_deg, predicted_angle);
    if (fabsf(error) > ENCODER_FILTER_RESYNC_ERROR_DEG)
    {
        state->tracking_velocity_dps = clamp_float(
            signed_angle_error_deg(raw_angle_deg, state->last_angle_raw_deg) / ENCODER_TRACKING_DT,
            -ENCODER_TRACKING_VEL_MAX_DPS,
            ENCODER_TRACKING_VEL_MAX_DPS);
        state->filtered_angle_deg = wrap_deg(raw_angle_deg);
        return state->filtered_angle_deg;
    }

    state->tracking_velocity_dps += ENCODER_TRACKING_KI * error * ENCODER_TRACKING_DT;
    state->tracking_velocity_dps = clamp_float(state->tracking_velocity_dps,
                                               -ENCODER_TRACKING_VEL_MAX_DPS,
                                               ENCODER_TRACKING_VEL_MAX_DPS);
    state->filtered_angle_deg = wrap_deg(predicted_angle +
                                         (ENCODER_TRACKING_KP * error * ENCODER_TRACKING_DT));
    return state->filtered_angle_deg;
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
    trim->a1_raw_mag_sum = 0.0f;
    trim->a2_raw_mag_sum = 0.0f;
    trim->raw_mag_count = 0U;
    trim->cross_count[0] = 0U;
    trim->cross_count[1] = 0U;
    trim->cross_count[2] = 0U;
    trim->cross_count[3] = 0U;
}

/* Acquisition needs to know the rotor has traversed whole electrical cycles, and
 * span alone cannot tell it: half the peak-to-peak swing is also what a 180 deg
 * arc produces, where min is -A, max is 0, and the midpoint is off by A/2. So
 * count centre crossings instead. Two transitions per channel is one full cycle.
 *
 * The Schmitt band matters as much as the count. Park the rotor so a channel sits
 * near its centre and ADC noise alone would manufacture thousands of crossings;
 * requiring the signal to travel a quarter amplitude past the centre before the
 * sign is allowed to flip makes noise incapable of producing even one. */
/* Sized against the noise floor, not against the signal. A few LSB of ADC noise
 * is under 0.001 normalized, so 0.1 rejects it by more than two orders of
 * magnitude. Going wider is not free: the excursion is measured against the
 * assumed centre, so a weak signal sitting on a large centre offset may never
 * reach a generous band at all -- 0.25 left a half-amplitude board with a
 * 2000-count offset unable to register a single crossing. */
#define ENCODER_TRIM_CROSS_HYSTERESIS (0.1f)
#define ENCODER_TRIM_CROSS_REQUIRED (3U)

static void update_crossings(encoder_runtime_trim_t *trim,
                             const encoder_calibration_t *factory,
                             const encoder_raw_sample_t *sample)
{
    const float value[4] = {
        (float)sample->a1_sin_raw - factory->a1.center_sin,
        (float)sample->a1_cos_raw - factory->a1.center_cos,
        (float)sample->a2_sin_raw - factory->a2.center_sin,
        (float)sample->a2_cos_raw - factory->a2.center_cos,
    };
    const float inverse_amplitude[4] = {
        factory->a1.t00, factory->a1.t11, factory->a2.t00, factory->a2.t11,
    };
    uint32_t i;

    for (i = 0U; i < 4U; i++)
    {
        const float normalized = value[i] * inverse_amplitude[i];
        const bool was_high = ((trim->cross_sign_mask >> i) & 1U) != 0U;

        if (was_high ? (normalized < -ENCODER_TRIM_CROSS_HYSTERESIS)
                     : (normalized > ENCODER_TRIM_CROSS_HYSTERESIS))
        {
            trim->cross_sign_mask ^= (uint8_t)(1U << i);
            if (trim->cross_count[i] < UINT8_MAX)
            {
                trim->cross_count[i]++;
            }
        }
    }
}

static bool crossings_cover_cycles(const encoder_runtime_trim_t *trim)
{
    return (trim->cross_count[0] >= ENCODER_TRIM_CROSS_REQUIRED) &&
           (trim->cross_count[1] >= ENCODER_TRIM_CROSS_REQUIRED) &&
           (trim->cross_count[2] >= ENCODER_TRIM_CROSS_REQUIRED) &&
           (trim->cross_count[3] >= ENCODER_TRIM_CROSS_REQUIRED);
}

/* Acquisition takes the measured centre outright; tracking creeps toward it.
 *
 * The compiled-in defaults are one specific board's measured values, so every
 * other board starts with a centre offset of whatever those two boards differ
 * by -- thousands of counts is ordinary. Creeping at the tracking step limit
 * would take minutes to close that, and a centre error is not a small error:
 * 2000 counts against a 5464-count amplitude destroys the angle outright
 * (measured error span 348 deg, `encoder_sim bootstrap`). Once locked, the only
 * thing left to follow is slow thermal drift, which is what the step limit is
 * sized for. */
static float update_trim_delta(float current_delta, float desired_delta, bool tracking)
{
    float step;

    if (!tracking)
    {
        return clamp_float(desired_delta,
                           -ENCODER_RUNTIME_TRIM_TOTAL_LIMIT_COUNTS,
                           ENCODER_RUNTIME_TRIM_TOTAL_LIMIT_COUNTS);
    }

    step = clamp_float((desired_delta - current_delta) * ENCODER_RUNTIME_TRIM_ALPHA,
                       -ENCODER_RUNTIME_TRIM_STEP_LIMIT_COUNTS,
                       ENCODER_RUNTIME_TRIM_STEP_LIMIT_COUNTS);

    return clamp_float(current_delta + step,
                       -ENCODER_RUNTIME_TRIM_TOTAL_LIMIT_COUNTS,
                       ENCODER_RUNTIME_TRIM_TOTAL_LIMIT_COUNTS);
}

/* AGC step: per-revolution step proportional to (1 - mag_mean_observed), then
 * clamped. Converges current gain so that mean magnitude approaches 1.0 even if
 * the analog amplitude has drifted from factory-cal conditions. */
static float update_gain_delta(float current_delta, float observed_mag)
{
    float step;

    if (!is_finite_float(observed_mag) || (observed_mag <= 0.0f))
    {
        return current_delta;
    }

    step = clamp_float(1.0f - observed_mag,
                       -ENCODER_RUNTIME_TRIM_GAIN_STEP_LIMIT,
                       ENCODER_RUNTIME_TRIM_GAIN_STEP_LIMIT);

    return clamp_float(current_delta + step,
                       -ENCODER_RUNTIME_TRIM_GAIN_TOTAL_LIMIT,
                       ENCODER_RUNTIME_TRIM_GAIN_TOTAL_LIMIT);
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

/* Linear interpolation into a per-track INL table, indexed by electrical phase.
 * The table wraps, so bin N-1 interpolates back into bin 0. */
static float inl_lookup(const float *correction, float phase_deg)
{
    const float pos = phase_deg * ((float)ENCODER_INL_BINS / 360.0f);
    uint32_t bin = (uint32_t)pos;
    float frac;

    if (bin >= ENCODER_INL_BINS)
    {
        bin = ENCODER_INL_BINS - 1U;
    }
    frac = pos - (float)bin;

    return correction[bin] +
           (frac * (correction[(bin + 1U) % ENCODER_INL_BINS] - correction[bin]));
}

static bool transform_track(const encoder_track_calibration_t *track,
                            const encoder_inl_track_t *inl,
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
    float local_phase = wrap_deg(atan2f(sin_corr, cos_corr) * (180.0f / ENCODER_PI));

    if (inl != NULL)
    {
        /* Evaluated at the measured phase rather than the true one. The resulting
         * second-order error is what makes a second solve pass worth measuring. */
        local_phase = wrap_deg(local_phase - inl_lookup(inl->correction, local_phase));
    }

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
        result->turn_count = state->turn_count;
        result->multi_turn_deg = ((float)state->turn_count * 360.0f) +
                                 (((float)state->last_angle_counts * 360.0f) /
                                  (float)ENCODER_COUNTS_PER_REV);
    }
    else
    {
        result->angle_deg = 0.0f;
        result->angle_deg_raw = 0.0f;
        result->angle_deg_filtered = 0.0f;
        result->angle_counts = 0U;
        result->turn_count = (state != NULL) ? state->turn_count : 0;
        result->multi_turn_deg = 0.0f;
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
 * predicted angle. Prevents single-frame ADC glitches from flipping branches.
 *
 * The prediction uses last_angle_raw_deg (the previous Vernier solve, not the
 * PLL-filtered display angle) plus a velocity feed-forward term. Using the
 * filtered angle would create a circular dependency where the tracking
 * observer's lag biases the discrete branch decision, which can pick the wrong
 * branch on accel/recovery. */
static void stabilize_vernier_branch(const encoder_state_t *state, encoder_solver_candidate_t *candidate)
{
    encoder_solver_candidate_t adjusted;
    float predicted_angle;
    float current_step;
    float best_distance;
    int32_t branch;

    if ((state == NULL) || !state->has_valid_angle || (candidate == NULL))
    {
        return;
    }

    predicted_angle = wrap_deg(state->last_angle_raw_deg +
                               (state->tracking_velocity_dps * ENCODER_TRACKING_DT));

    current_step = fabsf(signed_angle_error_deg(candidate->angle, predicted_angle));
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
                                         predicted_angle));

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

static bool candidate_matches_motion(const encoder_state_t *state,
                                     const encoder_solver_candidate_t *candidate)
{
    float elapsed_samples;
    float predicted_angle;

    if ((state == NULL) || !state->has_valid_angle || (candidate == NULL) ||
        (state->hold_last_streak >= ENCODER_MOTION_INNOVATION_MAX_STREAK))
    {
        return true;
    }

    elapsed_samples = (float)(state->hold_last_streak + 1U);
    predicted_angle = wrap_deg(state->last_angle_raw_deg +
                               (state->tracking_velocity_dps * ENCODER_TRACKING_DT * elapsed_samples));
    return fabsf(signed_angle_error_deg(candidate->angle, predicted_angle)) <=
           ENCODER_MOTION_INNOVATION_LIMIT_DEG;
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
                          const encoder_inl_t *inl,
                          const encoder_raw_sample_t *sample,
                          uint32_t *status)
{
    const encoder_inl_track_t *inl16 = ((inl != NULL) && inl->valid) ? &inl->t16 : NULL;
    const encoder_inl_track_t *inl15 = ((inl != NULL) && inl->valid) ? &inl->t15 : NULL;
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

    if (!transform_track(&calibration->a1, inl16, sample->a1_sin_raw, sample->a1_cos_raw,
                         &phase_a1, &mag_a1))
    {
        local_status |= ENCODER_STATUS_TRACK16_WEAK | ENCODER_STATUS_CAL_FAILED;
    }

    if (!transform_track(&calibration->a2, inl15, sample->a2_sin_raw, sample->a2_cos_raw,
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

void encoder_inl_init(encoder_inl_t *inl)
{
    if (inl == NULL)
    {
        return;
    }

    *inl = (encoder_inl_t){0};
}

static void inl_accumulate_track(encoder_inl_track_t *track, float phase_deg, float fine_delta)
{
    uint32_t bin = (uint32_t)(wrap_deg(phase_deg) * ((float)ENCODER_INL_BINS / 360.0f));

    if (bin >= ENCODER_INL_BINS)
    {
        bin = ENCODER_INL_BINS - 1U;
    }

    track->sum[bin] += fine_delta;
    track->count[bin]++;
}

void encoder_inl_accumulate(encoder_inl_t *inl,
                            const encoder_diag_t *diag,
                            const encoder_result_t *result)
{
    if ((inl == NULL) || (diag == NULL) || (result == NULL) ||
        (result->status != ENCODER_STATUS_OK))
    {
        return;
    }

    inl_accumulate_track(&inl->t16, diag->phase_a1_deg, result->fine_delta_deg);
    inl_accumulate_track(&inl->t15, diag->phase_a2_deg, result->fine_delta_deg);
}

/* fine_delta = 16*e15 - 15*e16, so the bin mean taken against track 16's own
 * phase converges to -15*e16 and against track 15's phase to +16*e15. Divide
 * each by its own coefficient, then remove the DC term: a constant phase shift
 * is indistinguishable from a zero offset, and leaving it in would move the
 * captured zero every time the table is rebuilt. */
static bool inl_solve_track(encoder_inl_track_t *track, float coefficient)
{
    float mean_sum = 0.0f;
    uint32_t bin;

    for (bin = 0U; bin < ENCODER_INL_BINS; bin++)
    {
        if (track->count[bin] < ENCODER_INL_MIN_BIN_SAMPLES)
        {
            return false;
        }

        track->correction[bin] = track->sum[bin] / ((float)track->count[bin] * coefficient);
        mean_sum += track->correction[bin];
    }

    mean_sum /= (float)ENCODER_INL_BINS;
    for (bin = 0U; bin < ENCODER_INL_BINS; bin++)
    {
        track->correction[bin] -= mean_sum;
    }

    return true;
}

bool encoder_inl_solve(encoder_inl_t *inl)
{
    if (inl == NULL)
    {
        return false;
    }

    inl->valid = inl_solve_track(&inl->t16, -(float)ENCODER_TRACK15_CYCLES) &&
                 inl_solve_track(&inl->t15, (float)ENCODER_TRACK16_CYCLES);

    return inl->valid;
}

void encoder_state_init(encoder_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    *state = (encoder_state_t){0};
}

void encoder_state_reset_turn_count(encoder_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->turn_count = 0;
}

void encoder_runtime_trim_init(encoder_runtime_trim_t *trim)
{
    if (trim == NULL)
    {
        return;
    }

    *trim = (encoder_runtime_trim_t){0};
    trim->enabled = true;
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

    {
        const float a1_gain = 1.0f + trim->a1_gain_delta;
        const float a2_gain = 1.0f + trim->a2_gain_delta;
        effective->a1.t00 *= a1_gain;
        effective->a1.t10 *= a1_gain;
        effective->a1.t11 *= a1_gain;
        effective->a2.t00 *= a2_gain;
        effective->a2.t10 *= a2_gain;
        effective->a2.t11 *= a2_gain;
    }
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
    bool acquiring;

    if ((trim == NULL) || (sample == NULL) || (result == NULL))
    {
        return;
    }

    /* Acquisition runs on raw samples alone. The centre is min/max, which needs
     * neither a valid angle nor a healthy magnitude -- and demanding those first
     * is a trap, because a board whose centre is wrong by a few thousand counts
     * produces neither. That is how the estimator ends up refusing to run exactly
     * when it is the thing that would fix the problem. Rails are the only veto
     * that makes sense here: a clipped sample carries no min/max information. */
    acquiring = !trim->has_locked;

    if (!trim->enabled || (factory == NULL) || !factory->valid ||
        adc_is_rail(sample->a1_sin_raw) || adc_is_rail(sample->a1_cos_raw) ||
        adc_is_rail(sample->a2_sin_raw) || adc_is_rail(sample->a2_cos_raw))
    {
        trim->active = false;
        reset_trim_window(trim);
        return;
    }

    /* Once locked the angle is trustworthy, so tracking can afford to be picky.
     * AGC gate and feedback consume raw instantaneous magnitude, not the
     * binned-window display mean. Closing the loop on the display mean would
     * mask local weak / over-amplitude signals and let the AGC chase its own
     * averaged output. */
    if (!acquiring &&
        ((result->status != ENCODER_STATUS_OK) ||
         (result->mag16_raw < ENCODER_RUNTIME_TRIM_MAG_MIN) ||
         (result->mag16_raw > ENCODER_RUNTIME_TRIM_MAG_MAX) ||
         (result->mag15_raw < ENCODER_RUNTIME_TRIM_MAG_MIN) ||
         (result->mag15_raw > ENCODER_RUNTIME_TRIM_MAG_MAX)))
    {
        trim->active = false;

        if (trim->blocked_streak < ENCODER_RUNTIME_TRIM_RELOCK_SAMPLES)
        {
            /* Skip the sample, keep the window. Throwing the window away on every
             * rejected sample is the same trap as the gates above, one level down:
             * a mediocre lock makes the branch guard fire intermittently, each
             * firing discards the half-built window, the window never completes,
             * and the estimator never gets to refine the very error that is
             * causing the firing. */
            trim->blocked_streak++;
            return;
        }

        /* Sustained rejection means the lock itself is wrong, not the samples.
         * Give it up and re-earn one from the raw signal. solve_count goes with
         * it: leaving it high would make the fresh acquisition creep at the drift
         * step limit instead of snapping, which is the whole point of starting
         * over. */
        trim->has_locked = false;
        trim->blocked_streak = 0U;
        trim->solve_count = 0U;
        reset_trim_window(trim);
        return;
    }

    trim->blocked_streak = 0U;

    if (acquiring)
    {
        /* Only acquisition consults the crossing counters, so only acquisition
         * pays for them -- this runs in the 10 kHz ADC interrupt. */
        update_crossings(trim, factory, sample);
    }

    update_min_max(sample->a1_sin_raw, &trim->a1_sin_min, &trim->a1_sin_max);
    update_min_max(sample->a1_cos_raw, &trim->a1_cos_min, &trim->a1_cos_max);
    update_min_max(sample->a2_sin_raw, &trim->a2_sin_min, &trim->a2_sin_max);
    update_min_max(sample->a2_cos_raw, &trim->a2_cos_min, &trim->a2_cos_max);

    trim->a1_raw_mag_sum += result->mag16_raw;
    trim->a2_raw_mag_sum += result->mag15_raw;
    trim->raw_mag_count++;

    bin = (uint32_t)((wrap_deg(result->angle_deg_raw) *
                      (float)ENCODER_RUNTIME_TRIM_BIN_COUNT) /
                     360.0f);
    if (bin >= ENCODER_RUNTIME_TRIM_BIN_COUNT)
    {
        bin = ENCODER_RUNTIME_TRIM_BIN_COUNT - 1U;
    }
    trim->coverage_mask |= (1UL << bin);
    trim->window_count++;

    if (trim->window_count < ENCODER_RUNTIME_TRIM_MIN_WINDOW_SAMPLES)
    {
        trim->active = trim->has_locked;
        return;
    }

    /* Coverage means something different in each phase. While acquiring, the
     * rotor-angle bins come from an angle that is not trustworthy yet, so what
     * counts is whether whole electrical cycles have been traversed -- see
     * update_crossings(). After locking, rotor-angle coverage is both meaningful
     * and the stricter requirement. */
    if (acquiring)
    {
        if (!crossings_cover_cycles(trim))
        {
            trim->active = false;
            return;
        }
    }
    else if (trim->coverage_mask != ENCODER_RUNTIME_TRIM_FULL_COVERAGE_MASK)
    {
        trim->active = trim->has_locked;
        return;
    }

    a1_sin_center = 0.5f * ((float)trim->a1_sin_min + (float)trim->a1_sin_max);
    a1_cos_center = 0.5f * ((float)trim->a1_cos_min + (float)trim->a1_cos_max);
    a2_sin_center = 0.5f * ((float)trim->a2_sin_min + (float)trim->a2_sin_max);
    a2_cos_center = 0.5f * ((float)trim->a2_cos_min + (float)trim->a2_cos_max);

    trim->a1_center_sin_delta =
        update_trim_delta(trim->a1_center_sin_delta,
                          a1_sin_center - factory->a1.center_sin,
                          trim->solve_count >= ENCODER_RUNTIME_TRIM_SNAP_SOLVES);
    trim->a1_center_cos_delta =
        update_trim_delta(trim->a1_center_cos_delta,
                          a1_cos_center - factory->a1.center_cos,
                          trim->solve_count >= ENCODER_RUNTIME_TRIM_SNAP_SOLVES);
    trim->a2_center_sin_delta =
        update_trim_delta(trim->a2_center_sin_delta,
                          a2_sin_center - factory->a2.center_sin,
                          trim->solve_count >= ENCODER_RUNTIME_TRIM_SNAP_SOLVES);
    trim->a2_center_cos_delta =
        update_trim_delta(trim->a2_center_cos_delta,
                          a2_cos_center - factory->a2.center_cos,
                          trim->solve_count >= ENCODER_RUNTIME_TRIM_SNAP_SOLVES);

    /* Gain waits for the tracking phase. During acquisition the magnitudes were
     * computed against a centre known to be wrong, so their mean says more about
     * the centre error than about the analog amplitude. Correcting the centre
     * first costs one extra window and makes the AGC feedback mean what it says. */
    if (!acquiring)
    {
        const float inv_count = (trim->raw_mag_count > 0U)
                                    ? (1.0f / (float)trim->raw_mag_count)
                                    : 0.0f;
        const float a1_raw_mag_mean = trim->a1_raw_mag_sum * inv_count;
        const float a2_raw_mag_mean = trim->a2_raw_mag_sum * inv_count;

        trim->a1_gain_delta = update_gain_delta(trim->a1_gain_delta, a1_raw_mag_mean);
        trim->a2_gain_delta = update_gain_delta(trim->a2_gain_delta, a2_raw_mag_mean);
    }

    trim->active = true;
    trim->has_locked = true;
    if (trim->solve_count < UINT32_MAX)
    {
        trim->solve_count++;
    }
    reset_trim_window(trim);
}

void encoder_process(encoder_state_t *state,
                     const encoder_calibration_t *calibration,
                     const encoder_inl_t *inl,
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
    const encoder_inl_track_t *inl16 = ((inl != NULL) && inl->valid) ? &inl->t16 : NULL;
    const encoder_inl_track_t *inl15 = ((inl != NULL) && inl->valid) ? &inl->t15 : NULL;

    if (result == NULL)
    {
        return;
    }

    result->angle_deg = 0.0f;
    result->angle_deg_raw = 0.0f;
    result->angle_deg_filtered = 0.0f;
    result->angular_velocity_dps = 0.0f;
    result->angle_counts = 0U;
    result->phase16_deg = 0.0f;
    result->phase15_deg = 0.0f;
    result->coarse_deg = 0.0f;
    result->fine_delta_deg = 0.0f;
    result->mag16 = 0.0f;
    result->mag15 = 0.0f;
    result->mag16_raw = 0.0f;
    result->mag15_raw = 0.0f;
    result->turn_count = (state != NULL) ? state->turn_count : 0;
    result->multi_turn_deg = 0.0f;
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

    if (!transform_track(&calibration->a1, inl16, sample->a1_sin_raw, sample->a1_cos_raw,
                         &phase_a1, &mag_a1))
    {
        status |= ENCODER_STATUS_TRACK16_WEAK | ENCODER_STATUS_CAL_FAILED;
    }

    if (!transform_track(&calibration->a2, inl15, sample->a2_sin_raw, sample->a2_cos_raw,
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

            result->mag16_raw = mag_a1;
            result->mag15_raw = mag_a2;
            result->mag16 = mag_window_update((state != NULL) ? &state->mag_window_a1 : NULL,
                                              mag_a1, best.angle);
            result->mag15 = mag_window_update((state != NULL) ? &state->mag_window_a2 : NULL,
                                              mag_a2, best.angle);
            result->phase16_deg = best.p16;
            result->phase15_deg = best.p15;
            result->coarse_deg = best.coarse;
            result->fine_delta_deg =
                signed_angle_error_deg(best.p16, wrap_deg(best.coarse * (float)ENCODER_TRACK16_CYCLES));

            if (diag != NULL)
            {
                diag->angle_deg = best.angle;
                diag->coarse_angle_deg = best.coarse;
                diag->phase16_error_deg = best.phase16_error;
                diag->phase15_error_deg = best.phase15_error;
            }

            if ((fabsf(result->fine_delta_deg) > ENCODER_BRANCH_CONFIDENCE_LIMIT_DEG) ||
                !candidate_matches_motion(state, &best))
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

        if ((state != NULL) &&
            (state->hold_last_streak >= ENCODER_FILTER_HOLD_RESYNC_SAMPLES))
        {
            state->filter_initialized = false;
            state->tracking_velocity_dps = 0.0f;
        }

        result->angle_deg_raw = raw_angle_deg;
        result->angle_deg_filtered = filter_published_angle(state, raw_angle_deg);
        result->angular_velocity_dps = (state != NULL) ? state->tracking_velocity_dps : 0.0f;
        result->angle_deg = result->angle_deg_filtered;

        /* Output dead-band: when the new filtered angle is within
         * ENCODER_OUTPUT_DEADBAND_DEG of the last published angle, hold the last
         * value. Backstop for the PLL dead-zone — at rotor positions where the
         * analog SNR is worse the PLL may not fully freeze, so we add a second
         * gate here that only affects the published output, not the PLL state. */
        if ((state != NULL) && state->has_valid_angle &&
            (fabsf(signed_angle_error_deg(result->angle_deg, state->last_angle_deg)) <
             ENCODER_OUTPUT_DEADBAND_DEG))
        {
            result->angle_deg = state->last_angle_deg;
        }

        /* The 16-bit control interface follows the current Vernier solve. The
         * observer remains available through angle_deg/angle_deg_filtered for
         * smooth monitoring, but must not add speed-dependent protocol latency. */
        result->angle_counts = angle_to_counts(result->angle_deg_raw);

        if ((state != NULL) && state->has_valid_angle)
        {
            int32_t diff = (int32_t)result->angle_counts - (int32_t)state->last_angle_counts;
            if (diff < -((int32_t)ENCODER_COUNTS_PER_REV / 2))
            {
                diff += (int32_t)ENCODER_COUNTS_PER_REV;
            }
            else if (diff > ((int32_t)ENCODER_COUNTS_PER_REV / 2))
            {
                diff -= (int32_t)ENCODER_COUNTS_PER_REV;
            }
            if ((diff >= -ENCODER_ANGLE_COUNT_HYSTERESIS) &&
                (diff <= ENCODER_ANGLE_COUNT_HYSTERESIS))
            {
                result->angle_counts = state->last_angle_counts;
            }
        }

        if (state != NULL)
        {
            /* Multi-turn boundary detection runs on the PUBLISHED angle_counts,
             * not on any float angle. Counts are what the protocol sends, and they
             * carry an output hysteresis that can hold the previous value across
             * the wrap; deriving the turn counter from anything else lets the two
             * disagree by a whole revolution for as long as the hysteresis holds,
             * which one T-Format ID3 frame would publish as ABS and ABM describing
             * positions 360 deg apart. Comparing published against published makes
             * the pair consistent by construction. */
            if (state->has_valid_angle)
            {
                const uint32_t low = ENCODER_COUNTS_PER_REV / 4U;
                const uint32_t high = ENCODER_COUNTS_PER_REV - low;

                if ((state->last_angle_counts >= high) && (result->angle_counts < low))
                {
                    if (state->turn_count < INT32_MAX) state->turn_count++;
                }
                else if ((state->last_angle_counts < low) && (result->angle_counts >= high))
                {
                    if (state->turn_count > INT32_MIN) state->turn_count--;
                }
            }

            state->last_angle_deg = result->angle_deg;
            state->last_angle_raw_deg = result->angle_deg_raw;
            state->last_angle_counts = result->angle_counts;
            state->has_valid_angle = true;
            state->hold_last_streak = 0U;

            result->turn_count = state->turn_count;
            /* Built from the same published counts the turn counter watches, so
             * angle_counts, turn_count and multi_turn_deg tell one story. */
            result->multi_turn_deg = ((float)state->turn_count * 360.0f) +
                                     (((float)result->angle_counts * 360.0f) /
                                      (float)ENCODER_COUNTS_PER_REV);
        }
    }
    else
    {
        status |= ENCODER_STATUS_HOLD_LAST;
        hold_last_angle(state, result);
        if (state != NULL)
        {
            if (state->hold_last_streak < UINT32_MAX)
            {
                state->hold_last_streak++;
            }
        }
    }

    result->status = status;
}
