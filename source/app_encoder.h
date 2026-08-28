#ifndef APP_ENCODER_H_
#define APP_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

#define ENCODER_TRACK16_CYCLES (16U)
#define ENCODER_TRACK15_CYCLES (15U)
#define ENCODER_CAL_SAMPLE_COUNT (8192U)
#define ENCODER_COUNTS_PER_REV (65536U)

#define ENCODER_STATUS_OK (0UL)
#define ENCODER_STATUS_NOT_CALIBRATED (1UL << 0)
#define ENCODER_STATUS_TRACK16_WEAK (1UL << 1)
#define ENCODER_STATUS_TRACK15_WEAK (1UL << 2)
#define ENCODER_STATUS_ADC_RAIL (1UL << 3)
#define ENCODER_STATUS_TRACK_MISMATCH (1UL << 4)
#define ENCODER_STATUS_CAL_FAILED (1UL << 5)
#define ENCODER_STATUS_HOLD_LAST (1UL << 6)
#define ENCODER_STATUS_CAL_STORAGE_INVALID (1UL << 7)
#define ENCODER_STATUS_FACTORY_CAL_REQUIRED (1UL << 8)

/* The bits that mean "this sample's position is untrustworthy", as opposed to
 * "this board has never been through a factory step". Only the former may
 * invalidate an output sample: a board running on compiled-in defaults carries
 * CAL_STORAGE_INVALID | FACTORY_CAL_REQUIRED for its entire life, and folding
 * those in made every never-calibrated board report a permanent fault -- including
 * long after the online estimator had converged to full accuracy. */
#define ENCODER_STATUS_POSITION_INVALID_MASK                                   \
    (ENCODER_STATUS_NOT_CALIBRATED | ENCODER_STATUS_TRACK16_WEAK |             \
     ENCODER_STATUS_TRACK15_WEAK | ENCODER_STATUS_ADC_RAIL |                   \
     ENCODER_STATUS_TRACK_MISMATCH | ENCODER_STATUS_CAL_FAILED |               \
     ENCODER_STATUS_HOLD_LAST)

/* Type-II tracking observer (software PLL) for published angle, matching the
 * canonical resolver-to-digital converter loop (AD2S1210 / TI SPRAA94 / etc.).
 * - BW   sets closed-loop bandwidth in Hz.
 * - ZETA sets damping ratio (0.707 = Butterworth, no overshoot).
 * Kp/Ki are precomputed from BW, ZETA, and the ADC sample period at compile time.
 * VEL_MAX_DPS clamps the velocity integrator to a physically plausible bound so
 * startup / glitch transients cannot wind it up. HOLD_RESYNC_SAMPLES forces a
 * re-lock if the signal has been invalid long enough that the stored position is
 * meaningless (LOS recovery, AD2S1210 style). */
#define ENCODER_TRACKING_BW_HZ      (100.0f)
#define ENCODER_TRACKING_ZETA       (0.707f)
#define ENCODER_TRACKING_VEL_MAX_DPS    (216000.0f)
#define ENCODER_FILTER_HOLD_RESYNC_SAMPLES (50U)

/* Branch-decision guards. Kept here with the rest of the tuning surface so the
 * host regression can assert against the real values rather than a copy.
 *
 * At 6000 rpm the shaft advances 3.6 deg/sample. Nine degrees admits an
 * instantaneous direction reversal plus model/noise margin; the 150 deg
 * fine-phase limit keeps 30 deg of distance from the Vernier branch boundary.
 *
 * CONFIDENCE_LIMIT is coupled to uncorrected harmonic content: |fine_delta| also
 * grows with distortion, so on a board with enough 3rd harmonic the guard starts
 * tripping on healthy samples. */
#define ENCODER_BRANCH_CONFIDENCE_LIMIT_DEG (150.0f)
#define ENCODER_MOTION_INNOVATION_LIMIT_DEG (9.0f)
#define ENCODER_FILTER_RESYNC_ERROR_DEG (9.0f)

/* How long the motion gate keeps vetoing after signal loss. The prediction is
 * extrapolated over the whole outage while the tolerance stays fixed, so its
 * uncertainty grows as 0.5*a*t^2 and eventually exceeds the 11.25 deg it would
 * need to tell one Vernier branch from another -- past that point the gate is no
 * longer discriminating, only delaying. Measured with `encoder_sim highspeed`:
 * at 2e6 deg/s^2 (0 to 6000 rpm in 18 ms) the gate still passes at 20 samples and
 * blocks at 30, so recovery stalls until the streak reaches this limit. Borrowing
 * the observer's 50-sample resync count made a 3 ms outage cost 5 ms of held
 * position. */
#define ENCODER_MOTION_INNOVATION_MAX_STREAK (20U)

/* Output hysteresis: if the new filtered angle is within this threshold of the
 * previously published angle, hold the previous value. Eliminates static jitter
 * without perturbing the PLL math. Industry standard approach (AS5048 / iC-MU
 * default 2–3 LSB on a 14-bit output, equivalent to 0.011–0.017° here).
 * Trade-off: motion below ~150 deg/s is reported in steps of the threshold. */
#define ENCODER_OUTPUT_DEADBAND_DEG (0.015f)

/* Output-stage hysteresis on the low-latency integer angle_counts: if the new
 * count differs from the last published one by at most this many LSBs in either
 * direction, hold the previous value. Float angle_deg remains the smooth
 * observer output used by monitoring and velocity estimation. */
#define ENCODER_ANGLE_COUNT_HYSTERESIS  (3)

/* Centre trim authority. Sized for board-to-board spread, not just drift: the
 * compiled-in defaults come from one board, and a centre that is wrong by a few
 * thousand counts destroys the angle rather than merely degrading it. The step
 * limit applies only after the first lock, where all that remains is thermal
 * drift -- acquisition takes the measured centre in one go. */
#define ENCODER_RUNTIME_TRIM_STEP_LIMIT_COUNTS (1.0f)
#define ENCODER_RUNTIME_TRIM_TOTAL_LIMIT_COUNTS (8192.0f)

/* AGC on T-matrix scale: drives the rotation-mean mag toward 1.0 to compensate
 * for analog amplitude drift between factory cal and runtime (temperature, OPAMP
 * gain, supply). Step limit caps per-revolution change; total limit prevents
 * runaway. +/-0.5 covers +/-50% amplitude drift.
 *
 * Note a common amplitude error is nearly harmless on its own -- it cancels in
 * atan2(sin, cos), measured at under 0.01 deg of extra error from 0.5x to 1.5x
 * (`encoder_sim bootstrap`). The AGC matters because magnitude is the health
 * signal everything else gates on, not because the angle needs it. */
#define ENCODER_RUNTIME_TRIM_GAIN_STEP_LIMIT  (0.02f)
#define ENCODER_RUNTIME_TRIM_GAIN_TOTAL_LIMIT (0.5f)

typedef struct _encoder_raw_sample
{
    uint16_t a1_sin_raw;
    uint16_t a1_cos_raw;
    uint16_t a2_sin_raw;
    uint16_t a2_cos_raw;
} encoder_raw_sample_t;

/* Lower-triangular ellipse correction: normalized = T * (raw - center).
 * The upper-right element is zero by Cholesky construction, so it is omitted. */
typedef struct _encoder_track_calibration
{
    float center_sin;
    float center_cos;
    float t00;
    float t10;
    float t11;
} encoder_track_calibration_t;

typedef struct _encoder_calibration
{
    encoder_track_calibration_t a1;
    encoder_track_calibration_t a2;
    float phase_a1_zero_deg;
    float phase_a2_zero_deg;
    bool valid;
} encoder_calibration_t;

typedef struct _encoder_cal_quality
{
    uint32_t sample_count;
    uint32_t status;
    float mag16;
    float mag15;
} encoder_cal_quality_t;

typedef struct _encoder_result
{
    float angle_deg;
    float angle_deg_raw;
    float angle_deg_filtered;
    float angular_velocity_dps;     /* tracking observer velocity output (deg/s) */
    uint32_t angle_counts;
    float phase16_deg;
    float phase15_deg;
    float coarse_deg;
    /* Disagreement between the coarse branch pick and the fine track, in fine-track
     * degrees. Approaches +/-180 exactly when the coarse estimate sits on a branch
     * boundary, so it is the branch-confidence signal -- and, binned by rotor
     * angle, the reference-free harmonic error measurement. */
    float fine_delta_deg;
    float mag16_raw;                /* instantaneous track-1 magnitude - AGC feedback + WEAK gate */
    float mag15_raw;                /* instantaneous track-2 magnitude - AGC feedback + WEAK gate */
    uint32_t status;
} encoder_result_t;

typedef struct _encoder_diag
{
    float angle_deg;
    float coarse_angle_deg;
    float phase16_error_deg;
    float phase15_error_deg;
    float phase_a1_deg;
    float phase_a2_deg;
} encoder_diag_t;

typedef struct _encoder_track_cal_stats
{
    uint32_t count;
    uint32_t rail_count;
    uint16_t min_sin;
    uint16_t max_sin;
    uint16_t min_cos;
    uint16_t max_cos;
    double sum_sin;
    double sum_cos;
    double sum_sin2;
    double sum_cos2;
    double sum_sincos;
    double sum_sin3;
    double sum_cos3;
    double sum_sin2cos;
    double sum_sincos2;
    double sum_sin4;
    double sum_cos4;
    double sum_sin3cos;
    double sum_sincos3;
    double sum_sin2cos2;
} encoder_track_cal_stats_t;

typedef struct _encoder_cal_stats
{
    encoder_track_cal_stats_t a1;
    encoder_track_cal_stats_t a2;
} encoder_cal_stats_t;

typedef struct _encoder_state
{
    float last_angle_deg;
    float last_angle_raw_deg;
    float filtered_angle_deg;       /* tracking observer angle state (theta_est) */
    float tracking_velocity_dps;    /* tracking observer velocity integrator (deg/s) */
    uint32_t last_angle_counts;
    uint32_t hold_last_streak;      /* consecutive bad samples — drives filter resync */
    bool has_valid_angle;
    bool filter_initialized;
} encoder_state_t;

typedef struct _encoder_runtime_trim
{
    bool enabled;
    bool active;
    bool has_locked;                /* true once first full-window update succeeded */
    float a1_center_sin_delta;
    float a1_center_cos_delta;
    float a2_center_sin_delta;
    float a2_center_cos_delta;
    float a1_gain_delta;            /* AGC: effective T row scaled by (1+delta) */
    float a2_gain_delta;
    float a1_raw_mag_sum;           /* AGC feedback: window-mean of raw instantaneous mag (track 1) */
    float a2_raw_mag_sum;
    uint32_t raw_mag_count;
    uint32_t solve_count;           /* windows that produced an update; acquisition progress */
    /* Consecutive samples the tracking gate has rejected. A lock made on bad data
     * would otherwise be permanent: the angle it produces is wrong, wrong angles
     * fail the gate, and a failing gate never re-measures. Long enough here and
     * the estimator drops back to acquisition and re-earns the lock. */
    uint32_t blocked_streak;
    /* Acquisition progress: how many times each channel has crossed its centre,
     * with a Schmitt band so a parked shaft dithering on the crossing does not
     * count. Two transitions is one full electrical cycle, which is what min/max
     * needs before it means the centre rather than the middle of an arc. */
    uint8_t cross_count[4];
    uint8_t cross_sign_mask;
    uint32_t window_count;
    uint32_t coverage_mask;
    uint16_t a1_sin_min;
    uint16_t a1_sin_max;
    uint16_t a1_cos_min;
    uint16_t a1_cos_max;
    uint16_t a2_sin_min;
    uint16_t a2_sin_max;
    uint16_t a2_cos_min;
    uint16_t a2_cos_max;
} encoder_runtime_trim_t;

void encoder_calibration_set_defaults(encoder_calibration_t *calibration);
void encoder_calibration_set_board_defaults(encoder_calibration_t *calibration);
void encoder_cal_stats_init(encoder_cal_stats_t *stats);
void encoder_cal_stats_accumulate(encoder_cal_stats_t *stats, const encoder_raw_sample_t *sample);
bool encoder_cal_stats_build(const encoder_cal_stats_t *stats,
                             encoder_calibration_t *calibration,
                             uint32_t *status);
bool encoder_capture_zero(encoder_calibration_t *calibration,
                          const encoder_raw_sample_t *sample,
                          uint32_t *status);
void encoder_state_init(encoder_state_t *state);
void encoder_runtime_trim_init(encoder_runtime_trim_t *trim);
void encoder_runtime_trim_reset(encoder_runtime_trim_t *trim);
void encoder_runtime_trim_apply(const encoder_calibration_t *factory,
                                const encoder_runtime_trim_t *trim,
                                encoder_calibration_t *effective);
void encoder_runtime_trim_update(encoder_runtime_trim_t *trim,
                                 const encoder_calibration_t *factory,
                                 const encoder_raw_sample_t *sample,
                                 const encoder_result_t *result);
void encoder_process(encoder_state_t *state,
                     const encoder_calibration_t *calibration,
                     const encoder_raw_sample_t *sample,
                     encoder_result_t *result,
                     encoder_diag_t *diag);

#endif /* APP_ENCODER_H_ */
