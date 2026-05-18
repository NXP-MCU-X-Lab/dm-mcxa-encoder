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

/* Type-II tracking observer (software PLL) for published angle, matching the
 * canonical resolver-to-digital converter loop (AD2S1210 / TI SPRAA94 / etc.).
 * - BW   sets closed-loop bandwidth in Hz.
 * - ZETA sets damping ratio (0.707 = Butterworth, no overshoot).
 * Kp/Ki are precomputed from BW, ZETA, and the ADC sample period at compile time. */
#define ENCODER_TRACKING_BW_HZ      (100.0f)
#define ENCODER_TRACKING_ZETA       (0.707f)
#define ENCODER_RUNTIME_TRIM_STEP_LIMIT_COUNTS (1.0f)
#define ENCODER_RUNTIME_TRIM_TOTAL_LIMIT_COUNTS (512.0f)

#define ENCODER_RUNTIME_TRIM_FREEZE_NONE (0UL)
#define ENCODER_RUNTIME_TRIM_FREEZE_DISABLED (1UL)
#define ENCODER_RUNTIME_TRIM_FREEZE_NOT_CALIBRATED (2UL)
#define ENCODER_RUNTIME_TRIM_FREEZE_STATUS (3UL)
#define ENCODER_RUNTIME_TRIM_FREEZE_COVERAGE (4UL)
#define ENCODER_RUNTIME_TRIM_FREEZE_MAGNITUDE (5UL)

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
    float mag16;
    float mag15;
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
    bool has_valid_angle;
    bool filter_initialized;
} encoder_state_t;

typedef struct _encoder_runtime_trim
{
    bool enabled;
    bool active;
    uint32_t freeze_reason;
    uint32_t accepted_samples;
    uint32_t update_count;
    float a1_center_sin_delta;
    float a1_center_cos_delta;
    float a2_center_sin_delta;
    float a2_center_cos_delta;
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
