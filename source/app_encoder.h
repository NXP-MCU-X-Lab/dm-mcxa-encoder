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

#define ENCODER_MAPPING_A1_16_A2_15 (0U)
#define ENCODER_MAPPING_A1_15_A2_16 (1U)

#ifndef ENCODER_CONFIG_MAPPING
#define ENCODER_CONFIG_MAPPING ENCODER_MAPPING_A1_16_A2_15
#endif

#ifndef ENCODER_CONFIG_DIR16
#define ENCODER_CONFIG_DIR16 (1)
#endif

#ifndef ENCODER_CONFIG_DIR15
#define ENCODER_CONFIG_DIR15 (1)
#endif

#ifndef ENCODER_ROUGH_ANGLE_ENABLE
#define ENCODER_ROUGH_ANGLE_ENABLE (0U)
#endif

#ifndef ENCODER_FILTER_ENABLE
#define ENCODER_FILTER_ENABLE (1U)
#endif

#ifndef ENCODER_FILTER_ALPHA
#define ENCODER_FILTER_ALPHA (0.25f)
#endif

typedef struct _encoder_raw_sample
{
    uint16_t a1_sin_raw;
    uint16_t a1_cos_raw;
    uint16_t a2_sin_raw;
    uint16_t a2_cos_raw;
} encoder_raw_sample_t;

typedef struct _encoder_track_calibration
{
    float center_sin;
    float center_cos;
    float transform[2][2];
} encoder_track_calibration_t;

typedef struct _encoder_calibration
{
    encoder_track_calibration_t a1;
    encoder_track_calibration_t a2;
    float phase_a1_zero_deg;
    float phase_a2_zero_deg;
    bool valid;
} encoder_calibration_t;

typedef struct _encoder_result
{
    float angle_deg;
    float angle_deg_raw;
    float angle_deg_filtered;
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
    uint8_t mapping;
    int8_t dir16;
    int8_t dir15;
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

typedef struct _encoder_rough_track_state
{
    bool initialized;
    uint16_t min_sin;
    uint16_t max_sin;
    uint16_t min_cos;
    uint16_t max_cos;
} encoder_rough_track_state_t;

typedef struct _encoder_state
{
    float last_angle_deg;
    float last_angle_raw_deg;
    float filtered_angle_deg;
    uint32_t last_angle_counts;
    bool has_valid_angle;
    bool filter_initialized;
    encoder_rough_track_state_t rough_a1;
    encoder_rough_track_state_t rough_a2;
} encoder_state_t;

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
void encoder_process(encoder_state_t *state,
                     const encoder_calibration_t *calibration,
                     const encoder_raw_sample_t *sample,
                     encoder_result_t *result);
void encoder_process_with_diag(encoder_state_t *state,
                               const encoder_calibration_t *calibration,
                               const encoder_raw_sample_t *sample,
                               encoder_result_t *result,
                               encoder_diag_t *diag);

#endif /* APP_ENCODER_H_ */
