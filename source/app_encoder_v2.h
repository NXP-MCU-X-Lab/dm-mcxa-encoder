#ifndef APP_ENCODER_V2_H_
#define APP_ENCODER_V2_H_

#include <stdbool.h>
#include <stdint.h>

#define V2_ENCODER_TRACK16_CYCLES (16U)
#define V2_ENCODER_TRACK15_CYCLES (15U)
#define V2_ENCODER_CAL_SAMPLE_COUNT (8192U)
#define V2_ENCODER_COUNTS_PER_REV (65536U)

#define V2_ENCODER_STATUS_OK (0UL)
#define V2_ENCODER_STATUS_NOT_CALIBRATED (1UL << 0)
#define V2_ENCODER_STATUS_TRACK16_WEAK (1UL << 1)
#define V2_ENCODER_STATUS_TRACK15_WEAK (1UL << 2)
#define V2_ENCODER_STATUS_ADC_RAIL (1UL << 3)
#define V2_ENCODER_STATUS_TRACK_MISMATCH (1UL << 4)
#define V2_ENCODER_STATUS_CAL_FAILED (1UL << 5)
#define V2_ENCODER_STATUS_HOLD_LAST (1UL << 6)

#define V2_ENCODER_MAPPING_A1_16_A2_15 (0U)
#define V2_ENCODER_MAPPING_A1_15_A2_16 (1U)

#ifndef V2_ENCODER_CONFIG_MAPPING
#define V2_ENCODER_CONFIG_MAPPING V2_ENCODER_MAPPING_A1_16_A2_15
#endif

#ifndef V2_ENCODER_CONFIG_DIR16
#define V2_ENCODER_CONFIG_DIR16 (1)
#endif

#ifndef V2_ENCODER_CONFIG_DIR15
#define V2_ENCODER_CONFIG_DIR15 (1)
#endif

#ifndef V2_ENCODER_ROUGH_ANGLE_ENABLE
#define V2_ENCODER_ROUGH_ANGLE_ENABLE (0U)
#endif

typedef struct _v2_encoder_raw_sample
{
    uint16_t a1_sin_raw;
    uint16_t a1_cos_raw;
    uint16_t a2_sin_raw;
    uint16_t a2_cos_raw;
} v2_encoder_raw_sample_t;

typedef struct _v2_encoder_track_calibration
{
    float center_sin;
    float center_cos;
    float transform[2][2];
} v2_encoder_track_calibration_t;

typedef struct _v2_encoder_calibration
{
    v2_encoder_track_calibration_t a1;
    v2_encoder_track_calibration_t a2;
    float phase_a1_zero_deg;
    float phase_a2_zero_deg;
    bool valid;
} v2_encoder_calibration_t;

typedef struct _v2_encoder_result
{
    float angle_deg;
    uint32_t angle_counts;
    float phase16_deg;
    float phase15_deg;
    float coarse_deg;
    float mag16;
    float mag15;
    uint32_t status;
} v2_encoder_result_t;

typedef struct _v2_encoder_diag
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
} v2_encoder_diag_t;

typedef struct _v2_encoder_track_cal_stats
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
} v2_encoder_track_cal_stats_t;

typedef struct _v2_encoder_cal_stats
{
    v2_encoder_track_cal_stats_t a1;
    v2_encoder_track_cal_stats_t a2;
} v2_encoder_cal_stats_t;

typedef struct _v2_encoder_rough_track_state
{
    bool initialized;
    uint16_t min_sin;
    uint16_t max_sin;
    uint16_t min_cos;
    uint16_t max_cos;
} v2_encoder_rough_track_state_t;

typedef struct _v2_encoder_state
{
    float last_angle_deg;
    uint32_t last_angle_counts;
    bool has_valid_angle;
    v2_encoder_rough_track_state_t rough_a1;
    v2_encoder_rough_track_state_t rough_a2;
} v2_encoder_state_t;

void v2_encoder_calibration_set_defaults(v2_encoder_calibration_t *calibration);
void v2_encoder_cal_stats_init(v2_encoder_cal_stats_t *stats);
void v2_encoder_cal_stats_accumulate(v2_encoder_cal_stats_t *stats, const v2_encoder_raw_sample_t *sample);
bool v2_encoder_cal_stats_build(const v2_encoder_cal_stats_t *stats,
                                v2_encoder_calibration_t *calibration,
                                uint32_t *status);
bool v2_encoder_capture_zero(v2_encoder_calibration_t *calibration,
                             const v2_encoder_raw_sample_t *sample,
                             uint32_t *status);
void v2_encoder_state_init(v2_encoder_state_t *state);
void v2_encoder_process(v2_encoder_state_t *state,
                        const v2_encoder_calibration_t *calibration,
                        const v2_encoder_raw_sample_t *sample,
                        v2_encoder_result_t *result);
void v2_encoder_process_with_diag(v2_encoder_state_t *state,
                                  const v2_encoder_calibration_t *calibration,
                                  const v2_encoder_raw_sample_t *sample,
                                  v2_encoder_result_t *result,
                                  v2_encoder_diag_t *diag);

/* Legacy result layout retained for FreeMASTER pmpx compatibility.
 * Populated from v2_encoder_result_t via copy_v2_to_legacy_result() in main.c. */
typedef struct {
    uint16_t sin_raw;
    uint16_t cos_raw;
    float sin_norm;
    float cos_norm;
    float magnitude;
    float elec_angle_deg;
    float angle_deg;
    int32_t turns;
    uint16_t angle_counts;
    float speed_dps;
    float speed_rpm;
} encoder_result_t;

#endif /* APP_ENCODER_V2_H_ */
