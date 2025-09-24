/**
 * @file nl_zaru.h
 * @brief Zero Angular Rate Update (ZARU) detector interface
 */
#ifndef NL_ZARU_H
#define NL_ZARU_H

#include "nl.h"
#include "nl_stats.h"
#include "nl_filter.h"

// ZARU detector states
typedef enum {
    ZARU_STATE_INIT,
    ZARU_STATE_PROGRESS,
    ZARU_STATE_FINIAL
} nl_zaru_state_t;

// ZARU event types
typedef enum {
    ZARU_EVENT_STATIC_DETECTED,
    ZARU_EVENT_MOTION_DETECTED
} nl_zaru_event_t;

// Event callback function type
typedef void (*nl_zaru_event_cb_t)(void *user_data, nl_zaru_event_t event, nl_t *bias);

// ZARU detector structure
typedef struct {
    // Statistics for each axis
    nl_stats_basic_t stat_gyr[3];
    
    // State variables
    nl_zaru_state_t state;
    uint8_t enable;
    uint8_t is_static;
    uint8_t is_first_wb_get;
    
    uint32_t dt_ms;
    nl_t detection_threshold;
    
    // Measurements
    nl_t gyr_lp[3];
    
    // Bias estimation
    nl_t initial_wb[3];
    nl_t wb[3];
    nl_t tmp[3];
    
    // Detection counters
    uint8_t win_counter;
    uint8_t det_win_count;
    uint32_t static_duration_ms;
    
    
    // Callback
    nl_zaru_event_cb_t event_cb;
    
    biquad_filter_t gyr_lp_filter[3];
    nl_t filter_cutoff_hz;
    
    void *user_data;
} nl_zaru_detector_t;

void nl_zaru_init(nl_zaru_detector_t *det, uint32_t dt);
void nl_zaru_update(nl_zaru_detector_t *det, nl_t *acc, nl_t *gyr);
void nl_zaru_set_enable(nl_zaru_detector_t *det, uint8_t enable);
void nl_zaru_set_event_callback(nl_zaru_detector_t *det, nl_zaru_event_cb_t cb, void *user_data);
void nl_zaru_set_filter_cutoff(nl_zaru_detector_t *det, nl_t cutoff_hz);
nl_t nl_zaru_get_filter_cutoff(nl_zaru_detector_t *det);
void nl_zaru_force_motion(nl_zaru_detector_t *det);

#endif // NL_ZARU_H
