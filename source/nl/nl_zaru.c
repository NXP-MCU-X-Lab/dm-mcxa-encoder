/**
 * @file nl_zaru.c
 * @brief Zero Angular Rate Update (ZARU) detector implementation
 */

#include "nl_zaru.h"
#include <string.h>

#define NL_DBG_TAG    "nl.zaru"
#define NL_DBG_LVL    NL_DBG_ERROR
#include "nl_log.h"

// Detection parameters
#define ZARU_WINDOW_SIZE_MS             (350)
#define DEFAULT_ZARU_THR                (0.3 * D2R)

/**
 * @brief Check if statistics window is full
 */
static bool is_window_full(const nl_zaru_detector_t *det)
{
    return det->stat_gyr[0].count >= (ZARU_WINDOW_SIZE_MS / det->dt_ms);
}

/**
 * @brief Check static conditions using gyroscope data
 */
static bool is_static_condition(nl_zaru_detector_t *det, nl_t *gyr)
{
    nl_t residual[3];
    nl_t detection_threshold = det->detection_threshold*0.8;
    
    v3sub(residual, gyr, det->wb);
    
    if(!det->is_first_wb_get)
    {
        detection_threshold *= 2;
    }

    if(fabs(residual[0]) > detection_threshold || fabs(residual[1]) > detection_threshold || fabs(residual[2]) > detection_threshold)
    {
        return false;
    }

    return true;
}

/**
 * @brief Handle motion detection event
 */
static void handle_motion_detected(nl_zaru_detector_t *det)
{
    if(det->is_static)
    {
        NL_LOG_D("motion detected");
    }
    
    det->state = ZARU_STATE_INIT;
    det->is_static = 0;
    det->static_duration_ms = 0;
    if (!det->is_static && det->event_cb) {
        det->event_cb(det->user_data, ZARU_EVENT_MOTION_DETECTED, det->wb);
    }
}

/**
 * @brief Update bias estimation in PROGRESS state
 */
static bool update_bias_progress(nl_zaru_detector_t *det)
{
    for (int i = 0; i < 3; i++) {
        nl_t curr_mean = nl_stats_basic_get_mean(&det->stat_gyr[i]);
        if (fabs(curr_mean - det->initial_wb[i]) >= det->detection_threshold / 3.75) {
            NL_LOG_D("REJECT:%.3f", fabs(curr_mean - det->initial_wb[i])*R2D);
            return false;
        }
        det->tmp[i] += curr_mean;
    }
    return true;
}

/**
 * @brief Set ZARU static detection time in milliseconds
 */
static int nl_zaru_set_det_thr_time(nl_zaru_detector_t *det, uint32_t ms)
{
    if (!det || ms < ZARU_WINDOW_SIZE_MS) {
        return -1;
    }
    
    uint32_t window_count = (ms + ZARU_WINDOW_SIZE_MS - 1) / ZARU_WINDOW_SIZE_MS;
    if (window_count < 3)
    {
        NL_LOG_W("det_win_count too low:%d", det->det_win_count);
        window_count = 3;
    }        
    
    det->det_win_count = window_count;
    NL_LOG_D("det_win_count:%d", det->det_win_count);
    
    return 0;
}

/**
 * @brief Initialize ZARU detector
 */
void nl_zaru_init(nl_zaru_detector_t *det, uint32_t dt_ms)
{
    memset(det, 0, sizeof(nl_zaru_detector_t));
    
    det->static_duration_ms = 0;
    det->dt_ms = dt_ms;
    det->enable = 1;
    det->state = ZARU_STATE_INIT;
    det->filter_cutoff_hz = 4.0f;
    det->detection_threshold = DEFAULT_ZARU_THR;
    det->is_static = 1;
    det->is_first_wb_get = 0;
    det->static_duration_ms = 9999;
    
    nl_zaru_set_det_thr_time(det, 1750);

    for (int i = 0; i < 3; i++) {
        nl_stats_basic_reset(&det->stat_gyr[i]);
    }

    biquad_lowpass_init(&det->gyr_lp_filter[0], det->filter_cutoff_hz, 1000.0 / det->dt_ms);
    biquad_lowpass_init(&det->gyr_lp_filter[1], det->filter_cutoff_hz, 1000.0 / det->dt_ms);
    biquad_lowpass_init(&det->gyr_lp_filter[2], det->filter_cutoff_hz, 1000.0 / det->dt_ms);
}

/**
 * @brief Update ZARU detector with new measurements
 */
void nl_zaru_update(nl_zaru_detector_t *det, nl_t *acc, nl_t *gyr)
{
    // Quick motion detection
    det->gyr_lp[0] = biquad_update(&det->gyr_lp_filter[0], gyr[0]);
    det->gyr_lp[1] = biquad_update(&det->gyr_lp_filter[1], gyr[1]);
    det->gyr_lp[2] = biquad_update(&det->gyr_lp_filter[2], gyr[2]);
    
    if(!is_static_condition(det, det->gyr_lp))
    {
        handle_motion_detected(det);
        return;
    }
    
    // NL_LOG_D("%.3f, %.3f, %.3f", det->gyr_lp[0]*R2D, det->gyr_lp[1]*R2D, det->gyr_lp[2]*R2D);
    
    for (int i = 0; i < 3; i++) {
        nl_stats_basic_update(&det->stat_gyr[i], det->gyr_lp[i]);
    }
    
    if (!is_window_full(det)) {
        return;
    }
    
    // Static detection state machine
    if (det->enable) 
    {
        switch (det->state) {
            case ZARU_STATE_INIT:
                // Initialize bias estimation
                for (int i = 0; i < 3; i++) {
                    det->initial_wb[i] = nl_stats_basic_get_mean(&det->stat_gyr[i]);
                }
                det->state = ZARU_STATE_PROGRESS;
                det->win_counter = 0;
                v3fill(det->tmp, 0);
                break;
                
            case ZARU_STATE_PROGRESS:
                if (update_bias_progress(det)) {
                    det->win_counter++;
                    if (det->win_counter >= det->det_win_count) {
                        det->state = ZARU_STATE_FINIAL;
                    }
                } else {
                    det->state = ZARU_STATE_INIT;
                }
                break;
                
            case ZARU_STATE_FINIAL:
                if (update_bias_progress(det)) {
                    det->win_counter++;
                    v3scale2(det->tmp, 1.0f / det->win_counter);
                    
                    NL_LOG_D("ZARU_STATE_FINIAL, [%d],wb:%.3f,%.3f,%.3f", nl_get_sys_ms()/1000, det->tmp[0]*R2D, det->tmp[1]*R2D, det->tmp[2]*R2D);
                    
                    det->static_duration_ms += det->win_counter * ZARU_WINDOW_SIZE_MS;
                    det->is_static = 1;
                    
                    v3copy(det->wb, det->tmp);

                    if(!det->is_first_wb_get) {
                        det->is_first_wb_get = 1;  
                    }
                    
                    if (det->event_cb) {
                        det->event_cb(det->user_data, ZARU_EVENT_STATIC_DETECTED, det->wb);
                    }
                }
                
                det->state = ZARU_STATE_INIT;
                det->win_counter = 0;
                break;
        }
    }
    else
    {
        handle_motion_detected(det);
    }
    
    // Reset statistics after processing
    if (is_window_full(det)) {
        for (int i = 0; i < 3; i++) {
            nl_stats_basic_reset(&det->stat_gyr[i]);
        }
    }
    
}

void nl_zaru_set_enable(nl_zaru_detector_t *det, uint8_t enable)
{
    det->enable = enable;
    handle_motion_detected(det);
}

/**
 * @brief Set ZARU filter cutoff frequency in Hz
 */
void nl_zaru_set_filter_cutoff(nl_zaru_detector_t *det, nl_t cutoff_hz)
{
    if (!det || cutoff_hz <= 0) {
        return;
    }
    
    det->filter_cutoff_hz = cutoff_hz;
    
    biquad_lowpass_init(&det->gyr_lp_filter[0], det->filter_cutoff_hz, 1000.0 / det->dt_ms);
    biquad_lowpass_init(&det->gyr_lp_filter[1], det->filter_cutoff_hz, 1000.0 / det->dt_ms);
    biquad_lowpass_init(&det->gyr_lp_filter[2], det->filter_cutoff_hz, 1000.0 / det->dt_ms);
    
    NL_LOG_D("Filter cutoff set to %.2f Hz", det->filter_cutoff_hz);
}

/**
 * @brief Get ZARU filter cutoff frequency in Hz
 */
nl_t nl_zaru_get_filter_cutoff(nl_zaru_detector_t *det)
{
    if (!det) return 0;
    return det->filter_cutoff_hz;
}


void nl_zaru_set_event_callback(nl_zaru_detector_t *det, nl_zaru_event_cb_t cb, void *user_data)
{
    det->event_cb = cb;
    det->user_data = user_data;
}


void nl_zaru_force_motion(nl_zaru_detector_t *det)
{
    handle_motion_detected(det);
}
