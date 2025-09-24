/**
 * @file nl_acc_analyzer.h
 * @brief Acceleration Analyzer for VRU Algorithm - Uniform Acceleration Detection & Filtering
 * @author Alex Yang
 * @version 1.0
 * @date 2025
 */

#ifndef _NL_ACC_ANALYZER_H
#define _NL_ACC_ANALYZER_H

#include "nl_filter.h"
#include "nl_stats.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Analyzer status flags */
#define NL_ACC_STATUS_UNIFORM       (1 << 0)  /* Uniform acceleration detected */
#define NL_ACC_STATUS_IN_MOTION     (1 << 1)  /* In acceleration motion */
#define NL_ACC_STATUS_INITIALIZED   (1 << 2)  /* Analyzer initialized */

/**
 * @brief Acceleration analyzer state structure
 */
typedef struct {
    /* Uniform acceleration detector state */
    uint32_t stable_counter;        /* Stable sample counter */
    uint32_t low_var_counter;       /* Low variance stable counter */
    uint8_t is_uniform_acc;         /* Uniform acceleration flag */
    uint8_t is_in_acceleration;
    nl_t prev_acc[3];               /* Previous acceleration for jerk calculation */
    uint8_t initialized;            /* Detector initialization flag */
    
    /* Acceleration filters */
    biquad_filter_t filters_n[3];   /* Navigation frame filters (X,Y,Z) */
    
    nl_stats_window_t *stats_acc;
    
    /* Output data */
    nl_t acc_flt_n[3];             /* Filtered acceleration in navigation frame */
    nl_t acc_flt_b[3];             /* Filtered acceleration in body frame */
    nl_t jerk_norm;                /* Current jerk magnitude */
    nl_t acc_n_remove_gravity_norm;
    nl_t horizontal_acc_n_norm;
    nl_t horizontal_acc_n_norm_var;
    nl_t horizontal_acc_n_norm_envelope;
} nl_acc_analyzer_t;

/**
 * @brief Initialize acceleration analyzer
 * @param analyzer Pointer to analyzer structure
 * @param sample_freq Sampling frequency in Hz
 * @return 0 on success, -1 on error
 */
int nl_acc_analyzer_init(nl_acc_analyzer_t *analyzer, nl_t sample_freq);

/**
 * @brief Update acceleration analyzer with new data
 * @param analyzer Pointer to analyzer structure
 * @param q Quaternion for coordinate transformation
 * @param acc_n Acceleration in navigation frame [m/s²]
 */
uint8_t nl_acc_analyzer_update(nl_acc_analyzer_t *analyzer, nl_t *q, nl_t *acc_n);


#ifdef __cplusplus
}
#endif

#endif /* _NL_ACC_ANALYZER_H */
