/**
 * @file nl_acc_analyzer.c
 * @brief Acceleration Analyzer Implementation
 * @author Alex Yang
 * @version 1.0
 * @date 2025
 */

#include "nl_acc_analyzer.h"
#include <string.h>

/* Fixed configuration parameters */
#define JERK_THRESHOLD          (0.01f)     /* Jerk threshold for uniform detection [m/s³] */
#define ACC_MIN_THRESHOLD       (0.3f)      /* Minimum acceleration for detection [m/s²] */
#define STABLE_COUNT_REQ        (200)       /* Required stable count (0.2s @ 1kHz) */
#define FILTER_FC               (2.0f)      /* Filter cutoff frequency [Hz] */
#define NACC_BUF_SIZE           (20)


/* Acceleration variation detection thresholds */
#define ACC_VAR_HIGH_THRESHOLD   (0.02f)    /* High variance threshold for acceleration detection */
#define ACC_VAR_LOW_THRESHOLD    (0.005f)    /* Low variance threshold for stable state */
#define ACC_VAR_STABLE_COUNT     (500)       /* Required stable low variance count */

int nl_acc_analyzer_init(nl_acc_analyzer_t *analyzer, nl_t sample_freq)
{
    if (!analyzer || sample_freq <= 0) return -1;
    
    /* Clear all state */
    memset(analyzer, 0, sizeof(nl_acc_analyzer_t));
    
    /* Initialize biquad lowpass filters for navigation frame only */
    for (int i = 0; i < 3; i++) {
        biquad_lowpass_init(&analyzer->filters_n[i], FILTER_FC, sample_freq);
    }
    
    analyzer->stats_acc = nl_stats_window_create(NACC_BUF_SIZE);
    
    return 0;
}

uint8_t nl_acc_analyzer_update(nl_acc_analyzer_t *analyzer, nl_t *q, nl_t *acc_n)
{
    if (!analyzer || !q || !acc_n) return 0;
    
    /* === Acceleration filtering === */
    
    /* Apply biquad lowpass filters to navigation frame data */
    for (int i = 0; i < 3; i++) {
        analyzer->acc_flt_n[i] = biquad_update(&analyzer->filters_n[i], acc_n[i]);
    }
    
    /* Transform filtered navigation frame acceleration to body frame */
    nl_t Qn2b[4];
    qconj(q, Qn2b);
    qmulv(Qn2b, analyzer->acc_flt_n, analyzer->acc_flt_b);
    
    /* === Uniform acceleration detection === */
    
    /* Use filtered acceleration for detection */
    nl_t current_acc[3];
    v3copy(current_acc, analyzer->acc_flt_n);
    current_acc[2] -= GRAVITY; /* Remove gravity component */
    
    analyzer->acc_n_remove_gravity_norm = v3norm(current_acc);
    
    if(analyzer->acc_n_remove_gravity_norm > analyzer->horizontal_acc_n_norm_envelope )
    {
        analyzer->horizontal_acc_n_norm_envelope  = analyzer->acc_n_remove_gravity_norm;
    }
    else
    {
        analyzer->horizontal_acc_n_norm_envelope  += 0.005 * (analyzer->acc_n_remove_gravity_norm - analyzer->horizontal_acc_n_norm_envelope);
    }

    /* we only care about honrional acc std */
    analyzer->horizontal_acc_n_norm = sqrt(POW2(analyzer->acc_flt_n[0]) + POW2(analyzer->acc_flt_n[1]));

    nl_stats_window_update(analyzer->stats_acc, analyzer->horizontal_acc_n_norm);
    
    /* === Acceleration variation detection === */
    analyzer->horizontal_acc_n_norm_var = nl_stats_window_get_variance(analyzer->stats_acc);
    
    if (analyzer->horizontal_acc_n_norm_var > ACC_VAR_HIGH_THRESHOLD) {
        /* High variance detected - set acceleration flag and reset counter */
        analyzer->is_in_acceleration = 1;
        analyzer->low_var_counter = 0;
    } else if (analyzer->horizontal_acc_n_norm_var < ACC_VAR_LOW_THRESHOLD) {
        /* Low variance - increment stable counter */
        analyzer->low_var_counter++;
        if (analyzer->low_var_counter >= ACC_VAR_STABLE_COUNT) {
            analyzer->is_in_acceleration = 0;
        }
    } else {
        /* In between thresholds - reset counter but keep current state */
        analyzer->low_var_counter = 0;
    }
    
    /* Only detect when significant acceleration is present */
    if (analyzer->acc_n_remove_gravity_norm < ACC_MIN_THRESHOLD) {
        analyzer->stable_counter = 0;
        analyzer->is_uniform_acc = 0;
    } else {
        if (!analyzer->initialized) {
            /* First sample - initialize previous acceleration */
            v3copy(analyzer->prev_acc, current_acc);
            analyzer->initialized = 1;
        } else {
            /* Calculate jerk (acceleration derivative) */
            nl_t jerk[3];
            v3sub(jerk, current_acc, analyzer->prev_acc);
            analyzer->jerk_norm = v3norm(jerk);
            
            /* Check for uniform acceleration conditions */
            if (analyzer->jerk_norm < JERK_THRESHOLD && analyzer->acc_n_remove_gravity_norm > ACC_MIN_THRESHOLD) {
                analyzer->stable_counter++;
                if (analyzer->stable_counter > STABLE_COUNT_REQ) {
                    analyzer->is_uniform_acc = 1;
                }
            } else {
                analyzer->stable_counter = 0;
                analyzer->is_uniform_acc = 0;
            }
            
            /* Update acceleration history */
            v3copy(analyzer->prev_acc, current_acc);
        }
    }
    
    uint8_t status = 0;
    if (analyzer->is_uniform_acc) status |= NL_ACC_STATUS_UNIFORM;
    if (analyzer->is_in_acceleration) status |= NL_ACC_STATUS_IN_MOTION;
    if (analyzer->initialized) status |= NL_ACC_STATUS_INITIALIZED;
    
    return status;
}


