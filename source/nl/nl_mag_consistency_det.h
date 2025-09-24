/**
 * @file nl_mag_consistency_det.h
 * @brief Magnetic field consistency detector interface
 */
#ifndef NL_MAG_CONSISTENCY_DET_H
#define NL_MAG_CONSISTENCY_DET_H

#include "nl.h"
#include "nl_stats.h"



/* Detector status flags */
#define NL_MAG_STATUS_HOMOGENEOUS           (1 << 0)  /* Magnetic field is homogeneous */
#define NL_MAG_STATUS_SUFFICIENT_DATA       (1 << 1)  /* Sufficient data for analysis */
#define NL_MAG_STATUS_SUFFICIENT_ROTATION   (1 << 2) /* Sufficient rotation occurred */
#define NL_MAG_STATUS_FIELD_STABLE          (1 << 3)  /* Field strength stable (no abrupt change) */

// Detector structure
typedef struct {
    nl_t mag_b_prev[3];             // Previous magnetic field vector
    nl_t q_prev[4];                 // Previous quaternion
    nl_t q_delta[4];
    nl_t residual;                  // Current residual
    nl_t residual_mean;
    nl_t residual_variance;
    
    // Sliding window statistics for residual
    nl_stats_window_t *residual_stats;
    
    void *user_data;                // User data for callback
} nl_mag_cons_detector_t;

/**
 * @brief Initialize magnetic field consistency detector
 * @param det Detector instance
 * @param ref_flux Calibrated magnetic field strength
 * @param window_size Residual statistics window size (0 to disable)
 */
void nl_mag_cons_init(nl_mag_cons_detector_t *det, int window_size);

/**
 * @brief Update detector with new measurements
 * @param det Detector instance
 * @param q_curr Current quaternion
 * @param mag_b_curr Current magnetic field vector
 * @return 0 on success, negative on error
 */
int nl_mag_cons_update(nl_mag_cons_detector_t *det, nl_t *q_curr, nl_t *mag_b_curr);



#endif /* NL_MAG_CONSISTENCY_DET_H */
