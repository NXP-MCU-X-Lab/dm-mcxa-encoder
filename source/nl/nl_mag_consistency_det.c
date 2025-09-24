/**
 * @file nl_mag_consistency_det.c
 * @brief Magnetic field consistency detector implementation
 */

#include "nl_mag_consistency_det.h"
#include <string.h>

#define NL_DBG_TAG    "nl.mag_cons"
#define NL_DBG_LVL    NL_DBG_ERROR
#include "nl_log.h"

// Detection parameters
#define MAG_CONS_MIN_ROTATION_THRESHOLD  (0.9992f)  // Corresponds to about 10 degrees
#define MAG_CONS_FIELD_CHANGE_THRESHOLD  (0.05f)    // % Abrupt change in magnetic field strength
#define MAG_CONS_RESIDUAL_VAR_THRESHOLD  (0.0001f)  // Residual variance threshold
#define MAG_CONS_RESIDUAL_THRESHOLD      (0.05f)

/**
 * @brief Initialize magnetic field consistency detector
 */
void nl_mag_cons_init(nl_mag_cons_detector_t *det, int window_size)
{
    memset(det, 0, sizeof(nl_mag_cons_detector_t));
    v3fill(det->mag_b_prev, 0);
    qidentity(det->q_prev);
    
    // Initialize sliding window statistics
    if (window_size > 0) {
        det->residual_stats = nl_stats_window_create(window_size);
        if (!det->residual_stats) {
            NL_LOG_E("Failed to create residual statistics window");
        }
    } else {
        det->residual_stats = RT_NULL;
    }
}

static int should_update(nl_mag_cons_detector_t *det, nl_t *q_curr)
{
    nl_t q_inv[4];
    
    // Calculate attitude change between current and previous quaternion
    qconj(q_curr, q_inv);
    qmul(q_inv, det->q_prev, det->q_delta);
    
    // Check if rotation is sufficient
    if (fabs(det->q_delta[0]) < MAG_CONS_MIN_ROTATION_THRESHOLD) {
        // Sufficient attitude change detected
        vcopy(det->q_prev, q_curr, 4);
        return 1;
    }
    
    return 0; // Insufficient attitude change
}

/**
 * @brief Update detector with new measurements
 */
int nl_mag_cons_update(nl_mag_cons_detector_t *det, nl_t *q_curr, nl_t *mag_b_curr)
{
    nl_t mag_rot[3];
    uint8_t status = 0;

    if(!should_update(det, q_curr)) {
        // Insufficient rotation - return only this flag cleared
        return status;
    }
    
    status |= NL_MAG_STATUS_SUFFICIENT_ROTATION;
    
    // Rotate previous magnetic field vector to current orientation
    qmulv(det->q_delta, det->mag_b_prev, mag_rot);
    
    // Calculate norms
    nl_t flux_curr = v3norm(mag_b_curr);
    nl_t flux_prev = v3norm(mag_rot);
    
    // Check for abrupt changes in magnetic field strength
    if (flux_curr < 0.01f || flux_prev < 0.01f || 
        (fabs(flux_prev - flux_curr) / flux_curr) > MAG_CONS_FIELD_CHANGE_THRESHOLD) {
        
        NL_LOG_D("Magnetic field strength changed abruptly: prev=%.2f, curr=%.2f, diff=%.2f", 
                flux_prev, flux_curr, fabs(flux_prev - flux_curr));
                
        v3copy(det->mag_b_prev, mag_b_curr);
        
        // Reset statistics window on field disturbance
        if (det->residual_stats) {
            nl_stats_window_reset(det->residual_stats);
        }
        return status; // FIELD_STABLE bit remains 0
    }

    status |= NL_MAG_STATUS_FIELD_STABLE;

    nl_t diff[3];
    
    nl_t mag_b_rot_norm[3], mag_curr_norm[3];
    
    v3copy(mag_b_rot_norm, mag_rot);
    v3copy(mag_curr_norm, mag_b_curr);
    
    v3normlz(mag_b_rot_norm);
    v3normlz(mag_curr_norm);
    
    v3sub(diff, mag_b_rot_norm, mag_curr_norm);
    det->residual = v3norm(diff);

    // Update residual statistics
    if (det->residual_stats) {
        nl_stats_window_update(det->residual_stats, det->residual);
    }

    // Update state
    v3copy(det->mag_b_prev, mag_b_curr);
    
    // Check homogeneity and update status
    if (det->residual_stats && det->residual_stats->count >= 5) {
        status |= NL_MAG_STATUS_SUFFICIENT_DATA;
        
        det->residual_mean = nl_stats_window_get_mean(det->residual_stats);
        det->residual_variance = nl_stats_window_get_variance(det->residual_stats);
        
        NL_LOG_D("residual: %.3f, mean: %.3f, var: %.4f", det->residual, det->residual_mean, det->residual_variance);
        
        if (det->residual_variance <= MAG_CONS_RESIDUAL_VAR_THRESHOLD &&
            det->residual < MAG_CONS_RESIDUAL_THRESHOLD) 
        {
            status |= NL_MAG_STATUS_HOMOGENEOUS;
        }
    }
    
    return status;
}


