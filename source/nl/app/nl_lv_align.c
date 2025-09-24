#include "nl_lv_align.h"
#include "nl_stats.h"

#define NL_DBG_TAG    "nl.lv_align"
#define NL_DBG_LVL    NL_DBG_ERROR
#include "nl_log.h"




/**
 * @brief Perform initial alignment using gravity with gyroscope bias estimation
 * @param ins Pointer to INS structure
 * @param mgr Pointer to IMU manager structure
 * @return 0: stable alignment, 1: fallback alignment, -1: failed
 */
int nl_lv_align(ins_t *ins, nl_imu_mgr_t *mgr)
{
    enum {
        WINDOW_SIZE = 200,
        MAX_ITERATIONS = 4,           // Increase iteration count
    };
    
    // Tighten thresholds
    const nl_t ACC_VAR_THRESHOLD = 0.2;
    const nl_t GYR_VAR_THRESHOLD = 0.0005;
    const nl_t GRAVITY_TOLERANCE = 0.1;
    const nl_t GYR_BIAS_TOLERANCE = (0.1*D2R);
    
    nl_stats_basic_t acc_stats[3];
    nl_stats_basic_t gyr_stats[3];
    nl_stats_basic_t acc_mag_stats;
    
    bool acc_aligned = false;
    bool gyr_bias_set = false;
    
    // Add stability counters
    int acc_stable_count = 0;
    int gyr_stable_count = 0;
    nl_t last_gyr_bias[3] = {0};
    
    NL_LOG_D("Starting alignment: tighter thresholds, %d stable windows required", MAX_ITERATIONS);
    
    for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
        // Reset statistics
        for (int i = 0; i < 3; i++) {
            nl_stats_basic_reset(&acc_stats[i]);
            nl_stats_basic_reset(&gyr_stats[i]);
        }
        nl_stats_basic_reset(&acc_mag_stats);
        
        // Collect samples
        for (int sample = 0; sample < WINDOW_SIZE; sample++) {
            nl_imu_read_array(mgr);
            nl_mdelay(1);
            
            for (int i = 0; i < 3; i++) {
                nl_stats_basic_update(&acc_stats[i], mgr->buffer.cal.acc[i]);
                nl_stats_basic_update(&gyr_stats[i], mgr->buffer.cal.gyr[i]);
            }
            
            nl_t acc_mag = v3norm(mgr->buffer.cal.acc);
            nl_stats_basic_update(&acc_mag_stats, acc_mag);
        }
        
        // Check accelerometer (keep original logic but require consecutive stability)
        if (!acc_aligned) {
            nl_t acc_mag_mean = nl_stats_basic_get_mean(&acc_mag_stats);
            nl_t acc_mag_var = nl_stats_basic_get_variance(&acc_mag_stats);
            
            bool gravity_ok = fabs(acc_mag_mean - GRAVITY) < (GRAVITY_TOLERANCE * GRAVITY);
            bool acc_stable = acc_mag_var < ACC_VAR_THRESHOLD;
            
            if (gravity_ok && acc_stable) {
                acc_stable_count++;
                NL_LOG_D("Window %d: ACC stable (%d/%d)", iteration, acc_stable_count, MAX_ITERATIONS);
                
                if (acc_stable_count >= MAX_ITERATIONS) {
                    // Perform alignment
                    nl_t acc_mean[3];
                    for (int i = 0; i < 3; i++) {
                        acc_mean[i] = nl_stats_basic_get_mean(&acc_stats[i]);
                    }
                    
                    nl_t mag = v3norm(acc_mean);
                    if (mag > 0.1) {
                        for (int i = 0; i < 3; i++) {
                            acc_mean[i] *= GRAVITY / mag;
                        }
                    }
                    
                    ins_align(ins, acc_mean, ins->att.yaw);
                    acc_aligned = true;
                    NL_LOG_I("ACC aligned after %d stable windows", acc_stable_count);
                }
            } else {
                acc_stable_count = 0; // Reset counter
                NL_LOG_D("Window %d: ACC unstable - mag=%.3f(%.3f), var=%.6f(%.6f)", 
                        iteration, acc_mag_mean, GRAVITY, acc_mag_var, ACC_VAR_THRESHOLD);
            }
        }
        
        // Check gyroscope (add consistency and continuity checks)
        if (!gyr_bias_set) {
            bool gyr_stable = true;
            nl_t current_bias[3];
            
            for (int i = 0; i < 3; i++) {
                nl_t var = nl_stats_basic_get_variance(&gyr_stats[i]);
                current_bias[i] = nl_stats_basic_get_mean(&gyr_stats[i]);
                
                // Check variance
                if (var > GYR_VAR_THRESHOLD) {
                    gyr_stable = false;
                    NL_LOG_D("Window %d: GYR axis %d var too high: %.8f", iteration, i, var);
                    break;
                }
                
                // Check bias magnitude with different thresholds for different axes
                nl_t bias_threshold = (i == 2) ? GYR_BIAS_TOLERANCE : (2.0 * GYR_BIAS_TOLERANCE);
                
                if (fabs(current_bias[i]) > bias_threshold) {
                    gyr_stable = false;
                    NL_LOG_D("Window %d: GYR axis %d bias too large: %.6f deg/s (threshold: %.6f deg/s)", 
                            iteration, i, current_bias[i] * R2D, bias_threshold * R2D);
                    break;
                }
                
                // Check consistency with previous window (starting from 2nd window)
                if (gyr_stable_count > 0) {
                    nl_t bias_change = fabs(current_bias[i] - last_gyr_bias[i]);
                    if (bias_change > 0.05 * D2R) { // 0.05 deg/s consistency requirement
                        gyr_stable = false;
                        NL_LOG_D("Window %d: GYR axis %d inconsistent: change %.6f deg/s", 
                                iteration, i, bias_change * R2D);
                        break;
                    }
                }
            }
            
            if (gyr_stable) {
                gyr_stable_count++;
                // Save current bias for next comparison
                for (int i = 0; i < 3; i++) {
                    last_gyr_bias[i] = current_bias[i];
                }
                
                NL_LOG_D("Window %d: GYR stable (%d/%d)", iteration, gyr_stable_count, MAX_ITERATIONS);
                
                if (gyr_stable_count >= MAX_ITERATIONS) {
                    // Set bias
                    v3copy(ins->wb, current_bias);
                    gyr_bias_set = true;
                    
                    NL_LOG_I("GYR bias set after %d stable windows: [%.4f, %.4f, %.4f] deg/s", gyr_stable_count, ins->wb[0]*R2D, ins->wb[1]*R2D, ins->wb[2]*R2D);
                }
            } else {
                gyr_stable_count = 0; // Reset counter
            }
        }
        
        // Exit if both completed
        if (acc_aligned && gyr_bias_set) {
            NL_LOG_I("Alignment completed successfully after %d iterations", iteration + 1);
            return 0;
        }
    }
    
    // Timeout handling (keep original logic)
    if (acc_aligned && !gyr_bias_set) {
        NL_LOG_W("ACC aligned but GYR bias failed - using zero bias");
        return 1;
    }
    
    if (!acc_aligned && gyr_bias_set) {
        NL_LOG_W("GYR bias set but ACC alignment failed - using fallback");
        return 1;
    }
    
    nl_t acc_mean[3];
    for (int i = 0; i < 3; i++) {
        acc_mean[i] = nl_stats_basic_get_mean(&acc_stats[i]);
    }
    
    nl_t mag = v3norm(acc_mean);
    if (mag > 0.1) {
        for (int i = 0; i < 3; i++) {
            acc_mean[i] *= GRAVITY / mag;
        }
        ins_align(ins, acc_mean, ins->att.yaw);
        NL_LOG_W("Emergency fallback leveling done with mag=%.3f", mag);
    }
    
    return -1;
}


/**
 * @brief Set INS yaw angle based on magnetic field measurement and return confidence
 * @param ins Pointer to INS structure
 * @param ref_flux Expected local magnetic field strength
 * @param ref_incli Expected local magnetic inclination angle (rad)
 * @param mag Magnetic field vector in body frame
 * @param heading Pointer to store calculated heading
 * @param confidence Pointer to store confidence value (0-100, lower is better)
 * @return 0: Success, <0: Error
 */
int nl_set_yaw_from_mag(ins_t *ins, nl_t *mag, nl_t *heading)
{
    nl_t flux = v3norm(mag);
    
    // Check if magnetic field strength is valid
    if (flux < 0.01 )
    {
        return -1;  // Invalid magnetic field strength
    }
    
    
    // Calculate current magnetic inclination
    nl_t qtmp[4];
    nl_t tmp[3];
    att_t att_tmp = ins->att;
    
    // Make a new horizontal attitude with yaw=0
    att_tmp.yaw = 0;
    att2q(&att_tmp, qtmp);
    
    // Transform magnetic vector from body to navigation frame
    qmulv(qtmp, mag, tmp);
    
    
    // Calculate yaw based on coordinate system (unchanged)
    nl_t initial_heading = 0;
    if (ins->coord == NL_COORD_ENU)
    {
        initial_heading = atan2(tmp[0], tmp[1]);
    }
    else if (ins->coord == NL_COORD_NED || ins->coord == NL_COORD_NWU)
    {
        initial_heading = atan2(-tmp[1], tmp[0]);
    }
    
    *heading = initial_heading;
    
    NL_LOG_D("Magnetic heading: %.2f° (flux conf: %.1f%%, incli conf: %.1f%%, total: %d%%)", 
             initial_heading * R2D, flux_relative_diff * 100.0f, 
             incli_relative_diff * 100.0f, conf_value);
    
    return 0;
}

