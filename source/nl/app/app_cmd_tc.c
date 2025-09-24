#include "app_def.h"

#define NL_DBG_TAG    "nl.tc"
#define NL_DBG_LVL    NL_DBG_INFO
#include "nl_log.h"

// Temperature calibration configuration constants
#define TCAL_DEFAULT_BIN_CNT        (50)    // High-resolution collection bins
#define TCAL_DEFAULT_TEMP_RANGE     (80.0f) // Minimum temperature range required for calibration
#define TCAL_TEMP_MIN               (-30.0f) // Minimum temperature boundary
#define TCAL_TEMP_MAX               (70.0f)  // Maximum temperature boundary
#define SAMPLE_DT_MS                (5)
#define SAMPLE_FRQ                  (1000 / SAMPLE_DT_MS)

// Constants for stability detection
#define TEMP_STABILITY_THRESHOLD    (1.5f)              //  variation threshold
#define TEMP_STABILITY_REQUIRED     (SAMPLE_FRQ * 10)   // 1 minute stable required before collection

/**
 * @brief Temperature calibration bin structure
 * Stores accumulated sensor data for a specific temperature range
 */
typedef struct {
    nl_t temp_sum;          // Temperature sum
    nl_t acc_sum[3];        // Accelerometer sum [x, y, z]
    nl_t gyr_sum[3];        // Gyroscope sum [x, y, z]
    nl_t gyr_z_rot_sum;     // Z-axis gyroscope sum during rotation
    uint32_t static_cnt;    // Number of static samples
    uint32_t gyr_z_rot_cnt;  // Number of rotation samples
} tcal_bin_t;

/**
 * @brief Temperature calibration data structure
 * Manages the collection bins and temperature range information
 */
typedef struct {
    tcal_bin_t *bins;           // Dynamic array of temperature bins
    nl_t step;                  // Temperature step size between bins
    nl_t actual_temp_min;       // Actual minimum temperature observed
    nl_t actual_temp_max;       // Actual maximum temperature observed
    uint32_t bin_cnt;           // Number of collection bins
} tcal_data_t;

/**
 * @brief Lightweight temperature stability detection for STM32
 */
typedef struct {
    nl_t temp_min;              // Minimum temperature in window
    nl_t temp_max;              // Maximum temperature in window  
    nl_t temp_sum;              // Temperature sum for average
    uint32_t sample_count;      // Sample count in current window
    uint32_t stable_duration;   // Consecutive stable sample count
    bool is_stable;             // Temperature stability flag
} temp_stability_t;


/**
 * @brief Temperature compensation parameters structure
 * Stores polynomial coefficients: sensor_output = bias + tc1*T + tc2*T^2
 */
typedef struct {
    nl_t gbtc1[3];    // First order temperature coefficients for accelerometer
    nl_t gbtc2[3];    // Second order temperature coefficients for accelerometer
    nl_t wbtc1[3];    // First order temperature coefficients for gyroscope
    nl_t wbtc2[3];    // Second order temperature coefficients for gyroscope
    nl_t wstc1[3];   // Z-axis scale factor first order temperature coefficient, only[2] available
    nl_t wstc2[3];   // Z-axis scale factor second order temperature coefficient
} temp_comp_params_t;

/**
 * @brief Motion state detection
 */
typedef enum {
    MOTION_STATIC,      // Static state
    MOTION_ROTATING,    // Rotating state
    MOTION_TRANSITION   // Transition state
} motion_state_t;


// Global temperature compensation parameters
static temp_comp_params_t g_tc_params = {0};
static temp_comp_params_t g_tc_params_low = {0};

// Global temperature calibration data
static tcal_data_t g_tcal;

static temp_stability_t g_temp_stability = {0};

static motion_state_t detect_motion_state(nl_t gyr_z)
{
    static nl_t delayed_buffer[10] = {0};
    static uint32_t delay_idx = 0;
    static uint32_t stable_count = 0;
    
    // Get delayed sample
    nl_t delayed_gyr = delayed_buffer[delay_idx];
    delayed_buffer[delay_idx] = gyr_z;
    delay_idx = (delay_idx + 1) % 10;
    
    // Use delayed sample for state detection
    nl_t abs_delayed = fabsf(delayed_gyr);
    nl_t abs_current = fabsf(gyr_z);
    
    motion_state_t current_state;
    
    // Static: delayed sample is static AND current sample confirms it
    if (abs_delayed < 1.5f * D2R && abs_current < 1.5f * D2R && fabs(delayed_gyr - gyr_z) < 0.7*D2R) {
        current_state = MOTION_STATIC;
    } 
    // Rotating: standard detection
    else if (abs_delayed > 48.0f * D2R && abs_delayed < 52.0f * D2R  && fabs(abs_delayed - abs_current) < 0.5*D2R) {
        current_state = MOTION_ROTATING;
    } 
    else {
        current_state = MOTION_TRANSITION;
    }
    
    // Stability check
    static motion_state_t last_state = MOTION_TRANSITION;
    if (current_state == last_state) {
        stable_count++;
    } else {
        stable_count = 0;
        last_state = current_state;
    }
    
    return (stable_count > SAMPLE_FRQ * 2) ? current_state : MOTION_TRANSITION;
}

/**
 * @brief Detect temperature stability using min-max tracking (RAM efficient)
 * @param current_temp Current temperature reading
 * @return true if temperature is stable for bias collection
 */
static bool detect_temp_stability(nl_t current_temp)
{
    temp_stability_t *ts = &g_temp_stability;
    
    // Initialize on first sample
    if (ts->sample_count == 0) {
        ts->temp_min = current_temp;
        ts->temp_max = current_temp;
        ts->sample_count = 1;
        ts->stable_duration = 0;
        return false;
    }
    
    // Update min/max tracking
    if (current_temp < ts->temp_min) ts->temp_min = current_temp;
    if (current_temp > ts->temp_max) ts->temp_max = current_temp;
    ts->sample_count++;
    
    // Check if current range is stable
    nl_t temp_range = ts->temp_max - ts->temp_min;
    bool is_range_stable = (temp_range <= TEMP_STABILITY_THRESHOLD);
    
    if (is_range_stable) {
        ts->stable_duration++;
        ts->is_stable = (ts->stable_duration >= TEMP_STABILITY_REQUIRED);
    } else {
        // Reset when temperature becomes unstable
        ts->temp_min = current_temp;
        ts->temp_max = current_temp;
        ts->sample_count = 1;
        ts->stable_duration = 0;
        ts->is_stable = false;
    }
    
    return ts->is_stable;
}


/**
 * @brief Add sample to calibration bin with motion state and temperature stability awareness
 * @param bin Pointer to bin structure
 * @param temp Temperature value
 * @param acc Accelerometer data
 * @param gyr Gyroscope data
 * @param state Motion state
 * @param temp_stable Temperature stability flag
 */
static void tcal_add_sample(tcal_bin_t *bin, nl_t temp, nl_t acc[3], nl_t gyr[3], 
                           motion_state_t state, bool temp_stable)
{
    if (state == MOTION_STATIC && temp_stable) {
        // Static samples with stable temperature - for bias calibration
        bin->temp_sum += temp;
        v3add2(bin->acc_sum, acc);
        v3add2(bin->gyr_sum, gyr);
        bin->static_cnt++;
    } else if (state == MOTION_ROTATING) {
        // Rotation samples - for scale factor calibration (no temp stability required)
        bin->temp_sum += temp;
        bin->gyr_z_rot_sum += gyr[2];
        bin->gyr_z_rot_cnt++;
    }
}


/**
 * @brief Fit Z-axis scale factor temperature coefficients
 * @param bins Array of calibration bins
 * @param bin_cnt Number of bins
 * @return 0 on success, negative on error
 */
static int fit_scale_factor_poly(const tcal_bin_t *bins, uint32_t bin_cnt)
{
     int result = 1;
    
    // Count valid rotation bins
    uint32_t scale_valid_cnt = 0;
    for (uint32_t i = 0; i < bin_cnt; i++) {
        if (bins[i].gyr_z_rot_cnt > 0) scale_valid_cnt++;
    }
    
    if (scale_valid_cnt < 3) {
        return -1;
    }
    
    // Create matrices for scale factor fitting - 3 parameters like bias fitting
    m_t *A_scale = mcreate(scale_valid_cnt, 3);  // 3 columns: constant + linear + quadratic
    m_t *Y_scale = mcreate(scale_valid_cnt, 1);  // 1 column: scale factor error
    m_t *X_scale = mcreate(3, 1);                // 3 rows: constant + linear + quadratic coefficients
    m_t *Q_scale = mcreate(3, 3);                // 3x3 covariance matrix
    
    if (!A_scale || !Y_scale || !X_scale || !Q_scale) {
        goto cleanup_scale;
    }
    
    // Fill scale factor matrices
    uint32_t row = 0;
    const nl_t expected_rate = 50.0f * D2R;  // Expected 50 dps rotation rate
    
    for (uint32_t i = 0; i < bin_cnt; i++) {
        if (bins[i].gyr_z_rot_cnt > 0) {
            nl_t inv_cnt = 1.0f / bins[i].gyr_z_rot_cnt;
            nl_t temp = bins[i].temp_sum * inv_cnt / 100;  // Scale temperature for numerical stability
            nl_t avg_gyr_z = bins[i].gyr_z_rot_sum * inv_cnt;
            
            // Design matrix A: [1, T, T^2] - same as bias fitting
            MELEMENT(A_scale, row, 0) = 1.0f;           // Constant term
            MELEMENT(A_scale, row, 1) = temp;           // Linear term
            MELEMENT(A_scale, row, 2) = temp * temp;    // Quadratic term
            
            // Scale factor error = (measured - expected) / expected
            nl_t scale_error = (fabs(avg_gyr_z)) / expected_rate;
            MELEMENT(Y_scale, row, 0) = scale_error;
            
            row++;
        }
    }
    
    result = nl_lsq2(A_scale, Y_scale, X_scale, Q_scale);
    
    if (result == 0) {
        // Extract scale factor coefficients - store only TC1 and TC2, ignore constant term
        nl_t scale_bias = MELEMENT(X_scale, 0, 0);      // Constant term (not stored)
        g_tc_params.wstc1[2] = MELEMENT(X_scale, 1, 0);  // Linear coefficient
        g_tc_params.wstc2[2] = MELEMENT(X_scale, 2, 0);  // Quadratic coefficient
        
        NL_LOG_I("Z-axis scale factor coefficients fitted:");
        NL_LOG_I("Bias (not stored): %.6f", scale_bias);
        NL_LOG_I("TC1 (linear): %.6f", g_tc_params.wstc1[2]);
        NL_LOG_I("TC2 (quadratic): %.6f", g_tc_params.wstc2[2]);
    }
    
cleanup_scale:
    if (A_scale) nl_free(A_scale);
    if (Y_scale) nl_free(Y_scale);
    if (X_scale) nl_free(X_scale);
    if (Q_scale) nl_free(Q_scale);
    
    return result;
}


///**
// * @brief Fit polynomial coefficients for all axes using bin data
// * @param bins Array of calibration bins
// * @param bin_cnt Number of bins
// * @return 0 on success, negative on error
// */
//static int fit_all_axes_poly(const tcal_bin_t *bins, uint32_t bin_cnt)
//{
//    int result = 1;
//    
//    // Count valid bins for bias fitting
//    uint32_t valid_cnt = 0;
//    for (uint32_t i = 0; i < bin_cnt; i++) {
//        if (bins[i].static_cnt > 0) valid_cnt++;
//    }
//    
//    if (valid_cnt < 3) {
//        NL_LOG_E("Insufficient static data points (%u) for bias fitting", valid_cnt);
//        return -1;
//    }
//    
//    // 1. Fit bias coefficients (existing code)
//    m_t *A = mcreate(valid_cnt, 3);  // 3 columns: constant + linear + quadratic
//    m_t *Y = mcreate(valid_cnt, 6);  // 6 sensor axes
//    m_t *X = mcreate(3, 6);          // 3 rows: constant + linear + quadratic
//    m_t *Q = mcreate(3, 3);          // 3x3 covariance matrix
//    
//    if (!A || !Y || !X || !Q) {
//        NL_LOG_E("Matrix allocation failed for batch polynomial fitting");
//        goto cleanup;
//    }
//    
//    // Fill matrices directly from bins
//    uint32_t row = 0;
//    for (uint32_t i = 0; i < bin_cnt; i++) {
//        if (bins[i].static_cnt >= 1) {
//            // Calculate averages on-the-fly
//            nl_t inv_cnt = 1.0f / bins[i].static_cnt;
//            
//            /*  Scale temperature for numerical stability in polynomial fitting
//            Typical range: -20C to 70C becomes -0.2 to 0.7
//            This prevents large T^2 values that could cause numerical issues
//            */
//            nl_t temp = bins[i].temp_sum * inv_cnt / 100;
//            
//            // Design matrix A: [1, T, T^2]
//            MELEMENT(A, row, 0) = 1.0f;           // Constant term
//            MELEMENT(A, row, 1) = temp;           // Linear term
//            MELEMENT(A, row, 2) = temp * temp;    // Quadratic term
//            
//            // Measurement matrix Y: [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z]
//            for (int j = 0; j < 3; j++) {
//                MELEMENT(Y, row, j) = bins[i].acc_sum[j] * inv_cnt;      // Accelerometer
//                MELEMENT(Y, row, j+3) = bins[i].gyr_sum[j] * inv_cnt;    // Gyroscope
//            }
//            row++;
//        }
//    }
//    
//    result = nl_lsq2(A, Y, X, Q);
//    
//    if (result == 0) {
//        // Extract coefficients: bias + tc1*T + tc2*T^2
//        for (int i = 0; i < 3; i++) {
//         //   g_tc_params.acc_bias[i] = MELEMENT(X, 0, i);    // Constant term
//            g_tc_params.gbtc1[i] = MELEMENT(X, 1, i);     // Linear coefficient
//            g_tc_params.gbtc2[i] = MELEMENT(X, 2, i);     // Quadratic coefficient
//        }
//        
//        for (int i = 0; i < 3; i++) {
//         //   g_tc_params.gyr_bias[i] = MELEMENT(X, 0, i+3);  // Constant term
//            g_tc_params.wbtc1[i] = MELEMENT(X, 1, i+3);   // Linear coefficient
//            g_tc_params.wbtc2[i] = MELEMENT(X, 2, i+3);   // Quadratic coefficient
//        }
//        
//        NL_LOG_I("Bias coefficients fitted successfully");
//        
//        // 2. Fit Z-axis scale factor coefficients
//        if (fit_scale_factor_poly(bins, bin_cnt) != 0) {
//            NL_LOG_W("Scale factor fitting failed, but bias fitting succeeded");
//            // Don't fail the entire process if only scale factor fitting fails
//        }
//    }
//    
//cleanup:
//    if (A) nl_free(A);
//    if (Y) nl_free(Y);
//    if (X) nl_free(X);
//    if (Q) nl_free(Q);
//    
//    return result;
//}

/**
 * @brief Fit polynomial coefficients for all axes using bin data
 * @param bins Array of calibration bins
 * @param bin_cnt Number of bins
 * @return 0 on success, negative on error
 */
static int fit_all_axes_poly_constrained(const tcal_bin_t *bins, uint32_t bin_cnt, temp_comp_params_t *tc_params, nl_t temp_fref, int8_t flag)
{
    int result = 1;
    
    uint32_t valid_cnt = 0;           // Counter initialized to 0
    nl_t temp_diff;                   // Temperature difference variable
    nl_t min_abs_temp_diff = 80; // Minimum absolute temperature difference
    uint32_t min_temp_diff_index = 0; // Corresponding index
    nl_t abs_temp_diff;
    for (uint32_t i = 0; i < bin_cnt; i++) {
        if (bins[i].static_cnt > 1000) {
            // Calculate temperature difference from reference
            temp_diff = bins[i].temp_sum / bins[i].static_cnt - temp_fref;
            // Update minimum absolute temperature difference and its index
            abs_temp_diff = fabs(temp_diff);
            if (abs_temp_diff < min_abs_temp_diff) {
                min_abs_temp_diff = abs_temp_diff;
                min_temp_diff_index = i;  // Record corresponding index
            }
            // Valid count above/below reference temperature
            if (temp_diff * flag >= 0) valid_cnt++;
        }
    }
    NL_LOG_I("min_abs_temp_diff:%f", min_abs_temp_diff);
    nl_t temp_step = (TCAL_TEMP_MAX - TCAL_TEMP_MIN) / TCAL_DEFAULT_BIN_CNT;
    nl_t actual_temp_range = valid_cnt * temp_step;
    if (actual_temp_range < 30) {
        NL_LOG_E("Insufficient static data points (%u) for bias fitting", valid_cnt);
        return -1;
    }
    
    // 1. Fit bias coefficients with Lagrange multiplier constraint
    m_t *A = mcreate(valid_cnt, 3);          // Design matrix: [1, T, T2]
    m_t *At = mcreate(3, valid_cnt);         // At
    m_t *Y = mcreate(valid_cnt, 6);          // Measurement matrix
    m_t *AtA = mcreate(3, 3);                // A^T * A
    m_t *Atb = mcreate(3, 6);                // A^T * b
    m_t *KKT_matrix = mcreate(4, 4);         // KKT system: [2*AtA, C'; C, 0]
    m_t *rhs = mcreate(4, 6);                // Right hand side: [2*Atb; d]
    m_t *solution = mcreate(4, 6);           // Solution: [coeffs; lambda]
    m_t *X = mcreate(3, 6);                  // Final coefficients (without lambda)
    m_t *Q = mcreate(4, 4);                  
    if (!A || !Y || !AtA || !Atb || !KKT_matrix || !rhs || !solution || !X || !Q) {
      NL_LOG_E("Matrix allocation failed for batch polynomial fitting");
      goto cleanup;
    }

    // Fill design matrix A and measurement matrix Y
    uint32_t row = 0;
    nl_t inv_cnt_min = 1.0f / bins[min_temp_diff_index].static_cnt;
    for (uint32_t i = 0; i < bin_cnt; i++) {
        
        if (bins[i].static_cnt > 1000 && (bins[i].temp_sum/bins[i].static_cnt - temp_fref) * flag >= 0) {
          // Calculate averages on-the-fly
          nl_t inv_cnt = 1.0f / bins[i].static_cnt;
          
          /* Scale temperature for numerical stability in polynomial fitting
             Typical range: -20C to 70C becomes -0.2 to 0.7
             This prevents large T^2 values that could cause numerical issues */
          nl_t temp = (bins[i].temp_sum * inv_cnt - temp_fref) / 100;
          
          // Design matrix A: [1, T, T2] - consistent with MATLAB code
          MELEMENT(A, row, 0) = 1.0f;           // Constant term
          MELEMENT(A, row, 1) = temp;           // Linear term (T)
          MELEMENT(A, row, 2) = temp * temp;    // Quadratic term (T*T)
          
          // Measurement matrix Y: [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z]
          for (int j = 0; j < 3; j++) {
              MELEMENT(Y, row, j) = bins[i].acc_sum[j] * inv_cnt - bins[min_temp_diff_index].acc_sum[j] * inv_cnt_min;      // Accelerometer
              MELEMENT(Y, row, j+3) = bins[i].gyr_sum[j] * inv_cnt - bins[min_temp_diff_index].gyr_sum[j] * inv_cnt_min;    // Gyroscope
          }
          row++;
        }
    }
    mtrans(A, At);                // 
    mmul(At, A, AtA);                 // At * A
    // Compute A^T * Y (for all 6 output columns)  
    mmul(At, Y, Atb);                 // At * Y

    // Construct KKT matrix: [2*AtA, C'; C, 0]
    // Upper left block: 2*AtA (3x3)
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          MELEMENT(KKT_matrix, i, j) = 2.0f * MELEMENT(AtA, i, j);
      }
    }

    // Upper right block: C^T = [1; 0; 0] (constraint on constant term = 0)
    MELEMENT(KKT_matrix, 0, 3) = 1.0f;  // Constant term constraint
    MELEMENT(KKT_matrix, 1, 3) = 0.0f;  // Linear term unconstrained
    MELEMENT(KKT_matrix, 2, 3) = 0.0f;  // Quadratic term unconstrained

    // Lower left block: C = [1, 0, 0] (constraint matrix)
    MELEMENT(KKT_matrix, 3, 0) = 1.0f;
    MELEMENT(KKT_matrix, 3, 1) = 0.0f;
    MELEMENT(KKT_matrix, 3, 2) = 0.0f;

    // Lower right block: 0
    MELEMENT(KKT_matrix, 3, 3) = 0.0f;

    // Construct right hand side: [2*Atb; d]
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 6; j++) {
          MELEMENT(rhs, i, j) = 2.0f * MELEMENT(Atb, i, j);
        }
    }

    // Constraint values: d = 0 (for all output columns)
    for (int j = 0; j < 6; j++) {
        MELEMENT(rhs, 3, j) = 0.0f;
    }
    // Solve KKT system: KKT_matrix * solution = rhs
    result = nl_lsq2(KKT_matrix, rhs, solution, Q);
    // Debug output - print KKT matrix
    if (1) {
//        NL_LOG_I("=== KKT Matrix (%dx%d) ===", KKT_matrix->r, KKT_matrix->c);
//        for (int i = 0; i < 4; i++) {
//          NL_LOG_I("Row %d: [%.6f, %.6f, %.6f, %.6f]", i,
//                   MELEMENT(KKT_matrix, i, 0),
//                   MELEMENT(KKT_matrix, i, 1),
//                   MELEMENT(KKT_matrix, i, 2),
//                   MELEMENT(KKT_matrix, i, 3));
//        }
        // solution (4x6)
        NL_LOG_I("=== Solution Matrix (%dx%d) ===", solution->r, solution->c);
        for (int i = 0; i < 4; i++) {
            NL_LOG_I("Row %d: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]", i,
                     MELEMENT(solution, i, 0),
                     MELEMENT(solution, i, 1),
                     MELEMENT(solution, i, 2),
                     MELEMENT(solution, i, 3),
                     MELEMENT(solution, i, 4),
                     MELEMENT(solution, i, 5));
        }

//        NL_LOG_I("=== RHS Matrix (%dx%d) ===", rhs->r, rhs->c);
//        for (int i = 0; i < 4; i++) {
//            NL_LOG_I("Row %d: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]", i,
//                     MELEMENT(rhs, i, 0),
//                     MELEMENT(rhs, i, 1),
//                     MELEMENT(rhs, i, 2),
//                     MELEMENT(rhs, i, 3));
//        }
        NL_LOG_I("=== Y Matrix (%dx%d) ===", Y->r, Y->c);
        for (int i = 0; i < Y->r; i++) {
            NL_LOG_I("Row %d: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]", i,
                     MELEMENT(Y, i, 0),
                     MELEMENT(Y, i, 1),
                     MELEMENT(Y, i, 2),
                     MELEMENT(Y, i, 3),
                     MELEMENT(Y, i, 4),
                     MELEMENT(Y, i, 5));
        }
    }
    if (result == 0) {
        // Extract coefficients (first 3 rows, ignore 4th row which is Lagrange multiplier)
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 6; j++) {
              MELEMENT(X, i, j) = MELEMENT(solution, i, j);
          }
        }
        // Extract coefficients: bias + tc1*T + tc2*T^2
        // Note: X matrix order is now [1, T, T2]
        for (int i = 0; i < 3; i++) {
          // tc_params->acc_bias[i] = MELEMENT(X, 0, i);    // Constant term (row 1)
          tc_params->gbtc1[i] = MELEMENT(X, 1, i);         // Linear term (row 2)
          tc_params->gbtc2[i] = MELEMENT(X, 2, i);         // Quadratic term (row 3)
        }

        for (int i = 0; i < 3; i++) {
          // tc_params->gyr_bias[i] = MELEMENT(X, 0, i+3);  // Constant term (row 1)
          tc_params->wbtc1[i] = MELEMENT(X, 1, i+3);       // Linear term (row 2)
          tc_params->wbtc2[i] = MELEMENT(X, 2, i+3);       // Quadratic term (row 3)
        }

        NL_LOG_I("Bias coefficients fitted successfully with Lagrange multiplier constraint");

        // 2. Fit Z-axis scale factor coefficients
        if (fit_scale_factor_poly(bins, bin_cnt) != 0) {
          NL_LOG_W("Scale factor fitting failed, but bias fitting succeeded");
          // Don't fail the entire process if only scale factor fitting fails
        }
    } else {
        NL_LOG_E("KKT system solve failed with result: %d", result);
    }

    cleanup:
    if (A) nl_free(A);
    if (Y) nl_free(Y);
    if (AtA) nl_free(AtA);
    if (Atb) nl_free(Atb);
    if (KKT_matrix) nl_free(KKT_matrix);
    if (rhs) nl_free(rhs);
    if (solution) nl_free(solution);
    if (X) nl_free(X);
    if (Q) nl_free(Q);
    if (At) nl_free(At);
    return result;
}
/*
 * Temperature Compensation Parameters
 * 
 * IMU sensors (accelerometer/gyroscope) are affected by temperature variations.
 * Temperature compensation uses polynomial models to correct sensor bias and scale:
 * 
 * Corrected_Value = Raw_Value - Bias_Compensation + Scale_Compensation
 * 
 * Where:
 * Bias_Compensation = TC1 * (T - T_ref) + TC2 * (T - T_ref)^2
 * Scale_Compensation = Raw_Value * [TC1 * T + TC2 * T^2]
 * 
 * Parameters:
 * TC1: Linear temperature coefficient
 * TC2: Quadratic temperature coefficient  
 * T: Current temperature
 * T_ref: Reference calibration temperature
 * 
 * Coefficient Storage:
 * gbtc1/gbtc2: Accelerometer Bias Temperature Coefficients (linear/quadratic)
 * wbtc1/wbtc2: Gyroscope Bias Temperature Coefficients (linear/quadratic)
 * wstc1/wstc2: Gyroscope Scale Temperature Coefficients (linear/quadratic)
 * 
 * Each coefficient array contains [X, Y, Z] axis values
 * 
 * Constraint Applied:
 * Constant term (bias at reference temperature) is constrained to zero
 * using Lagrange multiplier method in KKT system solution
 */
static void tcal_save_params(bool low_flag, bool high_flag)
{
    rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
    
    // Save high temperature calibration parameters
    if(high_flag)
    {
        rt_param_set_float_array("GB_TC1", g_tc_params.gbtc1, 3, app);
        rt_param_set_float_array("GB_TC2", g_tc_params.gbtc2, 3, app);
        rt_param_set_float_array("WB_TC1", g_tc_params.wbtc1, 3, app);
        rt_param_set_float_array("WB_TC2", g_tc_params.wbtc2, 3, app);
    }
    
    // Save low temperature calibration parameters
    if(low_flag)
    {
        rt_param_set_float_array("GB_TC1_LOW", g_tc_params_low.gbtc1, 3, app);
        rt_param_set_float_array("GB_TC2_LOW", g_tc_params_low.gbtc2, 3, app);
        rt_param_set_float_array("WB_TC1_LOW", g_tc_params_low.wbtc1, 3, app);
        rt_param_set_float_array("WB_TC2_LOW", g_tc_params_low.wbtc2, 3, app);
    }
    
    rt_param_set_float_array("WS_TC1", g_tc_params.wstc1, 3, app);
    rt_param_set_float_array("WS_TC2", g_tc_params.wstc2, 3, app);
    
    rt_setting_save(app);
    
    // Load all calibration parameters
    nl_imu_load_temp_calib(&eskfsvr.imu_mgr, 
                          g_tc_params.gbtc1, g_tc_params.gbtc2,        // High temp GB params
                          g_tc_params.wbtc1, g_tc_params.wbtc2,        // High temp WB params
                          g_tc_params.wstc1, g_tc_params.wstc2,        // WS params
                          g_tc_params_low.gbtc1, g_tc_params_low.gbtc2, // Low temp GB params
                          g_tc_params_low.wbtc1, g_tc_params_low.wbtc2, // Low temp WB params
                          rt_param_get_float("GB_TC_TEMP", app), 
                          rt_param_get_float("WB_TC_TEMP", app));
}

/**
 * @brief Initialize temperature calibration data structure
 * @param data Pointer to calibration data structure
 * @param bin_cnt Number of collection bins to allocate
 * @return RT_EOK on success, RT_ERROR on failure
 */
static int tcal_init_data(tcal_data_t *data, uint32_t bin_cnt)
{
    // Allocate memory for collection bins
    data->bins = (tcal_bin_t*)nl_malloc(bin_cnt * sizeof(tcal_bin_t));
    if (!data->bins) {
        NL_LOG_E("Failed to allocate %u collection bins", bin_cnt);
        return RT_ERROR;
    }
    
    // Initialize bins to zero
    rt_memset(data->bins, 0, bin_cnt * sizeof(tcal_bin_t));
    data->bin_cnt = bin_cnt;
    
    // Calculate temperature step size for bin distribution
    nl_t temp_range = TCAL_TEMP_MAX - TCAL_TEMP_MIN;
    data->step = temp_range / (bin_cnt - 1);
    
    // Initialize actual temperature range tracking
    data->actual_temp_min = TCAL_TEMP_MAX;  // Start with max value
    data->actual_temp_max = TCAL_TEMP_MIN;  // Start with min value
    
    return RT_EOK;
}

/**
 * @brief Find the appropriate collection bin for a given temperature
 * @param temp Temperature value
 * @param data Pointer to calibration data structure
 * @return Bin index on success, -1 if temperature is out of range
 */
static int tcal_find_bin(nl_t temp, tcal_data_t *data)
{
    // Check if temperature is within valid range
    if (temp < TCAL_TEMP_MIN || temp > TCAL_TEMP_MAX + 1) {
        return -1;
    }
    
    // Update actual temperature range tracking
    if (temp < data->actual_temp_min) data->actual_temp_min = temp;
    if (temp > data->actual_temp_max) data->actual_temp_max = temp;
    
    // Calculate bin index with rounding
    int bin_idx = (int)((temp - TCAL_TEMP_MIN) / data->step + 0.5f);
    
    // Ensure bin index is within valid range
    if (bin_idx >= (int)data->bin_cnt) bin_idx = data->bin_cnt - 1;
    if (bin_idx < 0) bin_idx = 0;
    
    return bin_idx;
}

/**
 * @brief Finalize temperature calibration process with coefficient calculation
 * @param data Pointer to calibration data structure
 * @param cli CLI handle for output
 * @return RT_EOK on success, RT_ERROR on failure
 */
static int tcal_finalize_data(tcal_data_t *data, rt_cli_t cli)
{
    // Print header for bin data
    NL_LOG_I("Format: BIN_IDX, TEMP_CENTER, STATIC_SAMPLES, ROT_SAMPLES, ACC_X, ACC_Y, ACC_Z, GYR_X, GYR_Y, GYR_Z, GYR_Z_ROT");
    
    // Print each bin's data
    for (uint32_t i = 0; i < data->bin_cnt; i++) {
        tcal_bin_t *bin = &data->bins[i];
        
        if (bin->static_cnt > 0 || bin->gyr_z_rot_cnt > 0) {
            nl_t avg_temp = bin->temp_sum / (bin->static_cnt + bin->gyr_z_rot_cnt);
            
            // Calculate static averages
            nl_t acc_avg[3] = {0}, gyr_avg[3] = {0};
            if (bin->static_cnt > 0) {
                nl_t inv_static = 1.0f / bin->static_cnt;
                for (int j = 0; j < 3; j++) {
                    acc_avg[j] = bin->acc_sum[j] * inv_static;
                    gyr_avg[j] = bin->gyr_sum[j] * inv_static;
                }
            }
            
            // Calculate rotation average
            nl_t gyr_z_rot_avg = 0;
            if (bin->gyr_z_rot_cnt > 0) {
                gyr_z_rot_avg = bin->gyr_z_rot_sum / bin->gyr_z_rot_cnt;
            }
            
            // Print bin data in CSV-like format for easy MATLAB import
            rt_kprintf("BIN_DATA: %2u, %6.2f, %4u, %4u, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f, %8.5f\r\n",
                         i, avg_temp, bin->static_cnt, bin->gyr_z_rot_cnt,
                         acc_avg[0], acc_avg[1], acc_avg[2],
                         gyr_avg[0]*R2D, gyr_avg[1]*R2D, gyr_avg[2]*R2D,
                         gyr_z_rot_avg * R2D);  // Convert to degrees/second for display
        }
    }
    // Fit temperature compensation coefficients
    int result = RT_EOK;
    bool low_temp_success = false;
    bool high_temp_success = false;
    
    // LOW temperature compensation
    if (fit_all_axes_poly_constrained(data->bins, data->bin_cnt, &g_tc_params_low, 20, -1) == 0) {
        low_temp_success = true;
        NL_LOG_I("LOW_Temperature compensation coefficients calculated;data->bin_cnt:%d",data->bin_cnt);
    } else {
        NL_LOG_E("LOW_Temperature FIT FAIL");
        result = RT_ERROR;
    }
    
    // HIGH temperature compensation    
    if (fit_all_axes_poly_constrained(data->bins, data->bin_cnt, &g_tc_params, 20, 1) == 0) {
        high_temp_success = true;
        NL_LOG_I("HIGH_Temperature compensation coefficients calculated");
    } else {
        NL_LOG_E("HIGH_Temperature FIT FAIL");
        result = RT_ERROR;
    }
    
    // Save parameters only once if at least one calibration succeeded
    if (low_temp_success || high_temp_success) {
        tcal_save_params(low_temp_success, high_temp_success);
        NL_LOG_I("Temperature compensation parameters saved");
    } else {
        NL_LOG_E("All temperature compensation calibrations failed - no parameters saved");
    }
    
    // Clean up
    if (data->bins) {
        nl_free(data->bins);
        data->bins = NULL;
    }
    
    return result;
}

/**
 * @brief Temperature calibration command implementation with configurable parameters
 * @param cli CLI handle
 * @param argc Argument count
 * @param argv Argument vector: FCAL_TC2 [max_time_s] [temp_min] [temp_max] [min_range]
 * @return RT_EOK on success, RT_ERROR on failure
 */
int FTC(rt_cli_t cli, int argc, char *argv[])
{
    nl_imu_mgr_t *imu_mgr = &eskfsvr.imu_mgr;
    nl_imu_data_t *cal = &imu_mgr->buffer.cal;
    
    eskfsvr.en_factory_mode = 1;
    
    if (argc >= 2 && !rt_strcmp("CLR", argv[1])) {
        rt_memset(&g_tc_params, 0, sizeof(g_tc_params));
        rt_memset(&g_tc_params_low, 0, sizeof(g_tc_params_low));
        tcal_save_params(1, 1);
        rt_kprintf("All temperature compensation coefficients cleared\r\n");
        return RT_EOK;
    }
        
    // Add RST command
    if (argc >= 2 && !rt_strcmp("RST", argv[1])) {
        rt_setting_t app = rt_setting_find(SETTING_APP_NAME);
        if (!app) {
            rt_kprintf("Failed to find application settings\r\n");
            return RT_ERROR;
        }
        
        // Temperature compensation parameters to restore
        const char *tc_params[] = {
            "GB_TC1", "GB_TC2", "GB_TC1_LOW", "GB_TC2_LOW",
            "WB_TC1", "WB_TC2", "WB_TC1_LOW", "WB_TC2_LOW", 
            "WS_TC1", "WS_TC2"
        };
        
        for (int i = 0; i < sizeof(tc_params)/sizeof(tc_params[0]); i++) {
            rt_setting_reset_param(app, tc_params[i]);
        }
        
            rt_setting_save(app);
            rt_kprintf("Restored temperature compensation parameters to factory defaults\r\n");
        return RT_EOK;
    }
    
    // Default calibration parameters (configurable)
    uint32_t max_time_s = 6000;                         // Maximum calibration time
    nl_t min_temp_range = TCAL_DEFAULT_TEMP_RANGE;      // Configurable minimum temperature range
    
    int bin_idx = -1;  // Current best bin assignment

    // Parse command line arguments
    if (argc >= 2) sscanf(argv[1], "%u", &max_time_s);
    if (argc >= 3) sscanf(argv[2], "%f", &min_temp_range);
    
    // Initialize calibration data structure
    rt_memset(&g_tcal, 0, sizeof(g_tcal));
    
    rt_memset(&g_temp_stability, 0, sizeof(g_temp_stability));
    
    if (tcal_init_data(&g_tcal, TCAL_DEFAULT_BIN_CNT) != RT_EOK) {
        return RT_ERROR;
    }
    
    // Disable existing temperature compensation during calibration
    nl_imu_set_tc(imu_mgr, 0);
    
    // Display calibration configuration
    NL_LOG_I("Temperature range: %.1f to %.1fC", TCAL_TEMP_MIN, TCAL_TEMP_MAX);
    NL_LOG_I("Collection bins: %u (step: %.2fC)", TCAL_DEFAULT_BIN_CNT, g_tcal.step);
    NL_LOG_I("Min temperature range: %.1fC", min_temp_range);
    NL_LOG_I("Max calibration time: %us", max_time_s);
    
    uint32_t start_time = rt_tick_get_millisecond() / 1000;
    uint32_t status_cnt = 0;
    
    while (1) {
        nl_imu_read_array(imu_mgr);
        uint32_t elapsed_s = rt_tick_get_millisecond() / 1000 - start_time;
        
        // Detect motion state
        motion_state_t motion_state = detect_motion_state(cal->gyr[2]);
        
        // Detect temperature stability (lightweight)
        bool temp_stable = detect_temp_stability(cal->temp);
        
        // Process samples
        bin_idx = tcal_find_bin(cal->temp, &g_tcal);
        if (bin_idx >= 0) {
            tcal_bin_t *bin = &g_tcal.bins[bin_idx];
            tcal_add_sample(bin, cal->temp, cal->acc, cal->gyr, motion_state, temp_stable);
        }
        
        if (++status_cnt >= SAMPLE_FRQ*10) {
            status_cnt = 0;
            nl_t current_range = g_tcal.actual_temp_max - g_tcal.actual_temp_min;
            
            if (bin_idx >= 0) {
                NL_LOG_I("T=%.1fC BIN=%d RANGE=%.1fC TIME=%us STAT:%d STABLE:%d GYR_Z=%.3f ACC_Z=%.3f", 
                         cal->temp, bin_idx, current_range, elapsed_s, 
                         motion_state, temp_stable, cal->gyr[2] * R2D, cal->acc[2]);
            }
        }
        
        // Check calibration completion conditions
        nl_t current_temp_range = g_tcal.actual_temp_max - g_tcal.actual_temp_min;
        
        if (elapsed_s >= max_time_s) {
            break;
        } else if ((current_temp_range) >= (min_temp_range + 1)) { /* make sure last bin collect enough data */
            break;
        }
        
        // Sample rate control (5ms interval)
        nl_mdelay(SAMPLE_DT_MS);
    }
    
    // Finalize calibration with coefficient calculation and save results
    int result = tcal_finalize_data(&g_tcal, cli);
    
    return result;
}

/**
 * @brief Temperature compensation coefficient test using structured data
 * @param cli CLI handle
 * @param argc Argument count
 * @param argv Argument vector
 * @return RT_EOK on success, RT_ERROR on failure
 */
int FTC_TEST(rt_cli_t cli, int argc, char *argv[])
{
    NL_LOG_I("=== TEMPERATURE COMPENSATION COEFFICIENT TEST ===");
    
    // Test data as structured bins - simulate calibration result
    static const struct {
        nl_t temp;
        nl_t acc[3];
        nl_t gyr[3];
    } test_data[] = {
//        {-10.14f, {0.13642f, 0.03441f, 9.86032f}, {-0.00381f, 0.00065f, 0.00532f}},
//        {-9.04f,  {0.13795f, 0.03364f, 9.86096f}, {-0.00376f, 0.00066f, 0.00512f}},
//        {-7.25f,  {0.14025f, 0.03249f, 9.86455f}, {-0.00373f, 0.00066f, 0.00475f}},
//        {-5.36f,  {0.14150f, 0.03128f, 9.86459f}, {-0.00375f, 0.00068f, 0.00440f}},
//        {-3.45f,  {0.14293f, 0.03037f, 9.86620f}, {-0.00359f, 0.00070f, 0.00411f}},
//        {-1.61f,  {0.14404f, 0.03006f, 9.86542f}, {-0.00357f, 0.00079f, 0.00380f}},
//        {0.19f,   {0.14535f, 0.02939f, 9.86567f}, {-0.00354f, 0.00082f, 0.00345f}},
//        {1.92f,   {0.14648f, 0.02952f, 9.86385f}, {-0.00348f, 0.00088f, 0.00320f}},
//        {3.71f,   {0.14765f, 0.02935f, 9.86146f}, {-0.00334f, 0.00092f, 0.00297f}},
//        {5.62f,   {0.14845f, 0.02966f, 9.85779f}, {-0.00309f, 0.00102f, 0.00275f}},
//        {7.59f,   {0.14893f, 0.02957f, 9.84976f}, {-0.00296f, 0.00103f, 0.00248f}},
//        {9.43f,   {0.14859f, 0.02922f, 9.84275f}, {-0.00294f, 0.00100f, 0.00223f}},
//        {11.36f,  {0.14792f, 0.02909f, 9.82998f}, {-0.00292f, 0.00099f, 0.00197f}},
//        {12.97f,  {0.14752f, 0.02945f, 9.81943f}, {-0.00271f, 0.00099f, 0.00182f}},
//        {14.64f,  {0.14656f, 0.03006f, 9.80648f}, {-0.00267f, 0.00095f, 0.00163f}},
//        {16.68f,  {0.14616f, 0.03063f, 9.79727f}, {-0.00263f, 0.00092f, 0.00141f}},
//        {18.64f,  {0.14586f, 0.03077f, 9.78935f}, {-0.00259f, 0.00093f, 0.00119f}},
//        {20.42f,  {0.14524f, 0.03059f, 9.78478f}, {-0.00244f, 0.00092f, 0.00101f}},
//        {22.20f,  {0.14476f, 0.02990f, 9.78450f}, {-0.00227f, 0.00092f, 0.00086f}},
//        {23.98f,  {0.14426f, 0.02914f, 9.78795f}, {-0.00204f, 0.00090f, 0.00074f}},
//        {25.46f,  {0.14324f, 0.02776f, 9.79024f}, {-0.00187f, 0.00084f, 0.00059f}},
//        {27.68f,  {0.14252f, 0.02709f, 9.79088f}, {-0.00153f, 0.00080f, 0.00049f}},
//        {29.61f,  {0.14176f, 0.02622f, 9.79340f}, {-0.00137f, 0.00073f, 0.00034f}},
//        {31.55f,  {0.14061f, 0.02569f, 9.79536f}, {-0.00128f, 0.00072f, 0.00025f}},
//        {33.38f,  {0.13991f, 0.02514f, 9.79366f}, {-0.00108f, 0.00058f, 0.00012f}},
//        {35.29f,  {0.13878f, 0.02463f, 9.79554f}, {-0.00107f, 0.00059f, 0.00010f}},
//        {36.91f,  {0.13824f, 0.02446f, 9.79839f}, {-0.00091f, 0.00059f, 0.00000f}},
//        {38.74f,  {0.13719f, 0.02363f, 9.79770f}, {-0.00082f, 0.00055f, -0.00005f}},
//        {40.55f,  {0.13652f, 0.02336f, 9.79734f}, {-0.00064f, 0.00056f, -0.00014f}},
//        {42.53f,  {0.13486f, 0.02210f, 9.79712f}, {-0.00062f, 0.00049f, -0.00018f}},
//        {44.42f,  {0.13237f, 0.02184f, 9.79372f}, {-0.00063f, 0.00039f, -0.00024f}},
//        {46.15f,  {0.13034f, 0.02139f, 9.78976f}, {-0.00071f, 0.00017f, -0.00022f}},
//        {47.89f,  {0.12846f, 0.02120f, 9.78988f}, {-0.00075f, 0.00007f, -0.00021f}},
//        {49.28f,  {0.12730f, 0.02172f, 9.78938f}, {-0.00082f, -0.00004f, -0.00024f}}
//    };

    {-40.00f, {0.01049f, -0.07843f, 9.94843f}, {0.00148f, 0.00038f, 0.00812f}},
    {-30.00f, {0.01325f, -0.08130f, 9.96201f}, {0.00150f, 0.00045f, 0.00730f}},
    {-20.00f, {0.01739f, -0.08132f, 9.96425f}, {0.00158f, 0.00050f, 0.00566f}},
    {-10.00f, {0.01902f, -0.08273f, 9.95359f}, {0.00226f, 0.00063f, 0.00355f}},
    {0.00f, {0.02567f, -0.08173f, 9.93976f}, {0.00294f, 0.00086f, 0.00236f}},
    {10.00f, {0.02386f, -0.08166f, 9.88862f}, {0.00352f, 0.00102f, 0.00144f}},
    {20.00f, {0.02306f, -0.08107f, 9.85481f}, {0.00411f, 0.00105f, 0.00054f}},
    {30.00f, {0.01790f, -0.08484f, 9.85876f}, {0.00510f, 0.00083f, 0.00001f}},
    {40.00f, {0.01360f, -0.09038f, 9.85503f}, {0.00569f, 0.00061f, -0.00032f}},
    {50.00f, {0.00686f, -0.09021f, 9.85700f}, {0.00594f, 0.00026f, -0.00057f}},
    {60.00f, {-0.00163f, -0.09081f, 9.85331f}, {0.00618f, -0.00031f, -0.00035f}},
    {70.00f, {-0.00854f, -0.08749f, 9.85547f}, {0.00571f, -0.00033f, -0.00001f}},
    {80.00f, {-0.01445f, -0.07876f, 9.84740f}, {0.00499f, -0.00043f, 0.00042f}}
};
    
    uint32_t count = sizeof(test_data) / sizeof(test_data[0]);
    
    // Create test bins from structured data
    tcal_bin_t *test_bins = (tcal_bin_t*)nl_malloc(count * sizeof(tcal_bin_t));
    if (!test_bins) {
        NL_LOG_E("Failed to allocate test bins");
        return RT_ERROR;
    }
    memset(test_bins, 0, count * sizeof(tcal_bin_t));
    // Fill test bins (each bin has one sample)
    for (uint32_t i = 0; i < count; i++) {
        test_bins[i].temp_sum = test_data[i].temp;
        test_bins[i].acc_sum[0] = test_data[i].acc[0];
        test_bins[i].acc_sum[1] = test_data[i].acc[1];
        test_bins[i].acc_sum[2] = test_data[i].acc[2];
        test_bins[i].gyr_sum[0] = test_data[i].gyr[0];
        test_bins[i].gyr_sum[1] = test_data[i].gyr[1];
        test_bins[i].gyr_sum[2] = test_data[i].gyr[2];
        test_bins[i].static_cnt = 1;
    }
    
    // Calculate temperature compensation coefficients using test data
//    fit_all_axes_poly(test_bins, count);
    if(test_bins[count-1].temp_sum / test_bins[count-1].static_cnt - 20 < 30) fit_all_axes_poly_constrained(test_bins, count, &g_tc_params, 20, 1);
    if(test_bins[0].temp_sum / test_bins[0].static_cnt - 20 < 30) fit_all_axes_poly_constrained(test_bins, count, &g_tc_params_low, 20, -1);
    tcal_save_params(1, 1);
    
    nl_free(test_bins);
    
    return RT_EOK;
}

/**
 * @brief Initialize temperature calibration CLI commands
 */
void app_cli_tc_cmd_init()
{
    rt_cli_cmd_create("FTC", FTC, rt_cli_find(CLI1_NAME));
    //rt_cli_cmd_create("FTC_TEST", FTC_TEST, rt_cli_find(CLI1_NAME));
}
