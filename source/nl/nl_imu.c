#include "nl_imu.h"
#include "nl.h"

/**
 * @file nl_imu.c
 * @brief IMU (Inertial Measurement Unit) manager implementation
 *
 * This file implements an IMU management system that:
 * - Manages multiple IMU devices
 * - Handles device initialization and data collection
 * - Applies calibration (misalignment, bias, temperature compensation)
 * - Supports coordinate frame transformations
 * - Combines data from multiple sensors
 *
 * Key features:
 * - Multi-device support with priority handling
 * - Flexible update rates per device
 * - Comprehensive calibration framework
 * - ENU/NED coordinate frame conversion
 * - Temperature compensation
 * - Thread-safe data access
 */

#define NL_DBG_TAG    "nl.imu"
#define NL_DBG_LVL    NL_DBG_INFO
#include "nl_log.h"




// Matrix validation thresholds
#define MAT_DIAG_THRESHOLD     0.2f    // Allowed deviation from 1.0 for diagonal elements
#define MAT_OFFDIAG_THRESHOLD  0.1f    // Allowed absolute value for off-diagonal elements

/**
 * @brief Check if a 3x3 calibration matrix is valid
 * @param mat Pointer to matrix
 * @return true if matrix is valid, false otherwise
 */
static bool is_valid_matrix(const m_t *mat) 
{
    for (int i = 0; i < 3; i++) 
    {
        // Check diagonal elements: should be close to 1.0
        if (fabs(MELEMENT(mat, i, i) - 1.0f) > MAT_DIAG_THRESHOLD) 
        {
            return false;
        }
        
        // Check off-diagonal elements: should be close to 0
        for (int j = 0; j < 3; j++) 
        {
            if (i != j && fabs(MELEMENT(mat, i, j)) > MAT_OFFDIAG_THRESHOLD) 
            {
                return false;
            }
        }
    }
    return true;
}


/**
 * @brief Reset all calibration parameters to default values
 * @param mgr IMU manager instance
 * @return RT_EOK on success
 */
/**
 * @brief Reset all calibration parameters to default values
 */
void nl_imu_reset_calib(nl_imu_mgr_t *mgr)
{
    // Reset calibration data structure
    memset(&mgr->calib, 0, sizeof(nl_imu_calib_t));
    
    // Reset misalignment matrices to identity
    meye(&mgr->gmis);  // Accelerometer
    meye(&mgr->wmis);  // Gyroscope
    meye(&mgr->mmis);  // Magnetometer
    meye(&mgr->urfr);  // User reference frame
    
    // Reset bias vectors to zero
    v3fill(mgr->calib.gb, 0);  // Accelerometer bias
    v3fill(mgr->calib.wb, 0);  // Gyroscope bias
    v3fill(mgr->calib.mb, 0);  // Magnetometer bias
    
    // Reset temperature compensation coefficients
    v3fill(mgr->calib.gb_tc1, 0);  // Accelerometer 1st order temp coef
    v3fill(mgr->calib.gb_tc2, 0);  // Accelerometer 2nd order temp coef
    v3fill(mgr->calib.wb_tc1, 0);  // Gyroscope bias 1st order temp coef
    v3fill(mgr->calib.wb_tc2, 0);  // Gyroscope bias 2nd order temp coef
    v3fill(mgr->calib.ws_tc1, 0);  // Gyroscope scale 1st order temp coef
    v3fill(mgr->calib.ws_tc2, 0);  // Gyroscope scale 2nd order temp coef
    
    v3fill(mgr->temp_cache.cached_gb_drift, 0);
    v3fill(mgr->temp_cache.cached_wb_drift, 0);
    v3fill(mgr->temp_cache.cached_ws_drift, 0);
        
    mgr->calib.wb_cal_temp = 25;      // Default calibration temperature
    mgr->calib.gb_cal_temp = 25;      // Default calibration temperature
}


/**
 * @brief Create an IMU manager instance
 * @return Pointer to the created IMU manager
 */
void nl_imu_mgr_init(nl_imu_mgr_t *mgr)
{
    if (mgr) 
    {
        mgr->dev_list = NULL;
        mgr->en_tc = 1;
        mgr->en_enu2ned = 0;
        
        // Initialize calibration matrices
        minit(&mgr->gmis, 3, 3, mgr->calib.gmis_data);
        minit(&mgr->wmis, 3, 3, mgr->calib.wmis_data);
        minit(&mgr->mmis, 3, 3, mgr->calib.mmis_data);
        minit(&mgr->urfr, 3, 3, mgr->calib.urfr_data);
        
        nl_imu_reset_calib(mgr);
    }
}


/**
 * @brief Add an IMU device to the manager
 * @param mgr IMU manager instance
 * @param dev IMU device to be added
 * @param update_interval_ms Update interval in milliseconds
 * @return RT_EOK on success
 */
int nl_imu_add_device(nl_imu_mgr_t *mgr, nl_imu_dev_t *dev)
{
    if (!mgr || !dev || !dev->ops || !dev->ops->init) {
        NL_LOG_W("cannot add device");
        return -RT_ERROR;
    }
    

    dev->init_ok = (dev->ops->init(dev->user_data, dev->capabilities) == RT_EOK);
    
    dev->last_update_ms = 0;
    
    dev->next = mgr->dev_list;
    
    mgr->dev_list = dev;

    return RT_EOK;
}


static void update_temp_compensation(nl_imu_mgr_t *mgr, nl_t current_temp)
{
    nl_imu_temp_cache_t *cache = &mgr->temp_cache;  // Get pointer to temperature cache structure for storing calculated compensation values

    // Check if temperature change is significant
    if (fabs(current_temp - cache->last_tc_cache_update_temp) < 1.0) {  // Check if temperature difference from last cache update is less than 1 degree, return if change is insignificant to use cached values
      return; // Use cached values
    }

    nl_t temp_curr_scaled = current_temp * 0.01;  // Scale current temperature value (multiply by 0.01)
    nl_t temp_point_scaled = 20.0 * 0.01;        // Segmented temperature point
    // Recalculate compensation values
    nl_t wb_temp_ref = mgr->calib.wb_cal_temp * 0.01;  // Get gyroscope bias calibration reference temperature and scale it
    nl_t gb_temp_ref = mgr->calib.gb_cal_temp * 0.01;  // Get accelerometer bias calibration reference temperature and scale it

    for (int i = 0; i < 3; i++) {  // Iterate through three axes (X, Y, Z axes)
      // Determine temperature compensation coefficients based on current temperature vs segment point
      nl_t *gb_tc1_coeff, *gb_tc2_coeff;  // Accelerometer temperature compensation coefficient pointers
      nl_t *wb_tc1_coeff, *wb_tc2_coeff;  // Gyroscope temperature compensation coefficient pointers
      
      if (temp_curr_scaled < temp_point_scaled) {  // Current temperature is below segment point temperature
          gb_tc1_coeff = mgr->calib.gb_tc1_low;  // Use low temperature first-order coefficients
          gb_tc2_coeff = mgr->calib.gb_tc2_low;  // Use low temperature second-order coefficients
      } else {  // Current temperature is above or equal to segment point temperature
          gb_tc1_coeff = mgr->calib.gb_tc1;     // Use high temperature first-order coefficients
          gb_tc2_coeff = mgr->calib.gb_tc2;     // Use high temperature second-order coefficients
      }
      
      if (temp_curr_scaled < temp_point_scaled) {  // Current temperature is below segment point temperature
          wb_tc1_coeff = mgr->calib.wb_tc1_low;  // Use low temperature first-order coefficients
          wb_tc2_coeff = mgr->calib.wb_tc2_low;  // Use low temperature second-order coefficients
      } else {  // Current temperature is above or equal to segment point temperature
          wb_tc1_coeff = mgr->calib.wb_tc1;     // Use high temperature first-order coefficients
          wb_tc2_coeff = mgr->calib.wb_tc2;     // Use high temperature second-order coefficients
      }
      
      // Calculate accelerometer bias compensation for current temperature
      nl_t temp_diff_curr = temp_curr_scaled - temp_point_scaled;  // Calculate temperature difference
      nl_t tempt_bias_gb1 = gb_tc1_coeff[i] * temp_diff_curr +                      // Calculate accelerometer bias temperature compensation: selected first-order coefficient times temperature difference
                            gb_tc2_coeff[i] * temp_diff_curr * temp_diff_curr;      // Add selected second-order coefficient times temperature difference squared, and cache the result
      
      // Calculate gyroscope bias compensation for current temperature
      nl_t tempt_bias_wb1 = wb_tc1_coeff[i] * temp_diff_curr +                      // Calculate gyroscope bias temperature compensation: selected first-order coefficient times temperature difference
                            wb_tc2_coeff[i] * temp_diff_curr * temp_diff_curr;      // Add selected second-order coefficient times temperature difference squared, and cache the result
      
      if (gb_temp_ref < temp_point_scaled) {  // Reference temperature is below segment point temperature
          gb_tc1_coeff = mgr->calib.gb_tc1_low;  // Use low temperature first-order coefficients
          gb_tc2_coeff = mgr->calib.gb_tc2_low;  // Use low temperature second-order coefficients
      } else {  // Reference temperature is above or equal to segment point temperature
          gb_tc1_coeff = mgr->calib.gb_tc1;     // Use high temperature first-order coefficients
          gb_tc2_coeff = mgr->calib.gb_tc2;     // Use high temperature second-order coefficients
      }
      
      if (wb_temp_ref < temp_point_scaled) {  // Reference temperature is below segment point temperature
          wb_tc1_coeff = mgr->calib.wb_tc1_low;  // Use low temperature first-order coefficients
          wb_tc2_coeff = mgr->calib.wb_tc2_low;  // Use low temperature second-order coefficients
      } else {  // Reference temperature is above or equal to segment point temperature
          wb_tc1_coeff = mgr->calib.wb_tc1;     // Use high temperature first-order coefficients
          wb_tc2_coeff = mgr->calib.wb_tc2;     // Use high temperature second-order coefficients
      }
      
        // Calculate accelerometer bias compensation for reference temperature
        nl_t temp_diff_ref_gb = gb_temp_ref - temp_point_scaled;  // Calculate accelerometer reference temperature difference
        nl_t tempt_bias_gb2 = gb_tc1_coeff[i] * temp_diff_ref_gb +                    // Calculate accelerometer bias temperature compensation: selected first-order coefficient times temperature difference
                            gb_tc2_coeff[i] * temp_diff_ref_gb * temp_diff_ref_gb;  // Add selected second-order coefficient times temperature difference squared, and cache the result

        // Calculate gyroscope bias compensation for reference temperature
        nl_t temp_diff_ref_wb = wb_temp_ref - temp_point_scaled;  // Calculate gyroscope reference temperature difference
        nl_t tempt_bias_wb2 = wb_tc1_coeff[i] * temp_diff_ref_wb +                    // Calculate gyroscope bias temperature compensation: selected first-order coefficient times temperature difference
                            wb_tc2_coeff[i] * temp_diff_ref_wb * temp_diff_ref_wb;  // Add selected second-order coefficient times temperature difference squared, and cache the result

        cache->cached_gb_drift[i] = tempt_bias_gb1 - tempt_bias_gb2;  // Current temperature compensation minus reference temperature compensation
        cache->cached_wb_drift[i] = tempt_bias_wb1 - tempt_bias_wb2;  // Current temperature compensation minus reference temperature compensation

        // Cache gyroscope scale compensation (Z-axis only) - Scale factor always uses normal temperature coefficients
        if (i == 2) {  // Only apply gyroscope scale compensation to Z-axis
            nl_t temp_diff  = temp_curr_scaled - wb_temp_ref;  // Calculate temperature difference
            cache->cached_ws_drift[i] = 1.0f + mgr->calib.ws_tc1[i] * temp_diff  +                    // Calculate gyroscope scale temperature compensation: base value 1.0 plus normal temperature first-order compensation
                                             mgr->calib.ws_tc2[i] * temp_diff  * temp_diff;     // Add normal temperature second-order compensation, and cache the result
        }
    }
    cache->last_tc_cache_update_temp = current_temp;  // Update the temperature value recorded in cache for the last temperature compensation update
}

static void apply_cached_temp_compensation(nl_imu_mgr_t *mgr, nl_imu_data_t *cal)
{
    if (!mgr->en_tc) return;
    
    nl_imu_temp_cache_t *cache = &mgr->temp_cache;
    
    // Apply cached accelerometer compensation
    for (int i = 0; i < 3; i++) {
        cal->acc[i] -= cache->cached_gb_drift[i];
    }
    
    // Apply cached gyroscope compensation
    for (int i = 0; i < 3; i++) {
        cal->gyr[i] -= cache->cached_wb_drift[i];
        if (i == 2) { // Z-axis scale factor
            cal->gyr[i] /= cache->cached_ws_drift[i];
        }
    }
}


/**
 * @brief Apply calibration to array data
 * @note Temperature compensation is based on the difference from cal_temp
 *       cal_temp is the reference temperature point for bias/scale calibration
 */
static void apply_array_calibration(nl_imu_mgr_t *mgr, nl_imu_data_t *raw, nl_imu_data_t *cal)
{
    // Copy temperature data
    cal->temp = raw->temp;
    cal->prs = raw->prs;
    
    // Update temperature compensation if needed
    update_temp_compensation(mgr, raw->temp);
    
    // Apply misalignment + bias in one step
    nl_t tmp[3];
    
    // Accelerometer: misalignment -> bias -> temp compensation
    mvmul(&mgr->gmis, raw->acc, tmp);
    v3sub(cal->acc, tmp, mgr->calib.gb);
    
     // Gyroscope
    mvmul(&mgr->wmis, raw->gyr, tmp);
    v3sub(cal->gyr, tmp, mgr->calib.wb);
    
    apply_cached_temp_compensation(mgr, cal);

    // Process magnetometer data
    v3sub(tmp, raw->mag, mgr->calib.mb);
    mvmul(&mgr->mmis, tmp, cal->mag);
    
    // Apply user reference frame rotation if valid   
    mvmul(&mgr->urfr, cal->acc, tmp);
    v3copy(cal->acc, tmp);
    
    // Apply to gyrscope data
    mvmul(&mgr->urfr, cal->gyr, tmp);
    v3copy(cal->gyr, tmp);
    
    // Apply to magnetometer data
    mvmul(&mgr->urfr, cal->mag, tmp);
    v3copy(cal->mag, tmp);
}



/**
 * @brief Apply rotation transformation to sensor data
 * @param dev IMU device containing rotation configuration
 * @param data 3-axis sensor data to be transformed
 */
static void apply_rotation(nl_imu_rotation_t rotation, nl_t data[3])
{
    nl_t x = data[0];
    nl_t y = data[1];
    nl_t z = data[2];

    switch (rotation) {
        case NL_IMU_ROT_Z_P90:  // Rotate +90 degrees around Z axis
            data[0] = y;
            data[1] = -x;
            break;
        case NL_IMU_ROT_Z_N90:  // Rotate -90 degrees around Z axis
            data[0] = -y;
            data[1] = x;
            break;
        case NL_IMU_ROT_Z_P180:
            data[0] = -x;
            data[1] = -y;
            break;
        case NL_IMU_ROT_X_180:  // Rotate 180 degrees around X axis
            data[1] = -y;
            data[2] = -z;
            break;
        case NL_IMU_ROT_Y_180:  // Rotate 180 degrees around Y axis
            data[0] = -x;
            data[2] = -z;
            break;
        case NL_IMU_ROT_Y_180_ZN90: // // Transform to ENU from NED
            data[0] = y;            // ENU.y -> NED.x
            data[1] = x;            // ENU.x -> NED.y
            data[2] = -z;           // ENU.z -> NED.-z
            break;
        case NL_IMU_ROT_Y_180_ZP90:
            data[0] = -y;
            data[1] = -x;
            data[2] = -z;
            break;
        case NL_IMU_ROT_NONE:
            break;
        default:
            NL_LOG_W("no rotation found!");
            break;
    }
}

/**
 * @brief Read data from a single IMU device
 * @param dev IMU device to read from
 * @param raw Raw sensor data output
 * @return RT_EOK on success
 */
static int nl_imu_read_single(nl_imu_dev_t *dev, nl_imu_data_t *raw)
{
    // Read raw data from device
    dev->ops->read_raw(dev->user_data, dev->capabilities, raw);

    // Apply rotation based on device capabilities
    if (dev->capabilities & IMU_CAP_ACC_XYZ) 
    {
        apply_rotation(dev->rotation, raw->acc);
    }
    
    if (dev->capabilities & (IMU_CAP_GYR_XYZ | IMU_CAP_GYR_Z)) 
    {
        apply_rotation(dev->rotation, raw->gyr);
    }
    
    if (dev->capabilities & IMU_CAP_MAG)
    {
        apply_rotation(dev->rotation, raw->mag);
    }
    
    return RT_EOK;
}

/**
 * @brief Read and combine data from all IMU devices in array
 * @param mgr IMU manager instance
 * @param data Combined sensor data output
 * @return RT_EOK on success
 */
void nl_imu_read_array(nl_imu_mgr_t *mgr)
{
    nl_imu_data_t single_snesor;
    nl_t acc[3] = {0};
    nl_t gyr[3] = {0};
    nl_t mag[3] = {0};
    nl_t temp = 0;
    nl_t prs = 0;
    
    uint32_t current_ms = nl_get_sys_ms();  // Get current time in milliseconds
    
    mgr->acc_count = 0;
    mgr->gyr_xy_count = 0;
    mgr->gyr_z_count = 0;
    mgr->mag_count = 0;
    mgr->temp_count = 0;
    mgr->prs_count = 0;
    
    for (nl_imu_dev_t *dev = mgr->dev_list; dev; dev = dev->next) 
    {
        // Check if device needs update
        if (current_ms - dev->last_update_ms < dev->update_interval_ms)  continue;
        if (!dev->init_ok)  continue;
        if (nl_imu_read_single(dev, &single_snesor) != RT_EOK) continue;
        
        dev->last_update_ms = current_ms;

        // Accumulate sensor data based on capabilities
        if (dev->capabilities & IMU_CAP_ACC_XYZ) 
        {
            v3add2(acc, single_snesor.acc);
            mgr->acc_count++;
        }
        
        if (dev->capabilities & IMU_CAP_GYR_XYZ) 
        {
            v3add2(gyr, single_snesor.gyr);
            mgr->gyr_xy_count++;
            mgr->gyr_z_count++;
        } 
        
        if (dev->capabilities & IMU_CAP_GYR_Z) 
        {
            /* 4 times weight */
            gyr[2] += single_snesor.gyr[2] * 50;
            mgr->gyr_z_count += 50;
        }

        if (dev->capabilities & IMU_CAP_MAG) {
            v3add2(mag, single_snesor.mag);
            mgr->mag_count++;
        }

        if (dev->capabilities & IMU_CAP_TEMP) {
            temp += single_snesor.temp;
            mgr->temp_count++;
        }
        
        if (dev->capabilities & IMU_CAP_PRS) {
            prs += single_snesor.prs;
            mgr->prs_count++;
        }
    }

    // Calculate averages
    nl_enter_critical();
    
    if (mgr->acc_count) 
    {
        mgr->buffer.raw.acc[0] = acc[0] / mgr->acc_count;
        mgr->buffer.raw.acc[1] = acc[1] / mgr->acc_count;
        mgr->buffer.raw.acc[2] = acc[2] / mgr->acc_count;
    }
    
    if(mgr->gyr_xy_count)
    {
        mgr->buffer.raw.gyr[0] = gyr[0] / mgr->gyr_xy_count;
        mgr->buffer.raw.gyr[1] = gyr[1] / mgr->gyr_xy_count;
    }

    if(mgr->gyr_z_count)
    {
        mgr->buffer.raw.gyr[2] = gyr[2] / mgr->gyr_z_count;
    }

    if (mgr->mag_count) 
    {
        mgr->buffer.raw.mag[0] = mag[0] / mgr->mag_count;
        mgr->buffer.raw.mag[1] = mag[1] / mgr->mag_count;
        mgr->buffer.raw.mag[2] = mag[2] / mgr->mag_count;
    }

    if (mgr->temp_count) 
    {
        mgr->buffer.raw.temp = temp / mgr->temp_count;
    }

    if (mgr->prs_count > 0) 
    {
        mgr->buffer.raw.prs = prs / mgr->prs_count;
    }
    
    if(mgr->en_enu2ned)
    {
        apply_rotation(NL_IMU_ROT_Y_180_ZN90, mgr->buffer.raw.acc);
        apply_rotation(NL_IMU_ROT_Y_180_ZN90, mgr->buffer.raw.gyr);
        apply_rotation(NL_IMU_ROT_Y_180_ZN90, mgr->buffer.raw.mag);
    }
    
    // Apply calibration to averaged data
    apply_array_calibration(mgr, &mgr->buffer.raw, &mgr->buffer.cal);

    nl_exit_critical();
}

/**
 * @brief Enable/Disable ENU to NED coordinate frame conversion
 * 
 * @param mgr IMU manager instance
 * @param enable 1 to enable ENU->NED conversion, 0 to keep ENU frame
 * 
 * @note When enabled, the coordinate transformation follows:
 *       NED.x = ENU.y
 *       NED.y = ENU.x
 *       NED.z = -ENU.z
 *       This conversion is applied to both raw and calibrated data
 */
void nl_imu_set_enu2ned(nl_imu_mgr_t *mgr, uint8_t enable)
{
    mgr->en_enu2ned = enable;
}

/**
 * @brief Enable/Disable temperature compensation
 * 
 * @param mgr IMU manager instance
 * @param enable 1 to enable temperature compensation, 0 to disable
 * 
 * @note When enabled, applies:
 *       - Accelerometer bias temperature compensation
 *       - Gyroscope bias and scale factor temperature compensation
 *       Temperature compensation parameters must be loaded via nl_imu_load_temp_calib()
 */
void nl_imu_set_tc(nl_imu_mgr_t *mgr, uint8_t enable)
{
    mgr->en_tc = enable;
}


/**
 * @brief Load accelerometer misalignment matrix
 * @param mgr IMU manager instance
 * @param mis_data Misalignment matrix (3x3)
 * @return RT_EOK on success
 */
int nl_imu_load_acc_mis(nl_imu_mgr_t *mgr, const nl_t *mis_data)
{
    if (!mgr || !mis_data)  return -RT_ERROR;

    // Validate misalignment matrix
    m_t tmp;
    
    minit(&tmp, 3, 3, (nl_t *)mis_data);
    
    if (!is_valid_matrix(&tmp)) 
    {
        NL_LOG_W("invalid acc mis matrix");
        return -RT_ERROR;
    }

    memcpy(mgr->calib.gmis_data, mis_data, sizeof(nl_t) * 9);
    return RT_EOK;
}

/**
 * @brief Load accelerometer bias vector
 * @param mgr IMU manager instance
 * @param bias Bias vector (3x1)
 * @return RT_EOK on success
 */
int nl_imu_load_acc_bias(nl_imu_mgr_t *mgr, const nl_t *bias)
{
    if (!mgr || !bias)  return -RT_ERROR;

    memcpy(mgr->calib.gb, bias, sizeof(nl_t) * 3);
    return RT_EOK;
}

/**
 * @brief Load gyrscope misalignment matrix
 * @param mgr IMU manager instance
 * @param mis_data Misalignment matrix (3x3)
 * @return RT_EOK on success
 */
int nl_imu_load_gyr_mis(nl_imu_mgr_t *mgr, const nl_t *mis_data)
{
    if (!mgr || !mis_data)  return -RT_ERROR;

    // Validate misalignment matrix
    m_t tmp;
    minit(&tmp, 3, 3, (nl_t *)mis_data);
    if (!is_valid_matrix(&tmp)) 
    {
        NL_LOG_W("invalid gyr mis matrix");
        return -RT_ERROR;
    }

    memcpy(mgr->calib.wmis_data, mis_data, sizeof(nl_t) * 9);
    return RT_EOK;
}

/**
 * @brief Load gyrscope bias vector
 * @param mgr IMU manager instance
 * @param bias Bias vector (3x1)
 * @return RT_EOK on success
 */
int nl_imu_load_gyr_bias(nl_imu_mgr_t *mgr, const nl_t *bias)
{
    if (!mgr || !bias)  return -RT_ERROR;

    memcpy(mgr->calib.wb, bias, sizeof(nl_t) * 3);
    return RT_EOK;
}

/**
 * @brief Load magnetometer misalignment matrix
 * @param mgr IMU manager instance
 * @param mis_data Misalignment matrix (3x3)
 * @return RT_EOK on success
 */
int nl_imu_load_mag_mis(nl_imu_mgr_t *mgr, const nl_t *mis_data)
{
    if (!mgr || !mis_data)  return -RT_ERROR;

    // Validate misalignment matrix
    m_t tmp;
    minit(&tmp, 3, 3, (nl_t *)mis_data);
    if (!is_valid_matrix(&tmp)) 
    {
        NL_LOG_W("invalid mag mis matrix");
        return -RT_ERROR;
    }

    memcpy(mgr->calib.mmis_data, mis_data, sizeof(nl_t) * 9);
    return RT_EOK;
}

const nl_t urfr_mat_template[][10] = {
    {1, 0, 0, 0, 1, 0, 0, 0, 1},     // val == 0x00
    {1, 0, 0, 0, 0, 1, 0, -1, 0},    // val == 0x01
    {1, 0, 0, 0, 0, -1, 0, 1, 0},    // val == 0x02
    {0, 0, -1, 0, 1, 0, 1, 0, 0},    // val == 0x03
    {0, 0, 1, 0, 1, 0, -1, 0, 0},    // val == 0x04
    {1, 0, 0, 0, -1, 0, 0, 0, -1},   // val == 0x05
    {-1, 0, 0, 0, 1, 0, 0, 0, -1},   // val == 0x06
    {0, -1, 0, 1, 0, 0, 0, 0, 1},    // val == 0x07
    {0, 1, 0, -1, 0, 0, 0, 0, 1},    // val == 0x08
    {-1, 0, 0, 0, -1, 0, 0, 0, 1}    // val == 0x09
};


/**
 * @brief Set URFR matrix using template index
 * @param mgr IMU manager instance
 * @param idx Template index (0-7)
 * @return RT_EOK on success, -RT_ERROR on failure
 */
int nl_imu_set_urfr_idx(nl_imu_mgr_t *mgr, int idx)
{
    if (!mgr) {
        NL_LOG_E("IMU manager is NULL");
        return -RT_ERROR;
    }
    
    if (idx < 0 || idx >= ARRAY_SIZE(urfr_mat_template)) {
        NL_LOG_E("Invalid URFR index: %d (valid range: 0-%d)", idx, ARRAY_SIZE(urfr_mat_template) - 1);
        return -RT_ERROR;
    }
    
    // Copy template data to URFR matrix
    vcopy(mgr->calib.urfr_data, (nl_t *)urfr_mat_template[idx], 9);

    return RT_EOK;
}

/**
 * @brief Load user reference frame rotation matrix directly
 * @param mgr IMU manager instance
 * @param urfr_data URFR matrix data (3x3, 9 elements)
 * @return RT_EOK on success, -RT_ERROR on failure
 */
int nl_imu_load_urfr(nl_imu_mgr_t *mgr, const nl_t *urfr_data)
{
    if (!mgr || !urfr_data) {
        NL_LOG_E("Invalid parameters");
        return -RT_ERROR;
    }

    memcpy(mgr->calib.urfr_data, urfr_data, sizeof(nl_t) * 9);
    
    NL_LOG_I("URFR matrix loaded successfully");
    return RT_EOK;
}

/**
 * @brief Get current URFR matrix template index
 * @param mgr IMU manager instance
 * @return Template index (0-4) if matches a template, -1 if no match or custom matrix
 */
int nl_imu_get_urfr_idx(nl_imu_mgr_t *mgr)
{
    if (!mgr) {
        NL_LOG_E("IMU manager is NULL");
        return -1;
    }
    
    // Compare current URFR matrix with all templates
    for (int template_idx = 0; template_idx < ARRAY_SIZE(urfr_mat_template); template_idx++) {
        bool match = true;
        for (int j = 0; j < 9; j++) {
            if (fabsf(mgr->calib.urfr_data[j] - urfr_mat_template[template_idx][j]) > 1e-6f) {
                match = false;
                break;
            }
        }
        if (match) {
            return template_idx;
        }
    }
    
    // No template match found - custom matrix
    return -1;
}

/**
 * @brief Load magnetometer bias vector
 * @param mgr IMU manager instance
 * @param bias Bias vector (3x1)
 * @return RT_EOK on success
 */
int nl_imu_load_mag_bias(nl_imu_mgr_t *mgr, const nl_t *bias)
{
    if (!mgr || !bias)  return -RT_ERROR;

    memcpy(mgr->calib.mb, bias, sizeof(nl_t) * 3);
    return RT_EOK;
}


/**
 * @brief Load all 2nd order temperature compensation parameters
 * @param mgr IMU manager instance
 * @param gb_tc1 Accelerometer bias 1st order temperature coefficients (3x1)
 * @param gb_tc2 Accelerometer bias 2nd order temperature coefficients (3x1)
 * @param wb_tc1 Gyroscope bias 1st order temperature coefficients (3x1)
 * @param wb_tc2 Gyroscope bias 2nd order temperature coefficients (3x1)
 * @param ws_tc1 Gyroscope scale 1st order temperature coefficients (3x1)
 * @param ws_tc2 Gyroscope scale 2nd order temperature coefficients (3x1)
 * @param cal_temp Calibration reference temperature in Celsius
 * @return RT_EOK on success
 * 
 * @note Temperature coefficients are applied as:
 *       - Accelerometer: acc_corrected = acc_raw - (gb + gb_tc1*dT + gb_tc2*dT^2)
 *       - Gyroscope: gyr_corrected = (gyr_raw - (wb + wb_tc1*dT + wb_tc2*dT^2)) * (1 - ws_tc1*dT - ws_tc2*dT^2)
 *       where dT = (current_temp - cal_temp) / 100
 */
int nl_imu_load_temp_calib(nl_imu_mgr_t *mgr, 
                          const nl_t *gb_tc1, const nl_t *gb_tc2,
                          const nl_t *wb_tc1, const nl_t *wb_tc2,
                          const nl_t *ws_tc1, const nl_t *ws_tc2,
                          const nl_t *gb_tc1_low, const nl_t *gb_tc2_low,
                          const nl_t *wb_tc1_low, const nl_t *wb_tc2_low,
                          nl_t gb_cal_temp, nl_t wb_cal_temp)           
{
    if (!mgr || !gb_tc1 || !gb_tc2 || !wb_tc1 || !wb_tc2 || 
        !ws_tc1 || !ws_tc2 || !gb_tc1_low || !gb_tc2_low || 
        !wb_tc1_low || !wb_tc2_low) 
    {
        NL_LOG_W("invalid data: temp_calib");
        return -RT_ERROR;
    }

    // High temperature calibration parameters
    memcpy(mgr->calib.gb_tc1, gb_tc1, sizeof(nl_t) * 3);
    memcpy(mgr->calib.gb_tc2, gb_tc2, sizeof(nl_t) * 3);
    memcpy(mgr->calib.wb_tc1, wb_tc1, sizeof(nl_t) * 3);
    memcpy(mgr->calib.wb_tc2, wb_tc2, sizeof(nl_t) * 3);
    memcpy(mgr->calib.ws_tc1, ws_tc1, sizeof(nl_t) * 3);
    memcpy(mgr->calib.ws_tc2, ws_tc2, sizeof(nl_t) * 3);
    
    // Low temperature calibration parameters
    memcpy(mgr->calib.gb_tc1_low, gb_tc1_low, sizeof(nl_t) * 3);
    memcpy(mgr->calib.gb_tc2_low, gb_tc2_low, sizeof(nl_t) * 3);
    memcpy(mgr->calib.wb_tc1_low, wb_tc1_low, sizeof(nl_t) * 3);
    memcpy(mgr->calib.wb_tc2_low, wb_tc2_low, sizeof(nl_t) * 3);
    
    mgr->calib.gb_cal_temp = gb_cal_temp;
    mgr->calib.wb_cal_temp = wb_cal_temp;
    
    return RT_EOK;
}

/**
 * @brief Check if temperature compensation parameters are valid
 * @param mgr IMU manager instance
 * @return 1 if all parameters are non-zero, 0 otherwise
 */
static uint8_t nl_imu_tc_params_valid(nl_imu_mgr_t *mgr)
{
    // Check if ALL temperature compensation coefficients are non-zero
    for (int i = 0; i < 3; i++) {
        if (mgr->calib.gb_tc1[i] == 0.0f || mgr->calib.gb_tc2[i] == 0.0f ||
            mgr->calib.wb_tc1[i] == 0.0f || mgr->calib.wb_tc2[i] == 0.0f ||
            mgr->calib.gb_tc1_low[i] == 0.0f || mgr->calib.gb_tc2_low[i] == 0.0f ||
            mgr->calib.wb_tc1_low[i] == 0.0f || mgr->calib.wb_tc2_low[i] == 0.0f) {
            return 0;  // Any zero coefficient makes it invalid
        }
    }
    
    // Check gyroscope scale factor compensation (Z-axis only)
    if (mgr->calib.ws_tc1[2] == 0.0f || mgr->calib.ws_tc2[2] == 0.0f) {
        return 0;
    }
    
    return 1;  // All coefficients are non-zero
}

/**
 * @brief Get temperature compensation parameters validity status
 * @param mgr IMU manager instance
 * @return 1 if valid, 0 if invalid
 */
uint8_t nl_imu_get_tc_status(nl_imu_mgr_t *mgr)
{
    if (!mgr) return 0;
    return nl_imu_tc_params_valid(mgr);
}


/**
 * @brief Get the number of successfully initialized IMUs with 3-axis gyroscope
 * @param mgr IMU manager instance
 * @return Number of initialized IMUs with gyroscope
 */
uint8_t nl_imu_get_gyr_count(nl_imu_mgr_t *mgr)
{
    if (!mgr) return 0;
    
    uint8_t imu_count = 0;
    nl_imu_dev_t *dev = mgr->dev_list;
    
    // Count only devices with 3-axis gyroscope
    while (dev) 
    {
        if (dev->init_ok && (dev->capabilities & IMU_CAP_GYR_XYZ)) 
        {
            imu_count++;
        }
        dev = dev->next;
    }
    
    return imu_count;
}


