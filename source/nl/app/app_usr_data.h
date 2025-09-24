/**
 * @file app_usr_data.h
 * @brief User output data management for IMU/AHRS system
 * @author Alex Yang
 * @version 1.0
 * @date 2025
 */

#ifndef __NL_USR_DATA_H__
#define __NL_USR_DATA_H__

#include "nl_def.h"
#include "nl_imu.h"

/* user output data managment */

/* Main status bit definitions */
#define MAIN_STATUS_WB_CONV_BIT          3   /**< Gyroscope bias convergence status */
#define MAIN_STATUS_MAG_DIST_BIT         4   /**< Magnetic disturbance detection */
#define MAIN_STATUS_MAG_AIDING_BIT      10   /**< Magnetometer aiding status */
#define MAIN_STATUS_UTC_TIME_BIT        11  /**< UTC time synchronization status */
#define MAIN_STATUS_SOUT_PULSE_BIT      12  /**< SOUT pulse output status */

/**
 * @brief User data structure containing processed IMU/AHRS output
 */
typedef struct {
    uint32_t sys_time;      /**< System timestamp in milliseconds */
    nl_t acc[3];           /**< Processed accelerometer data [m/s²] */
    nl_t gyr[3];           /**< Processed gyroscope data [rad/s] */
    nl_t mag[3];           /**< Magnetometer data [uT] */
    nl_t temp;             /**< Temperature [°C] */
    nl_t prs;              /**< Pressure [Pa] */
    nl_t q[4];             /**< Quaternion [w,x,y,z] */
    att_t att;             /**< Attitude (roll, pitch, yaw) */
    nl_t incli_x;          /* roll */
    nl_t incli_y;          /* pitch */
    uint16_t main_status;  /**< Main status register - see MAIN_STATUS_*_BIT definitions */
} topic_usr_data_t;

/**
 * @brief Initialize user data processing module
 * @param topic_name RT-Thread topic name for data publishing
 * @param coord Coordinate system (ENU/NED)
 * @return RT_EOK on success, -RT_ERROR on failure
 */
int app_usr_data_init(const char *topic_name, uint8_t coord);

/**
 * @brief Configure accelerometer low-pass filter
 * @param fc Cutoff frequency [Hz]
 * @param sample_freq Sampling frequency [Hz]
 * @param enable Filter enable flag (1=enable, 0=disable)
 * @return RT_EOK on success, -RT_ERROR on failure
 */
int app_usr_data_config_acc_filter(nl_t fc, nl_t sample_freq, uint8_t enable);

/**
 * @brief Configure gyroscope low-pass filter
 * @param fc Cutoff frequency [Hz]
 * @param sample_freq Sampling frequency [Hz]
 * @param enable Filter enable flag (1=enable, 0=disable)
 * @return RT_EOK on success, -RT_ERROR on failure
 */
int app_usr_data_config_gyr_filter(nl_t fc, nl_t sample_freq, uint8_t enable);

/**
 * @brief Process IMU data and publish to RT-Thread topic
 * @param q Current quaternion from AHRS algorithm
 * @param input_imu Raw IMU sensor data
 * @param wb Gyroscope bias vector
 * @return RT_EOK on success, -RT_ERROR on failure
 */
int app_usr_data_process_and_publish(const nl_t *q, const nl_imu_data_t *input_imu, const nl_t *wb);


int app_usr_data_slow_process_and_publish(const uint8_t mag_aiding_enable, const uint8_t wb_not_conv, const uint8_t mag_dist);


/**
 * @brief Lock/unlock attitude calculation
 * @param enable Lock flag (1=lock attitude, 0=unlock)
 */
void app_usr_data_lock_att(uint8_t enable);

/**
 * @brief Set quaternion offset for attitude correction
 * @param q_ofs Quaternion offset [w,x,y,z]
 */
void app_usr_data_set_quat_offset(nl_t *q_ofs);

/**
 * @brief Configure inclinometer output parameters
 * @param enable_2pi Enable 2p range for roll (0-2p instead of -p to p)
 * @param negate_x Negate roll output
 * @param negate_y Negate pitch output
 */
void app_usr_data_config_inclinometer(uint8_t enable_2pi, uint8_t negate_x, uint8_t negate_y);

#endif
