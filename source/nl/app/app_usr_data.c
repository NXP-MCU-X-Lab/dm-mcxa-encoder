/**
 * @file app_usr_data.c
 * @brief User output data management implementation
 * @author Alex Yang
 * @version 1.0
 * @date 2025
 */

#include "app_usr_data.h"
#include "nl_imu.h"
#include "rt_topic.h"
#include "nl_filter.h"
#include "nl_gps_time_sync.h"

#include <rtthread.h>

#define NL_DBG_TAG    "nl.usr_data"
#define NL_DBG_LVL    NL_DBG_ERROR
#include "nl_log.h"

/* Private variables */
static topic_usr_data_t         usr_data;              /**< User output data structure */
static biquad_filter_t          acc_filters[3];        /**< Accelerometer biquad filters for X,Y,Z axes */
static biquad_filter_t          gyr_filters[3];        /**< Gyroscope biquad filters for X,Y,Z axes */
static uint8_t                  acc_filter_enabled = 0; /**< Accelerometer filter enable flag */
static uint8_t                  gyr_filter_enabled = 0; /**< Gyroscope filter enable flag */
static rt_topic_t               topic_usr_data = RT_NULL; /**< RT-Thread topic handle */
static uint8_t                  init_ok = 0;           /**< Initialization status flag */
static uint8_t                  lock_att = 0;          /**< Attitude lock flag */
static nl_t                     q_uofs[4];             /**< User attitude offset quaternion */

/* Inclinometer configuration variables */
static uint8_t                  incli_enable_2pi = 0;  /**< Enable 2p range for roll */
static uint8_t                  incli_negate_x = 0;    /**< Negate roll output */
static uint8_t                  incli_negate_y = 0;    /**< Negate pitch output */


/**
 * @brief Initialize user data processing module
 */
int app_usr_data_init(const char *topic_name, uint8_t coord)
{
    /* Initialize output data structure */
    memset(&usr_data, 0, sizeof(topic_usr_data_t));
    
    /* Initialize quaternion offset to identity */
    qidentity(q_uofs);
    qidentity(usr_data.q);
    usr_data.att.coord = coord;
    q2att(usr_data.q, &usr_data.att);
    
    /* Create RT-Thread topic for publishing processed IMU data */
    topic_usr_data = rt_topic_create(topic_name, &usr_data, sizeof(topic_usr_data_t));
    if (!topic_usr_data) {
        NL_LOG_E("Failed to create imu_usr topic");
        return -RT_ERROR;
    }
    
    /* Initialize biquad filter states to zero */
    for(int i = 0; i < 3; i++) {
        memset(&acc_filters[i], 0, sizeof(biquad_filter_t));
        memset(&gyr_filters[i], 0, sizeof(biquad_filter_t));
    }
    
    /* Initialize inclinometer configuration */
    incli_enable_2pi = 0;
    incli_negate_x = 0;
    incli_negate_y = 0;
    
    init_ok = 1;
    NL_LOG_I("IMU user process module initialized");
    
    return RT_EOK;
}

/**
 * @brief Configure accelerometer low-pass filter parameters
 */
int app_usr_data_config_acc_filter(nl_t fc, nl_t sample_freq, uint8_t enable)
{
    if (!init_ok) {
        NL_LOG_E("Module not initialized");
        return -RT_ERROR;
    }
    
    acc_filter_enabled = enable;
    
    if (enable) {
        /* Initialize biquad low-pass filters for all 3 axes */
        for(int i = 0; i < 3; i++) {
            biquad_lowpass_init(&acc_filters[i], fc, sample_freq);
        }
        NL_LOG_I("Accelerometer filter configured: fc=%.1f Hz, fs=%.1f Hz", fc, sample_freq);
    } else {
        NL_LOG_I("Accelerometer filter disabled");
    }
    
    return RT_EOK;
}

/**
 * @brief Configure gyroscope low-pass filter parameters
 */
int app_usr_data_config_gyr_filter(nl_t fc, nl_t sample_freq, uint8_t enable)
{
    if (!init_ok) {
        NL_LOG_E("Module not initialized");
        return -RT_ERROR;
    }
    
    gyr_filter_enabled = enable;
    
    if (enable) {
        /* Initialize biquad low-pass filters for all 3 axes */
        for(int i = 0; i < 3; i++) {
            biquad_lowpass_init(&gyr_filters[i], fc, sample_freq);
        }
        NL_LOG_I("Gyroscope filter configured: fc=%.1f Hz, fs=%.1f Hz", fc, sample_freq);
    } else {
        NL_LOG_I("Gyroscope filter disabled");
    }
    
    return RT_EOK;
}

/**
 * @brief Set user-defined quaternion offset for attitude correction
 */
void app_usr_data_set_quat_offset(nl_t *q_ofs)
{
    vcopy(q_uofs, q_ofs, 4);
}

/**
 * @brief Lock or unlock attitude calculation
 */
void app_usr_data_lock_att(uint8_t enable)
{
    lock_att = enable;
}

/**
 * @brief Configure inclinometer output parameters
 */
void app_usr_data_config_inclinometer(uint8_t enable_2pi, uint8_t negate_x, uint8_t negate_y)
{
    incli_enable_2pi = enable_2pi;
    incli_negate_x = negate_x;
    incli_negate_y = negate_y;
    
    NL_LOG_I("Inclinometer config: 2p=%d, neg_x=%d, neg_y=%d", enable_2pi, negate_x, negate_y);
}

int app_usr_data_slow_process_and_publish(const uint8_t mag_aiding_enable, const uint8_t wb_not_conv, const uint8_t mag_dist)                                  
{
    const uint16_t update_mask = (1 << MAIN_STATUS_MAG_AIDING_BIT) |
                                (1 << MAIN_STATUS_WB_CONV_BIT) |
                                (1 << MAIN_STATUS_MAG_DIST_BIT) |
                                (1 << MAIN_STATUS_UTC_TIME_BIT);
    
    // Force MAG_DIST to 0 when MAG_AIDING is disabled
    uint8_t effective_mag_dist = mag_aiding_enable ? mag_dist : 0;
    
    uint16_t new_bits = (mag_aiding_enable << MAIN_STATUS_MAG_AIDING_BIT) |
                       (wb_not_conv << MAIN_STATUS_WB_CONV_BIT) |
                       (effective_mag_dist << MAIN_STATUS_MAG_DIST_BIT) |
                       ((!nl_gps_time_is_locked()) << MAIN_STATUS_UTC_TIME_BIT);
    
    usr_data.main_status = (usr_data.main_status & ~update_mask) | new_bits;
    
    return 0;
}

/**
 * @brief Process raw IMU data and publish to RT-Thread topic
 */
int app_usr_data_process_and_publish(const nl_t *q, const nl_imu_data_t *input_imu, const nl_t *wb)
{
    if (!init_ok || !topic_usr_data) {
        return -RT_ERROR;
    }
    
    nl_t con_qofs[4];                   /* Conjugate of quaternion offset */
    nl_t gyr_removed_runtime_bias[3];   /* Gyroscope data with bias removed */
    nl_t acc_rotated[3];                /* Accelerometer data after rotation */
    nl_t gyr_rotated[3];                /* Gyroscope data after rotation */
    
    /* Convert user quaternion offset to conjugate for inverse rotation */
    qconj((nl_t*)q_uofs, con_qofs);
    
    /* Remove runtime bias from gyroscope measurements */
    v3sub(gyr_removed_runtime_bias, (nl_t*)input_imu->gyr, (nl_t*)wb);
    
    /* Apply quaternion rotation to align sensor frame with user frame */
    qmulv(con_qofs, (nl_t*)input_imu->acc, acc_rotated);
    qmulv(con_qofs, gyr_removed_runtime_bias, gyr_rotated);
    
    /* Process accelerometer data with optional filtering */
    if (acc_filter_enabled) {
        for(int i = 0; i < 3; i++) {
            usr_data.acc[i] = biquad_update(&acc_filters[i], acc_rotated[i]);
        }
    } else {
        v3copy(usr_data.acc, acc_rotated);
    }

    /* Process gyroscope data with optional filtering */
    if (gyr_filter_enabled) {
        for(int i = 0; i < 3; i++) {
            usr_data.gyr[i] = biquad_update(&gyr_filters[i], gyr_rotated[i]);
        }
    } else {
        v3copy(usr_data.gyr, gyr_rotated);
    }
    
    /* Copy magnetometer, temperature and pressure data directly */
    v3copy(usr_data.mag, (nl_t *)input_imu->mag);
    usr_data.temp = input_imu->temp;
    usr_data.prs = input_imu->prs;
                             
    /* Generate timestamp and update UTC time status (bit 11) */
    if(nl_gps_time_is_locked())
    {
        usr_data.sys_time = nl_gps_time_get_daily_ms();
    }
    else
    {
        usr_data.sys_time = rt_tick_get_millisecond();
    }

    /* Calculate quaternion and attitude if not locked */
    if(!lock_att) 
    {
        /* Apply user offset to AHRS quaternion */
        qmul(q, q_uofs, usr_data.q);
        /* Convert quaternion to Euler angles */
        q2att(usr_data.q, &usr_data.att);
        
        /* Process inclinometer outputs */
        usr_data.incli_x = incli_negate_x ? -usr_data.att.roll : usr_data.att.roll;
        usr_data.incli_x = (incli_enable_2pi && usr_data.incli_x < 0) ? usr_data.incli_x + 2*M_PI : usr_data.incli_x;
        usr_data.incli_y = incli_negate_y ? -usr_data.att.pitch : usr_data.att.pitch;
    }

    /* Publish processed data to RT-Thread topic */
    rt_topic_publish(topic_usr_data, &usr_data);
    
    return RT_EOK;
}

