#ifndef __NL_IMU_H__
#define __NL_IMU_H__

#include "nl_def.h"

/* IMU device capability flags */
#define IMU_CAP_ACC_XYZ     (1<<0)   // 3-axis accelerometer
#define IMU_CAP_GYR_XYZ     (1<<1)   // 3-axis gyroscope  
#define IMU_CAP_GYR_Z       (1<<2)   // Single Z-axis gyro
#define IMU_CAP_MAG         (1<<3)   // Magnetomete
#define IMU_CAP_TEMP        (1<<4)   // Gyroscope temperature
#define IMU_CAP_PRS         (1<<5)   // air pressure

typedef enum
{
    NL_IMU_ROT_NONE = 0,
    NL_IMU_ROT_Z_P90,
    NL_IMU_ROT_Z_P180,
    NL_IMU_ROT_Z_N90,
    NL_IMU_ROT_X_180,
    NL_IMU_ROT_Y_180,
    NL_IMU_ROT_Y_180_ZP90,
    NL_IMU_ROT_Y_180_ZN90,  // Transform to ENU from NED
} nl_imu_rotation_t;


/* IMU device raw data structure */
typedef struct {
    nl_t acc[3];    // virutal sensor acceleration in m/s^2
    nl_t gyr[3];    // virutal sensor angular velocity in rad/s
    nl_t mag[3];    // Magnetic field in uT  (1 uT = 10 mGauss , earth mag field::250 - 650 mGauss ? 25 - 64 uT) */
    nl_t temp;      // Temperature in degrees Celsius
    nl_t prs;       // Pressure in hPa (hectopascals)
} nl_imu_data_t;

/* Combined array data structure */
typedef struct {
    nl_imu_data_t raw;  /* sensor raw measuerments */
    nl_imu_data_t cal;  /* calibrated(factory) sensor measurements */
} nl_imu_array_data_t;

/* IMU device operations */
typedef struct
{
    int (*init)(void *user_data, uint8_t capabilities);
    int (*read_raw)(void *user_data, uint8_t capabilities, nl_imu_data_t *raw);
} nl_imu_ops_t;

/* 
    those param is determinted during the factory calibration process
*/
typedef struct
{
    nl_t            gmis_data[9];       /* acc mis 3x3 */
    nl_t            wmis_data[9];       /* gyr mis 3x3 */
    nl_t            mmis_data[9];       /* mag mis 3x3 */
    nl_t            urfr_data[9];       /* URFR mis, user reference rotation */
    nl_t            gb[3];              /* acc bias */
    nl_t            wb[3];              /* gyr bias */
    nl_t            mb[3];              /* mag bias(hard iron) */
    nl_t            gb_tc1[3];          /* 1st order temperature compensation for acc bias */
    nl_t            gb_tc2[3];          /* 2nd order temperature compensation for acc bias */
    nl_t            wb_tc1[3];          /* 1st order temperature compensation for gyr bias */
    nl_t            wb_tc2[3];          /* 2nd order temperature compensation for gyr bias */
    nl_t            ws_tc1[3];          /* 1st order temperature compensation for gyr scale */
    nl_t            ws_tc2[3];          /* 2nd order temperature compensation for gyr scale */
    nl_t            gb_tc1_low[3];      /* 1st order temperature compensation for acc bias (low temp) */
    nl_t            gb_tc2_low[3];      /* 2nd order temperature compensation for acc bias (low temp) */
    nl_t            wb_tc1_low[3];      /* 1st order temperature compensation for gyr bias (low temp) */
    nl_t            wb_tc2_low[3];      /* 2nd order temperature compensation for gyr bias (low temp) */
    nl_t            wb_cal_temp;        /* calibration reference temperature */
    nl_t            gb_cal_temp;        /* calibration reference temperature */
}nl_imu_calib_t;

/* IMU device object */
typedef struct nl_imu_dev
{
    const char *name;
    nl_imu_rotation_t rotation;
    const nl_imu_ops_t *ops;
    void *user_data;
    
    uint8_t capabilities;
    uint8_t update_interval_ms;    // Update interval in milliseconds
    uint32_t last_update_ms;        // Last successful update timestamp
    
    bool init_ok;
    struct nl_imu_dev *next;
} nl_imu_dev_t;

typedef struct {
    nl_t last_tc_cache_update_temp;        // Last temperature used for compensation
    nl_t cached_gb_drift[3];     // Cached accelerometer bias compensation
    nl_t cached_wb_drift[3];     // Cached gyroscope bias compensation  
    nl_t cached_ws_drift[3];     // Cached gyroscope scale compensation
} nl_imu_temp_cache_t;

/* IMU manager structure */
typedef struct
{
    nl_imu_dev_t *dev_list;
    
    nl_imu_array_data_t buffer;
    
    /* calibratrion data */
    m_t             gmis;               /* acc mis-aligne mat */
    m_t             wmis;               /* gyr mis-aligne mat */
    m_t             mmis;               /* mag mis-aligne mat */
    m_t             urfr; 
    
    uint8_t acc_count;
    uint8_t gyr_xy_count; 
    uint8_t gyr_z_count; 
    uint8_t mag_count;    
    uint8_t temp_count;   
    uint8_t prs_count;
    
    nl_imu_temp_cache_t temp_cache;
    /*  Calibration parameters */
    nl_imu_calib_t calib;
    
    /* enable temperature compensation */
    uint8_t en_tc;
    
    /* change from ENU to NED(X' = Y, Y' = Z, Z' = -Z) */
    uint8_t en_enu2ned;
} nl_imu_mgr_t;


void nl_imu_mgr_init(nl_imu_mgr_t *mgr);
int nl_imu_add_device(nl_imu_mgr_t *mgr, nl_imu_dev_t *dev);
void nl_imu_read_array(nl_imu_mgr_t *mgr);
void nl_imu_reset_calib(nl_imu_mgr_t *mgr);

void nl_imu_set_enu2ned(nl_imu_mgr_t *mgr, uint8_t enable);
void nl_imu_set_tc(nl_imu_mgr_t *mgr, uint8_t enable);



int nl_imu_load_acc_mis(nl_imu_mgr_t *mgr, const nl_t *mis_data);
int nl_imu_load_acc_bias(nl_imu_mgr_t *mgr, const nl_t *bias);
int nl_imu_load_gyr_mis(nl_imu_mgr_t *mgr, const nl_t *mis_data);
int nl_imu_load_gyr_bias(nl_imu_mgr_t *mgr, const nl_t *bias);
int nl_imu_load_mag_mis(nl_imu_mgr_t *mgr, const nl_t *mis_data);
int nl_imu_set_urfr_idx(nl_imu_mgr_t *mgr, int idx);
int nl_imu_get_urfr_idx(nl_imu_mgr_t *mgr);
int nl_imu_load_urfr(nl_imu_mgr_t *mgr, const nl_t *urfr_data);
int nl_imu_load_mag_bias(nl_imu_mgr_t *mgr, const nl_t *bias);
int nl_imu_load_temp_calib(nl_imu_mgr_t *mgr, 
                              const nl_t *gb_tc1, const nl_t *gb_tc2,
                              const nl_t *wb_tc1, const nl_t *wb_tc2,
                              const nl_t *ws_tc1, const nl_t *ws_tc2,
                              const nl_t *gb_tc1_low, const nl_t *gb_tc2_low,
                              const nl_t *wb_tc1_low, const nl_t *wb_tc2_low,
                              nl_t gb_cal_temp, nl_t wb_cal_temp);
uint8_t nl_imu_get_tc_status(nl_imu_mgr_t *mgr);

/**
 * @brief Get the number of successfully initialized IMUs with 3-axis gyroscope
 * @param mgr IMU manager instance
 * @return Number of initialized IMUs with gyroscope
 */
uint8_t nl_imu_get_gyr_count(nl_imu_mgr_t *mgr);


#endif

