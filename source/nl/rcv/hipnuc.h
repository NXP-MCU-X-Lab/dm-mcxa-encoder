#ifndef __HIPNUC_H__
#define __HIPNUC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <nl.h>

#ifdef QT_CORE_LIB
#pragma pack(push)
#pragma pack(1)
#endif

#define CHSYNC1                 (0x5A)              /* CHAOHE message sync code 1 */
#define CHSYNC2                 (0xA5)              /* CHAOHE message sync code 2 */
#define CH_HDR_SIZE             (0x06)          /* CHAOHE protocol header size */
#define HIPNUC_MAX_RAW_SIZE     (256+CH_HDR_SIZE)


typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag, if tag = 0x00, means that this packet is null */
    uint16_t        main_status;
    int8_t          temp;           /* Temperature */
    float           air_pressure;   /* Pressure */
    uint32_t        system_time;    /* Timestamp */
    float           acc[3];         /* Accelerometer data (x, y, z) */
    float           gyr[3];         /* Gyroscope data (x, y, z) */
    float           mag[3];         /* Magnetometer data (x, y, z) */
    float           roll;           /* Roll angle */
    float           pitch;          /* Pitch angle */
    float           yaw;            /* Yaw angle */
    float           quat[4];        /* Quaternion (w, x, y, z) */
}
hi91_t;

/**
 * Packet 0x92: IMU data (integer type)
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag */
    uint16_t        main_status;    /* Status information */
    int8_t          temperature;    /* Temperature */
    uint16_t        rev;            /* reserved */
    uint32_t        system_time;    /* Timestamp */ 
    int16_t         gyr_b[3];       /* Gyroscope data (raw) */
    int16_t         acc_b[3];       /* Accelerometer data (raw) */
    int16_t         mag_b[3];       /* Magnetometer data (raw) */
    int32_t         roll;           /* Roll angle (raw) */
    int32_t         pitch;          /* Pitch angle (raw) */
    int32_t         yaw;            /* Yaw angle (raw) */
    int16_t         quat[4];        /* Quaternion (raw) */
}
hi92_t;

/**
 * Packet 0x81: INS data, including lat, lon, eul, quat, raw IMU data
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag */
    uint16_t        main_status;    /* Status information */
    uint8_t         ins_status;     /* INS status */
    uint16_t        gpst_wn;        /* GPS time: week number */
    uint32_t        gpst_tow;       /* GPS time: time of week */
    uint16_t        rev;            /* Synchronization time */
    int16_t         gyr_b[3];       /* Gyroscope data (raw) */
    int16_t         acc_b[3];       /* Accelerometer data (raw) */
    int16_t         mag_b[3];       /* Magnetometer data (raw) */
    int16_t         air_pressure;   /* Air pressure */
    int16_t         od_vel;         /* odometer speed, (m/s) */
    int8_t          temperature;    /* Temperature */
    uint8_t         utc_year;       /* UTC year */
    uint8_t         utc_month;      /* UTC month */
    uint8_t         utc_day;        /* UTC day */
    uint8_t         utc_hour;       /* UTC hour */
    uint8_t         utc_min;        /* UTC minute */
    uint16_t        utc_msec;       /* UTC milliseconds */
    int16_t         roll;           /* Roll angle */
    int16_t         pitch;          /* Pitch angle */
    uint16_t        yaw;            /* Yaw angle */
    int16_t         quat[4];        /* Quaternion */
    int32_t         ins_lon;        /* INS longitude */
    int32_t         ins_lat;        /* INS latitude */
    int32_t         ins_msl;        /* INS mean sea level altitude */
    uint8_t         pdop;           /* Position dilution of precision */
    uint8_t         hdop;           /* Horizontal dilution of precision */
    uint8_t         solq_pos;       /* Solution quality for position */
    uint8_t         nv_pos;         /* Number of satellites used for position */
    uint8_t         solq_heading;   /* Solution quality for heading */
    uint8_t         nv_heading;     /* Number of satellites used for heading */
    uint8_t         diff_age;       /* Differential age */
    int16_t         undulation;     /* Undulation */
    uint8_t         rev2;           /* Reserved field */
    int16_t         vel_enu[3];     /* Velocity in ENU frame */
    int16_t         acc_enu[3];     /* Acceleration in ENU frame */
    int16_t         rev3;          /* Heave motion, scale: 0.01 m */
    int16_t         rev4;          /* Surge motion, scale: 0.01 m */
    int16_t         rev5;           /* Sway motion, scale: 0.01 m */
    int16_t         rev6;   /* Heave period, scale: 0.001 s */
    int32_t         rev7;           /*  */
    uint8_t         reserved2[2];   /* Reserved field */
}
hi81_t;

/* debug frame for IMU */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;
    uint16_t        status;
    uint8_t         ins_status;
    uint16_t        gpst_wn;
    uint32_t        gpst_tow;
    uint16_t        rev;
    int16_t         gyr_b[3];
    int16_t         acc_b[3];
    int16_t         mag_b[3];
    int16_t         air_pressure;
    int16_t         od_vel;
    int8_t          temperature;
    int16_t         roll;
    int16_t         pitch;
    uint16_t        yaw;
    int16_t         quat[4];
    int32_t         ins_lon;
    int32_t         ins_lat;
    int32_t         ins_msl;
    uint8_t         pdop;
    uint8_t         hdop;
    uint8_t         solq;
    uint8_t         nv_pos;
    uint8_t         solq_heading;
    uint8_t         nv_heading;
    uint8_t         diff_age;
    int16_t         undulation;
    uint8_t         ant_status;
    int16_t         vel_enu[3];
    int16_t         acc_enu[3];
    int32_t         gnss_lon;
    int32_t         gnss_lat;
    int32_t         gnss_msl;
    int16_t         gnss_vel_enu[3];
    uint16_t        gnss_pos_std_n;
    uint16_t        gnss_vel_std_n;

    int16_t         kf_att[3];
    int16_t         kf_wb[3];
    int16_t         kf_gb[3];
    int16_t         kf_vel[3];
    int16_t         kf_pos[3];

    uint16_t        kf_p_att[3];
    uint16_t        kf_p_wb[3];
    uint16_t        kf_p_gb[3];
    uint16_t        kf_p_vel[3];
    uint16_t        kf_p_pos[3];
    int16_t         gnss_dual_base_len;
    int16_t         gnss_dual_pitch_deg;
    int16_t         gnss_dual_heading_deg;
    int16_t         roll_test; /* pure intrgation */
    int16_t         pitch_test;
    int16_t         yaw_test;
}
hi43_t;

/**
 * Packet 0x85:  ÖÐ¿ÆÔº
 */
typedef struct __attribute__((__packed__))
{
    uint8_t         tag;            /* Data packet tag */
    uint16_t        status;         /* Status information */
    uint8_t         ins_status;     /* INS status */
    uint16_t        gpst_wn;        /* GPS time: week number */
    uint32_t        gpst_tow;       /* GPS time: time of week */
    float           gyr_b[3];       /* Gyroscope data (raw) */
    float           acc_b[3];       /* Accelerometer data (raw) */
    int16_t         od_vel;         /* odometer speed, (m/s) */
    uint8_t         utc_year;       /* UTC year */
    uint8_t         utc_month;      /* UTC month */
    uint8_t         utc_day;        /* UTC day */
    uint8_t         utc_hour;       /* UTC hour */
    uint8_t         utc_min;        /* UTC minute */
    uint16_t        utc_msec;       /* UTC milliseconds */
    double          roll;
    double          pitch;
    double          yaw;
    double          ins_lon;        /* INS longitude */
    double          ins_lat;        /* INS latitude */
    double          ins_msl;        /* INS mean sea level altitude */
    float           vel_enu[3];     /* Velocity in ENU frame */
    float           acc_enu[3];     /* Acceleration in ENU frame */
    
    double          gnss_lon;        /* INS longitude */
    double          gnss_lat;        /* INS latitude */
    double          gnss_msl;        /* INS mean sea level altitude */
    float           gnss_heading;        /* INS mean sea level altitude */
    float           gnss_enu[3];
    
    uint8_t         pdop;           /* Position dilution of precision */
    uint8_t         hdop;           /* Horizontal dilution of precision */
    uint8_t         solq_pos;       /* Solution quality for position */
    uint8_t         nv_pos;         /* Number of satellites used for position */
    uint8_t         solq_heading;   /* Solution quality for heading */
    uint8_t         nv_heading;     /* Number of satellites used for heading */
    uint8_t         diff_age;       /* Differential age */
    float           undulation;     /* Undulation */
    uint8_t         ant_status;     /* Reserved field */
    uint8_t         reserved2[8];   /* Reserved field */
}
hi85_t;

typedef struct
{
    int nbyte;                          /* number of bytes in message buffer */
    int len;                            /* message length (bytes) */
    uint8_t buf[HIPNUC_MAX_RAW_SIZE];   /* message raw buffer */
    hi91_t hi91;
    hi92_t hi92;
    hi81_t hi81;
    hi43_t hi43;
    hi85_t hi85;
} hipnuc_raw_t;

#ifdef QT_CORE_LIB
#pragma pack(pop)
#endif

int hipnuc_input(hipnuc_raw_t *raw, uint8_t data);
int hipnuc_dump_packet(hipnuc_raw_t *raw, char *buf, size_t buf_size);
int bin_hi91data(uint8_t *buf, uint16_t main_status, uint32_t sys_time, nl_t *acc, nl_t *gyr, nl_t *quat, nl_t *mag, nl_t pitch, nl_t roll, nl_t yaw, nl_t air_pressure, nl_t temperature);
int bin_hi92data(uint8_t *buf, uint16_t main_status, uint32_t sys_time, nl_t *acc, nl_t *gyr, nl_t *quat, nl_t *mag, nl_t pitch, nl_t roll, nl_t yaw, nl_t air_pressure, nl_t temperature);
int bin_hi43data(uint8_t *buf, uint32_t evt, nl_gtime_t gpst, gnss_sol_t *sol, lc_eskf_t *lc, ins_t *ins, nl_t *acc, nl_t *gyr, nl_t *mag, nl_t air_pressure, nl_t temperature, nl_t od_vel);
int bin_hi81data(uint8_t *buf, uint16_t main_status, nl_gtime_t gpst, gnss_sol_t *sol, ins_t *ins, nl_t *acc, nl_t *gyr, nl_t *mag, nl_t air_pressure, nl_t od_vel, nl_t temperature);
int ascii_hi93data(uint8_t *buf, uint16_t status, uint32_t sys_time, nl_t *acc, nl_t *gyr, nl_t *quat, nl_t *mag, nl_t pitch, nl_t roll, nl_t yaw, nl_t *displacement_ap, nl_t *frq, nl_t temperature);
int bin_hi85data(uint8_t *buf, uint32_t evt, nl_gtime_t gpst, gnss_sol_t *sol, ins_t *ins, nl_t *acc, nl_t *gyr, nl_t *mag, nl_t air_pressure, nl_t od_vel, nl_t heave, nl_t surge, nl_t sway, nl_t heave_period, nl_t temperature);

#ifdef __cplusplus
}
#endif


#endif


