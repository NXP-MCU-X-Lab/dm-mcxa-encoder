#ifndef _DECODE_AIROHA_H
#define _DECODE_AIROHA_H

#include "nl.h"


#define AIROHA_MSG_ID_RAW_MES_SV_INFO   (4003)
#define AIROHA_MSG_ID_RAW_MES_PVT       (4005)
#define AIROHA_MSG_ID_BIN_PVT           (2007)
#define AIROHA_MSG_ID_BIN_PVT_ADD       (2008)
#define AIROHA_MSG_ID_PBIN_TIME         (2014)


typedef struct __attribute__((__packed__))
{
    int32_t lat;            /* deg, scale:1E-7 */
    int32_t lon;            /* deg, scale:1E-7 */
    int32_t h_msl;          /* m, 1E-3 */
    int32_t msl_corr;       /* m, 1E-3, The difference between ellipsoid height and mean sea level. Equals (h_ref – h_msl) */
    uint32_t g_speed;       /* m/s, 1E-3 */
    uint32_t course;        /* degrees, 1E-5 */
    uint16_t hdop;          /* 1E-2, Horizontal dilution of precision */
    uint16_t vdop;          /* 1E-2, Vertical dilution of precision */
    uint16_t pdop;          /* 1E-2, Position dilution of precision*/
    uint16_t year;          /*  Year (UTC) */
    uint8_t month;         /* Day of month, 1-31 (UTC) */
    uint8_t day;           /* Day of month, 1-31 (UTC) */
    uint8_t hour;          /* Hour of day, 0-23 (UTC) */
    uint8_t min;           /* Minute of hour, 0-59 (UTC) */
    uint16_t msec;             /* Millisecond of second, 0-999 (UTC) */
    uint8_t  sec;              /* Second of hour, 0-59 (UTC) */
    uint8_t used_sv_num;       /* Number of satellites in use */
    uint16_t dgps_station_id;  /* DGPS station id */
    uint16_t dgps_age;         /* Age of Differential GPS data */
    uint8_t fix_quality;       /* GNSS Quality indicator 0: No fix  1: Standard positioning service (SPS)  2: Differential GNSS solution (DGPS)  4: RTK fixed  5: RTK float */
    uint8_t fix_mode;          /* GNSS fix type: 0: No fix 1: GNSS 2D fix 2: GNSS 3D fix */
    uint16_t reserved;
}airoha_pvt_t;


typedef struct __attribute__((__packed__))
{
    uint32_t pos_acc_n;       /* 1E-3, Position accuracy in north */
    uint32_t pos_acc_e;       /* 1E-3,Position accuracy in east */
    uint32_t pos_acc_d;       /* 1E-3,Position accuracy in vertical */
    int32_t  vel_n;           /* 1E-3, m/s, Velocity in north */
    int32_t  vel_e;           /* 1E-3, m/s, Velocity in east */
    int32_t  vel_d;           /* 1E-3, m/s, Velocity in down */
    uint32_t vel_acc_n;       /* 1E-3, m/s Velocity accuracy in north */
    uint32_t vel_acc_e;       /* 1E-3, m/s Velocity accuracy in east */
    uint32_t vel_acc_d;       /* 1E-3, m/s Velocity accuracy in vertical */
}airoha_pvt_add_t;


typedef struct __attribute__((__packed__))
{
    uint32_t tow;             /* 1E-3, s, GPS time of week */
    uint16_t wn;              /* GPS week number */
    uint16_t tdop;            /* 1E-2, Time dilution of precision */
    uint16_t leap_second;     /* s, Leap second between UTC */
    uint16_t reserved;
    int32_t  clock_bias;      /* 1E-8, s,Receiver clock bias */
    int32_t  clock_drift;      /* 1E-12,s, Receiver clock drift */
}airoha_time_t;

typedef struct
{
    int nbyte; /* number of bytes in message buffer */
    int len;
    int type;
    airoha_pvt_t        pvt;
    airoha_pvt_add_t    pvt_add;
    airoha_time_t       time;
    uint8_t buf[MAXRAWLEN]; /* message raw buffer */
} airoha_raw_t;

int airoha_input(airoha_raw_t *raw, uint8_t data);

#endif