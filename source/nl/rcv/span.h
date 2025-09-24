#ifndef _DECODE_SPAN_H
#define _DECODE_SPAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nl.h"


#define POS_TYPE_NONE                   (0)
#define POS_TYPE_FIXEDPOS               (1)
#define POS_TYPE_FIXEDHEIGHT            (2)
#define POS_TYPE_DOPPLER_VELOCITY       (8)
#define POS_TYPE_SINGLE                (16)
#define POS_TYPE_PSRDIFF               (17)
#define POS_TYPE_SBAS                  (18)
#define POS_TYPE_l1FLOAT               (32)
#define POS_TYPE_IONOFREE_FLOAT        (33)
#define POS_TYPE_NARROW_FLOAT          (34)
#define POS_TYPE_L1_INT                (48)
#define POS_TYPE_WIDE_INT              (49)
#define POS_TYPE_NARROW_INT            (50)
#define POS_TYPE_INS_SBAS              (52)
#define POS_TYPE_INS_PSRSP             (53)
#define POS_TYPE_INS_PSRDIFF           (54)
#define POS_TYPE_INS_RTKFLOAT          (55)
#define POS_TYPE_INS_RTKFIXED          (56)
#define POS_TYPE_INS_PPP_CONVERAGE     (68)
#define POS_TYPE_INS_PPP               (69)



#define TIME_QUALITY_UNKNOWN (20)
#define TIME_QUALITY_FINE (160)



#define OEM_ID_INSPVAX         (1465)
#define OEM_ID_RAWIMUX         (1461)
#define OEM_ID_BESTPOS         (42)
#define OEM_ID_ANTENNA         (51)
#define OEM_ID_BESTXYZ         (241)
#define OEM_ID_HEADING         (971)
#define OEM_ID_PVTSLN          (1021)
#define OEM_ID_VERSION         (37)
#define OEM_ID_GNSSRCV         (1590)
#define OEM_ID_CHNAV           (1594)
#define OEM_ID_AGRIC           (11276)
#define OEM_ID_SATSINFO        (2124)
#define OEM_ID_BESTSAT         (1041)

#ifdef QT_CORE_LIB
#pragma pack(push)
#pragma pack(1)
#endif

typedef struct __attribute__((__packed__))
{
    uint32_t preamble; /* 0x1C1244AA: AA 44 12 1C(header len(0x1C=28)) */
    uint16_t msg_id;
    uint8_t msg_type; /* 00 =二进制 01 = ASCII 10 = 简化ASCII */
    uint8_t rev1;
    uint16_t msg_len;
    uint16_t sequence;
    uint8_t idle_time;
    uint8_t time_quality; /* TIME_QUALITY_UNKNOWN or TIME_QUALITY_FINE */
    uint16_t gps_wn;
    uint32_t gps_tow_ms;
    uint32_t evt_bit;
    uint8_t rev3[4];
}
oem_hdr_t;



typedef struct __attribute__((__packed__))
{
    uint32_t preamble; /* 0x1C1244AA: AA 44 12 1C(header len(0x1C=28)) */
    uint16_t msg_id;
    uint16_t msg_len;
    uint8_t  time_ref;
    uint8_t  time_stat;
    uint16_t gps_wn;
    uint32_t gps_tow_ms;
    uint32_t rev;
    uint8_t  version;
    uint8_t  leap_sec;
    uint16_t delay_ms;
}
unicore_hdr_t;


typedef struct __attribute__((__packed__)) /* UM982 AGIRC msg */
{
    unicore_hdr_t  hdr;
    uint8_t         gnss_str[4];  /* 固定为 GNSS */
    uint8_t         len; /* 232 */
    uint8_t         year; /*2016年，为 16 */
    uint8_t         month;
    uint8_t         day;
    uint8_t         hour;
    uint8_t         minute;
    uint8_t         second;
    uint8_t         rtk_stat; /* 0：无效解  1：单点定位解  2：伪距差分  4：固定解  5：浮动解 */
    uint8_t         heading_stat; /* 0：无效解 4：固定解 5：浮动解 */
    uint8_t         nv_gps;
    uint8_t         nv_bd;
    uint8_t         nv_glo;
    float           bl_neu[3];        /* 到基站RTK的 基线 */
    float           bl_neu_std[3];    /* 到基站RTK的 基线 std */
    float           heading;          /* 0-360 */
    float           pitch;
    float           roll;
    float           speed;      /* spped 标量 */
    float           vel_neu[3];
    float           vel_neu_std[3];
    double          lla[3];
    double          XYZ[3];
    float           lla_std[3]; /* meaning pos_neu_std */
    float           XYZ_std[3];
    double          base_lla[3];
    double          sec_lla[3];
    uint32_t        tow;
    float           diff_age;
    float           speed_heading;
    float           undulation;
    float           rev[2];
    uint8_t         nv_gal;
    uint8_t         rev2[3];
    uint32_t        crc;
}
oem_agric_t;


typedef struct __attribute__((__packed__))
{
    uint8_t sat_id; /* satellite ID number */
    uint8_t el;     /* Elevation, degrees, */
    uint16_t az;     /* Azimuth, degrees */
    uint8_t snr[12+1];    /* SNR (C/No) 00-99 dB-Hz, null when not tracking */
}
satsinfo_data_t;


typedef struct __attribute__((__packed__)) /* UM982 AGIRC msg */
{
    unicore_hdr_t  hdr;
    uint32_t bestpos_type;
    float    bestpos_ght;
    double   bestpos_lat;
    double   bestpos_lon;
    float    bestpos_hgtstd;
    float    bestpos_latstd;
    float    bestpos_lonstd;
    float    bestpos_diffage;
    uint32_t psrpos_type;
    float    psrpos_ght;
    double   psrpos_lat;
    double   psrpos_lon;
    float    undulation;
    uint8_t  bestpos_svs;
    uint8_t  bestpos_solnsvs;
    uint8_t  psrpos_svs;
    uint8_t  psrpos_solnsvs;
    double   psrvel_north;
    double   psrvel_east;
    double   psrvel_ground;
    uint32_t heading_type;
    float    heading_length;
    float    heading_degree;
    float    heading_pitch;
    uint8_t  heading_trackedsvs;
    uint8_t  heading_solnsvs;
    uint8_t  heading_ggl1;
    uint8_t  heading_ggl1l2;
    float    gdop;
    float    pdop;
    float    hdop;
    float    htdop;
    float    tdop;
    float    cutoff;
    uint16_t PRN_No;
    uint16_t PRN_list[41];
    uint32_t    crc;
}
oem_pvtsln_t;



typedef struct
{
    int               nbyte;
    int               len;
    uint8_t           buf[MAXRAWLEN];
    oem_agric_t       agric;
    oem_pvtsln_t      pvtsln;
    uint32_t          type;
} oem_raw_t;



#ifdef QT_CORE_LIB
#pragma pack(pop)
#endif

uint32_t oem7_crc32(uint8_t *buf, uint32_t len);

void agricb2gnss_sol(gnss_sol_t *sol, oem_agric_t *agric);

int input_oem(oem_raw_t *raw, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif
