#ifndef __UBX_PROTOCOL_H__
#define __UBX_PROTOCOL_H__


#include "nl.h"



typedef struct __attribute__((__packed__))
{
    uint32_t        iTOW;          // ms       GPS time of week of the navigation epoch. See the description of iTOW for 
    uint16_t        year;          // y        Year UTC
    uint8_t         month;         // month    Month, range 1..12 UTC
    uint8_t         day;           // d        Day of month, range 1..31 UTC
    uint8_t         hour;          // h        Hour of day, range 0..23 UTC
    uint8_t         min;           // min      Minute of hour, range 0..59 UTC
    uint8_t         sec;           // s        Seconds of minute, range 0..60 UTC
    uint8_t         valid;         // -        Validity Flags (see graphic below)
    uint32_t        tAcc;          // ns       Time accuracy estimate UTC
    int32_t         nano;          // ns       Fraction of second, range -1e9..1e9 UTC
    uint8_t         fixType;       // -        GNSSfix Type, range 0..5
    int8_t          flags;         // -        Fix Status Flags (see graphic below)
    uint8_t         rev;           // -        Reserved
    uint8_t         numSV;         // -        Number of satellites used in Nav Solution
    int32_t         lon;           // deg      Longitude (1e-7)
    int32_t         lat;           // deg      Latitude (1e-7)
    int32_t         height;        // mm       Height above Ellipsoid
    int32_t         hMSL;          // mm       Height above mean sea level
    uint32_t        hAcc;          // mm       Horizontal Accuracy Estimate
    uint32_t        vAcc;          // mm       Vertical Accuracy Estimate
    int32_t         velN;          // mm/s     NED north velocity
    int32_t         velE;          // mm/s     NED east velocity
    int32_t         velD;          // mm/s     NED down velocity
    int32_t         gSpeed;        // mm/s     Ground Speed (2-D)
    int32_t         heading;       // deg      Heading of motion 2-D (1e-5)
    uint32_t        sAcc;          // mm/s     Speed Accuracy Estimate
    uint32_t        headingAcc;    // deg      Heading Accuracy Estimate (1e-5)
    uint16_t        pDOP;          // -        Position DOP (0.01)
    uint8_t         reserved2[6];  // -        Reserved
    int32_t         headVeh;
    uint8_t         reserved3[4];  // -        Reserved 
}ubx_pvt_t;

typedef struct
{
    int         nbyte;
    int         len;
    uint8_t     buf[MAXRAWLEN];
    ubx_pvt_t   pvt;
}ubx_raw_t;





int input_ubx(ubx_raw_t *raw, uint8_t ch);
void ubxpvt2gnss_sol(gnss_sol_t *sol, ubx_pvt_t *pvt);

#endif
