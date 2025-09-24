#ifndef _DECODE_NMEA_H
#define _DECODE_NMEA_H

#ifdef __cplusplus
extern "C"{
#endif

#include "nl.h"



typedef struct
{
    double        hour;
    double        min;
    double        sec;
    double        lat,lon,msl;  /* lat: in deg, lon: in deg, alt:m(msl) */
    uint8_t       nv;
    uint8_t       status;       /* see NMEA sol quatliy */
    float         hdop;
    float         undulation;
    float         diff_age;
    uint32_t      sta_id;
}nmea_gga_t;

typedef struct
{
    double       year;
    double       month;
    double       day;
    double       hour;
    double       min;
    double       sec;
    uint8_t      mode;
    double       lat,lon;
    float        sog; /* speed over ground, knots */
    float        cog; /* Course over ground, deg */
}nmea_rmc_t;


typedef struct
{
  int sat_id; /* satellite ID number */
  int el;     /* Elevation, degrees, */
  int az;     /* Azimuth, degrees */
  int snr;    /* SNR (C/No) 00-99 dB-Hz, null when not tracking */
}gsv_sat_t;

typedef struct
{
  int      n_sentences;       /* total number of sentences */
  int      idx_sentences;     /* sentence number */
  uint32_t     nv;                /*  total number of satellites in view */
  gsv_sat_t    sat[32];            /* sat information */
  int      sid;               /* signal id see: Table 21 - GNSS Identification Table - GSV*/
  int      sys;
}nmea_gsv_t;



typedef struct
{
  char mode;        /* M = Manual, forced to operate in 2D or 3D mode A = Automatic, allowed to automatically switch 2D/3D */
  int sol_mode; /* 1 = Fix not available, 2 = 2D, 3 = 3D */
  uint8_t sat[16];  /* valid sat array, see GSA doc */
  uint8_t nv;       /* # of valid sat in sat[] */
  float pdop;
  float hdop;
  float vdop;
  int sys;     /* 1:GP  2:GL  3:GA  4:BD(GB) 5:QZSS */
}nmea_gsa_t;

typedef struct
{
    double year;
    double month;
    double day;
    double hour;
    double min;
    double sec;
    double lon;
    double lat;
    double msl;
    double yaw;
    double pitch;
    double spped_yaw;
    double ground_speed;
    uint8_t solq;         /* align with GGA's indicator */
    uint8_t solq_heading; /* aligne with GGA's indicator */
    uint8_t nv_heading;
    uint8_t nv;
    double roll;
    double gyr_x;
    double gyr_y;
    double gyr_z;
    double ve;
    double vn;
    double vu;
    uint8_t ins_stat;   /* aligned with ins->ins_stat */
    uint8_t ant_stat;
}nmea_sxt_t;

typedef struct /* see VectorNAV VN100 UM: 7.2.3 Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rates */
{
    double yaw;
    double pitch;
    double roll;
    double mag_x;
    double mag_y;
    double mag_z;
    double acc_x;
    double acc_y;
    double acc_z;
    double gyr_x;
    double gyr_y;
    double gyr_z;
}nmea_ymr_t;

typedef struct
{
    double groud_yaw_true; // 实际航向，相对于真北（度）
    double groud_yaw_mag;   // 磁航向，相对于磁北（度）
    double groud_speed_knot;// 水平地面速度（节）
    double groud_speed_kmph;// 水平地面速度（公里/小时）
}nmea_vtg_t;



typedef struct
{
    int         nbyte;                  /* number of bytes in message buffer */ 
    int         len;
    uint8_t     buf[MAXRAWLEN];         /* message raw buffer */
    char type[3];                     /* which type of nmea sentense "GGA", "RMC", "GSV", "GSA" */
    nmea_gga_t gga;
    nmea_rmc_t rmc;
    nmea_gsa_t gsa;
    nmea_gsv_t gsv;
    nmea_sxt_t sxt;
    nmea_ymr_t ymr;
    nmea_vtg_t vtg;
}nmea_raw_t;

int input_nmea(nmea_raw_t *raw, uint8_t data);
void nmea2gnss_sol(gnss_sol_t *sol, nmea_raw_t *raw);
char *nmea_chksum(char *buf, uint8_t *sum);
uint32_t nmea_append_chksum(char *buf);

void nl_enc_nmea_set_tid(char *tid);
int nl_enc_nmea_gga(gnss_sol_t *sol, ins_t *ins, uint8_t *buf);
int nl_enc_nmea_rmc(gnss_sol_t *sol, ins_t *ins, uint8_t *buf);
int nl_enc_nmea_tra(ins_t *ins, gnss_sol_t *sol, uint8_t *buf);
int nl_enc_nmea_sxt(nl_gtime_t *gpst, ins_t *ins, nl_t *gyr, gnss_sol_t *sol, uint8_t *buf);
int nl_enc_nmea_vtg(ins_t *ins, gnss_sol_t *sol, uint8_t *buff);
int nl_enc_nmea_gsv(gnss_sol_t *sol, uint8_t *buff);
int nl_enc_nmea_gga_from_lonlat(nl_t ep[6], double lon, double lat, double msl, uint8_t *buf);


#ifdef __cplusplus
}
#endif

#endif
