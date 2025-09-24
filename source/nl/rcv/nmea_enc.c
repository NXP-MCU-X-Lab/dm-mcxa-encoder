#include "nmea.h"

#define DEFAULT_NMEA_TID "GP"      /* NMEA talker ID for RMC and GGA sentences */
#define KNOT2M 0.514444444 /* m/sec --> knot */

static uint8_t nmea_tid[] = DEFAULT_NMEA_TID;

static const int  nmea2solq[] __attribute__((unused)) = {/* NMEA GPS quality indicator [1] */
    /* 0=Fix not available or invalidi */
    /* 1=GPS SPS Mode, fix valid */
    /* 2=Differential GPS, SPS Mode, fix valid */
    /* 3=GPS PPS Mode, fix valid */
    /* 4=Real Time Kinematic. System used in RTK mode with fixed integers */
    /* 5=Float RTK. Satellite system used in RTK mode, floating integers */
    /* 6=Estimated (dead reckoning) Mode */
    /* 7=Manual Input Mode */
    /* 8=Simulation Mode */
    SOLQ_NONE, SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP, SOLQ_FIX,
    SOLQ_FLOAT, SOLQ_DR, SOLQ_NONE, SOLQ_NONE, SOLQ_NONE};

static const int solq2ggasolq[] = {0, 4, 5, 2, 2, 1, 0, 7, 0};

void nl_enc_nmea_set_tid(char *tid)
{
    nmea_tid[0] = tid[0];
    nmea_tid[1] = tid[1];
}

/* see 和芯星通 7.2.5 SXT定位定向数据输出语句 */
int nl_enc_nmea_sxt(nl_gtime_t *gpst, ins_t *ins, nl_t *gyr, gnss_sol_t *sol, uint8_t *buf)
{
    nl_gtime_t time;
    nl_t ep[6];
    double lat, lon, msl;
    
    char *p = (char *)buf, *q, sum;

    time = nl_gpst2utc(*gpst);
    nl_time2epoch(time, ep);

    lat = ins->lla[0];
    lon = ins->lla[1];
    msl = ins->lla[2];

    /* SXT need 0-360, clockwise */
    nl_t yaw = yaw_ccw_to_cw_2pi(ins->att.yaw);
    
    p += sprintf(p, "$%sSXT,%04.0f%02.0f%02.0f%02.0f%02.0f%05.2f,"
                    "%.9f,%.9f,%.4f,"
                    "%.2f,%.2f,%.2f,%.3f,%.2f,"
                    "%d,%d,%d,%d,"
                    "%.3f,%.3f,%.3f,"
                    "%.3f,%.3f,%.3f,"
                    "%d,%d",
        nmea_tid, ep[0], ep[1], ep[2], ep[3], ep[4], ep[5],
        lon * R2D, lat * R2D, msl - sol->undulation,
        yaw*R2D, ins->att.pitch*R2D, ins->TrackAngle*R2D, vnorm(ins->v, 2), ins->att.roll*R2D,
        solq2ggasolq[sol->solq], solq2ggasolq[sol->solq_heading], sol->nv, sol->nv_heading,
        gyr[0]*R2D, gyr[1]*R2D, gyr[2]*R2D, 
        ins->v[0], ins->v[1], ins->v[2],
        ins->ins_stat, 0);

    for (q = (char *)buf + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return (int)(p - (char *)buf);
}

/* NMEA:GGA */
int nl_enc_nmea_gga(gnss_sol_t *sol, ins_t *ins, uint8_t *buf)
{
    nl_gtime_t time;
    nl_t undulation, ep[6], dop = 1.0;
    double pos[3], dms1[3], dms2[3];
    int solq = 0, refid = 0;
    char *p = (char *)buf, *q, sum;

    if (sol->solq <= SOLQ_NONE && (ins->ins_stat != INS_SOLUTION_GOOD))
    {
        p += sprintf(p, "$%sGGA,,,,,,,,,,,,,,", nmea_tid);
        for (q = (char *)buf + 1, sum = 0; *q; q++)
            sum ^= *q;
        p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
        return (int)(p - (char *)buf);
    }

    solq = solq2ggasolq[sol->solq];

    time = nl_gpst2utc(sol->gpst);
    time.time = time.time + 0.0;
    if (time.sec >= 0.995)
    {
        time.time++;
        time.sec = 0.0;
    }
    nl_time2epoch(time, ep);

    if((ins->ins_stat == INS_SOLUTION_GOOD) || (ins->ins_stat == INS_SOLUTION_FREE))
    {
      pos[0] = ins->lla[0];
      pos[1] = ins->lla[1];
      pos[2] = ins->lla[2];
    }
    else
    {
      pos[0] = sol->lla[0];
      pos[1] = sol->lla[1];
      pos[2] = sol->lla[2];
    }

    undulation = sol->undulation;
    dop = sol->hdop;
    nl_deg2dms(fabs(pos[0]) * R2D, dms1, 8);
    nl_deg2dms(fabs(pos[1]) * R2D, dms2, 8);
    p += sprintf(p, "$%sGGA,%02.0f%02.0f%05.2f,%02.0f%011.8f,%s,%03.0f%011.8f,%s,"
                    "%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%04d",
        nmea_tid, ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0,
        pos[0] >= 0 ? "N" : "S", dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W",
        solq, sol->nv, dop, pos[2] - undulation, undulation, sol->diff_age, refid);
    for (q = (char *)buf + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return (int)(p - (char *)buf);
}

/* use for gengerate ntrip request
ep: UTC time
lon: deg
lat: deg
*/
int nl_enc_nmea_gga_from_lonlat(nl_t *ep, double lon, double lat, double msl, uint8_t *buf)
{
    gnss_sol_t gnss_sol;
    memset(&gnss_sol, 0, sizeof(gnss_sol_t));

    gnss_sol.lla[0] = lat*D2R;
    gnss_sol.lla[1] = lon*D2R;
    gnss_sol.lla[2] = msl;
    gnss_sol.solq = SOLQ_SINGLE;
    gnss_sol.gpst = nl_utc2gpst(nl_epoch2time(ep));

    ins_t ins;
    memset(&ins, 0, sizeof(ins_t));
    ins.ins_stat = INS_INACTIVE;
    return nl_enc_nmea_gga(&gnss_sol, &ins, buf);
}

/* NMEA:RMC */
int nl_enc_nmea_rmc(gnss_sol_t *sol, ins_t *ins, uint8_t *buf)
{
    nl_gtime_t time;
    nl_t ep[6], enuv[3], vel_n, dir, amag = 0.0;
    double  pos[3], dms1[3], dms2[3];
    char *p = (char *)buf, *q, sum;
    const char *emag = "E", *mode = "A", *status = "V";

    if (sol->solq <= SOLQ_NONE && (ins->ins_stat != INS_SOLUTION_GOOD))
    {
        p += sprintf(p, "$%sRMC,,,,,,,,,,,,,", nmea_tid);
        for (q = (char *)buf + 1, sum = 0; *q; q++)
            sum ^= *q;
        p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
        return (int)(p - (char *)buf);
    }

    time = nl_gpst2utc(sol->gpst);
    if (time.sec >= 0.995)
    {
        time.time++;
        time.sec = 0.0;
    }
    nl_time2epoch(time, ep);
    if(ins->ins_stat == INS_SOLUTION_GOOD)
    {
      pos[0] = ins->lla[0];
      pos[1] = ins->lla[1];
    }
    else
    {
      pos[0] = sol->lla[0];
      pos[1] = sol->lla[1];
    }


    enuv[0] = sol->vel_enu[0];
    enuv[1] = sol->vel_enu[1];
    enuv[2] = sol->vel_enu[2];
    vel_n = v3norm(enuv);

    dir = atan2(enuv[0], enuv[1]) * R2D;
    if (dir < 0.0) dir += 360.0;
            
    if (sol->solq == SOLQ_DGPS || sol->solq == SOLQ_SBAS)
        mode = "D";
    else if (sol->solq == SOLQ_FIX)
        mode = "R";
    else if (sol->solq == SOLQ_FLOAT)
        mode = "F";
    else if (sol->solq == SOLQ_PPP)
        mode = "P";
    else if (sol->solq == SOLQ_DR)
        mode = "E";
    nl_deg2dms(fabs(pos[0]) * R2D, dms1, 7);
    nl_deg2dms(fabs(pos[1]) * R2D, dms2, 7);
    p += sprintf(p, "$%sRMC,%02.0f%02.0f%05.2f,A,%02.0f%011.8f,%s,%03.0f%011.8f,"
                    "%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s,%s",
        nmea_tid, ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0,
        pos[0] >= 0 ? "N" : "S", dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W",
        vel_n / KNOT2M, dir, ep[2], ep[1], (int)ep[0] % 100, amag, emag, mode, status);
    for (q = (char *)buf + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return (int)(p - (char *)buf);
}

/* NMEA:TRA */
int nl_enc_nmea_tra(ins_t *ins, gnss_sol_t *sol, uint8_t *buf)
{
    nl_gtime_t time;
    int solq = 0;
    nl_t ep[6];
    char *p = (char *)buf, *q, sum;

    time = nl_gpst2utc(sol->gpst);
    if (time.sec >= 0.995)
    {
        time.time++;
        time.sec = 0.0;
    }
    nl_time2epoch(time, ep);
    solq = solq2ggasolq[sol->solq];

    p += sprintf(p, "$%sTRA,%02.0f%02.0f%05.2f,%03.2f,%03.2f,%03.2f,%01d,%02d,%02.2f,%04d",
        nmea_tid, ep[3], ep[4], ep[5],
        yaw_ccw_to_cw_2pi(ins->att.yaw) * R2D, ins->att.pitch * R2D, ins->att.roll * R2D,
        solq, sol->nv, sol->diff_age, 0);

    for (q = (char *)buf + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return (int)(p - (char *)buf);
}

int nl_enc_nmea_vtg(ins_t *ins, gnss_sol_t *sol, uint8_t *buff)
{
    static nl_t dirp = 0.0;
    nl_t enuv[3], vel, dir;
    char *p = (char *)buff, *q, sum;
    const char *mode = "A";

    if (sol->solq <= SOLQ_NONE)
    {
        p += sprintf(p, "$%sVTG,,,,,,,,,,,,,", nmea_tid);
        for (q = (char *)buff + 1, sum = 0; *q; q++)
            sum ^= *q;
        p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
        return p - (char *)buff;
    }

    for(uint8_t i=0; i<3; i++)  enuv[i] =ins->v[i];
    vel = v3norm(enuv);

        if (vel >= 1.0)
        {
            dir = atan2(enuv[0], enuv[1]) * R2D;
            if (dir < 0.0)
                dir += 360.0;
            dirp = dir;
        }
        else
        {
            dir = dirp;
        }

    if (sol->solq == SOLQ_NONE )
        mode = "N";
    else
        mode = "A";

    p += sprintf(p, "$%sVTG,%4.2f,%s,%4.2f,%s,%4.2f,%s,%4.2f,%s,%s",
        nmea_tid, yaw_ccw_to_cw_2pi(ins->att.yaw) * R2D, "T", ins->att.yaw * R2D, "M", vel / KNOT2M, "N", vel * 3.6, "K", mode);

    for (q = (char *)buff + 1, sum = 0; *q; q++)
        sum ^= *q; /* check-sum */
    p += sprintf(p, "*%02X\r\n", sum);
    return p - (char *)buff;
}

/* XW */
double julianDate(int year, int month, int day, int hour, int minute, double second) {
    if (month <= 2) {
        year--;
        month += 12;
    }
    int A = year / 100;
    int B = A / 4;
    int C = 2 - A + B;
    double E = 365.25 * (year + 4716);
    double F = 30.6001 * (month + 1);
    double extra = (hour + (minute + second / 60.0) / 60.0) / 24.0;
    return C + day + E + F + extra - 1524.5;
}



///* calcalute nmea chksum */
//char *nmea_chksum(char *buf, uint8_t *sum)
//{
//    *sum = 0;
//    char *q;
//    for (q = (char *)buf + 1, *sum = 0; *q && *q != '*'; q++)
//        *sum ^= *q;
//    return q;
//}

///* append check sum in a nmea string */
//uint32_t nmea_append_chksum(char *buf)
//{
//    uint8_t sum = 0;
//    char *p = nmea_chksum(buf, &sum);
//    p += sprintf(p, "*%02X\r\n", sum);
//    return (int)(p - (char *)buf);
//}
