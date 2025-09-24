#include "nl.h"



static const nl_t gpst0[] __attribute__((unused)) ={1980,1, 6,0,0,0}; /* gps time reference */
static const nl_t gst0 [] __attribute__((unused)) ={1999,8,22,0,0,0}; /* galileo system time reference */
static const nl_t bdt0 [] __attribute__((unused)) ={2006,1, 1,0,0,0}; /* beidou time reference */
static const nl_t leaps[MAXLEAPS+1][3]={ /* leap seconds (y,m,d,h,m,s,utc-gpst) */
    {2017,1,-18},
    {2015,7,-17},
    {2012,7,-16},
    {2009,1,-15},
    {2006,1,-14},
    {1999,1,-13},
    {1997,7,-12},
    {1996,1,-11},
    {1994,7,-10},
    {1993,7, -9},
    {1992,7, -8},
    {1991,1, -7},
    {1990,1, -6},
    {1988,1, -5},
    {1985,7, -4},
    {1983,7, -3},
    {1982,7, -2},
    {1981,7, -1},
    {0}
};

void lla2enu(ins_t *ins, double *lla, double *enu)
{
    enu[2] = lla[2] - ins->lla0[2];
    enu[1] = (lla[0] - ins->lla0[0]) * ins->Rns;
    enu[0] = (lla[1] - ins->lla0[1]) * ins->Rew;
}

void enu2lla(ins_t *ins, double *enu, double *lla)
{
    lla[2] = enu[2] + ins->lla0[2];
    lla[0] = enu[1] / ins->Rns + ins->lla0[0];
    lla[1] = enu[0] / ins->Rew + ins->lla0[1];
}

double lla2err(double *lla, double *lla0)
{
    double Earth_Re = 6378137;
    double Earth_e = 0.00335281066474748;
    double h0 = lla0[2];
    double lat0 = lla0[0];

    double Rm = Earth_Re * (1 - 2*Earth_e + 3*Earth_e*sin(lat0)*sin(lat0));
    double Rn = Earth_Re * (1 + Earth_e*sin(lat0)*sin(lat0));
    double Rmh = Rm + h0;
    double Rnh = Rn + h0;

    double err_n = (lla[0] - lla0[0]) * (Rmh);
    double err_e = (lla[1] - lla0[1]) * (Rnh) * cos(lat0);
    double pos_err = sqrt(err_e*err_e + err_n*err_n);

    return pos_err;
}

nl_gtime_t nl_epoch2time(const nl_t *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    nl_gtime_t time={0};
    int days,sec,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];
    
    if (year<1970||2099<year||mon<1||12<mon) return time;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    sec=(int)floor(ep[5]);
    time.time=(time_t)days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    time.sec=ep[5]-sec;
    return time;
}

void nl_deg2dms(double deg, double *dms, int ndec)
{
    double sign=deg<0.0?-1.0:1.0,a=fabs(deg);
    double unit=pow(0.1,ndec);
    dms[0]=floor(a); a=(a-dms[0])*60.0;
    dms[1]=floor(a); a=(a-dms[1])*60.0;
    dms[2]=floor(a/unit+0.5)*unit;
    if (dms[2]>=60.0) {
        dms[2]=0.0;
        dms[1]+=1.0;
        if (dms[1]>=60.0) {
            dms[1]=0.0;
            dms[0]+=1.0;
        }
    }
    dms[0]*=sign;
}

void nl_time2epoch(nl_gtime_t t, nl_t *ep)
{
    const int mday[]={ /* # of days in a month */
        31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
        31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
    };
    int days,sec,mon,day;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(int)(t.time/86400);
    sec=(int)(t.time-(time_t)days*86400);
    for (day=days%1461,mon=0;mon<48;mon++) {
        if (day>=mday[mon]) day-=mday[mon]; else break;
    }
    ep[0]=1970+days/1461*4+mon/12; ep[1]=mon%12+1; ep[2]=day+1;
    ep[3]=sec/3600; ep[4]=sec%3600/60; ep[5]=sec%60+t.sec;
}

uint32_t nl_getbitu(const uint8_t *buf, int pos, int len)
{
    uint32_t bits = 0;
    int i;
    for (i=pos;i<pos+len;i++) bits=(bits<<1)+((buf[i/8]>>(7-i%8))&1u);
    return bits;
}

nl_gtime_t nl_gpst2time(int week, double sec)
{
    nl_gtime_t t = nl_epoch2time(gpst0);
    
    if (sec<-1E9||1E9<sec) sec=0.0;
    t.time+=(time_t)86400*7*week+(int)sec;
    t.sec=sec-(int)sec;
    return t;
}

double nl_time2gpst(nl_gtime_t t, int *week)
{
    nl_gtime_t t0=nl_epoch2time(gpst0);
    time_t sec=t.time-t0.time;
    int w=(int)(sec/(86400*7));
    
    if (week) *week=w;
    return (double)(sec-(double)w*86400*7)+t.sec;
}

nl_gtime_t nl_utc2gpst(nl_gtime_t t)
{
    int i;
    
    for (i=0;leaps[i][0]>0;i++) 
    {
        if (nl_timediff(t,nl_epoch2time(leaps[i]))>=0.0) return nl_timeadd(t,-leaps[i][2]);
    }
    return t;
}

double nl_timediff(nl_gtime_t t1, nl_gtime_t t2)
{
    return t1.time - t2.time + t1.sec-t2.sec;
}

nl_gtime_t nl_timeadd(nl_gtime_t t, double sec)
{
    double tt;
    
    t.sec+=sec; tt=floor(t.sec); t.time+=(int)tt; t.sec-=tt;
    return t;
}


nl_gtime_t nl_gpst2utc(nl_gtime_t t)
{
    nl_gtime_t tu;
    int i;
    
    for (i=0;leaps[i][0]>0;i++) {
        tu=nl_timeadd(t,leaps[i][2]);
        if (nl_timediff(tu, nl_epoch2time(leaps[i]))>=0.0) return tu;
    }
    return t;
}

///* satellite system+prn/slot number to satellite number ------------------------
//* convert satellite system+prn/slot number to satellite number
//* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
//*          int    prn       I   satellite prn/slot number
//* return : satellite number (0:error)
//*-----------------------------------------------------------------------------*/
//int nl_satno(int sys, int prn)
//{
//    if (prn<=0) return 0;
//    switch (sys) {
//        case SYS_GPS:
//            if (prn<MINPRNGPS||MAXPRNGPS<prn) return 0;
//            return prn-MINPRNGPS+1;
//        case SYS_GLO:
//            if (prn<MINPRNGLO||MAXPRNGLO<prn) return 0;
//            return NSATGPS+prn-MINPRNGLO+1;
//        case SYS_GAL:
//            if (prn<MINPRNGAL||MAXPRNGAL<prn) return 0;
//            return NSATGPS+NSATGLO+prn-MINPRNGAL+1;
//        case SYS_QZS:
//            if (prn<MINPRNQZS||MAXPRNQZS<prn) return 0;
//            return NSATGPS+NSATGLO+NSATGAL+prn-MINPRNQZS+1;
//        case SYS_CMP:
//            if (prn<MINPRNCMP||MAXPRNCMP<prn) return 0;
//            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+prn-MINPRNCMP+1;
//        case SYS_LEO:
//            if (prn<MINPRNLEO||MAXPRNLEO<prn) return 0;
//            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+
//                   prn-MINPRNLEO+1;
//        case SYS_SBS:
//            if (prn<MINPRNSBS||MAXPRNSBS<prn) return 0;
//            return NSATGPS+NSATGLO+NSATGAL+NSATQZS+NSATCMP+NSATIRN+NSATLEO+
//                   prn-MINPRNSBS+1;
//    }
//    return 0;
//}


//int nl_satsys(int sat, int *prn)
//{
//    int sys=SYS_NONE;
//    if (sat<=0||MAXSAT<sat) sat=0;
//    else if (sat<=NSATGPS) {
//        sys=SYS_GPS; sat+=MINPRNGPS-1;
//    }
//    else if ((sat-=NSATGPS)<=NSATGLO) {
//        sys=SYS_GLO; sat+=MINPRNGLO-1;
//    }
//    else if ((sat-=NSATGLO)<=NSATGAL) {
//        sys=SYS_GAL; sat+=MINPRNGAL-1;
//    }
//    else if ((sat-=NSATGAL)<=NSATQZS) {
//        sys=SYS_QZS; sat+=MINPRNQZS-1; 
//    }
//    else if ((sat-=NSATQZS)<=NSATCMP) {
//        sys=SYS_CMP; sat+=MINPRNCMP-1; 
//    }
//    else if ((sat-=NSATCMP)<=NSATIRN) {
//        sys=SYS_IRN; sat+=MINPRNIRN-1; 
//    }
//    else if ((sat-=NSATIRN)<=NSATLEO) {
//        sys=SYS_LEO; sat+=MINPRNLEO-1; 
//    }
//    else if ((sat-=NSATLEO)<=NSATSBS) {
//        sys=SYS_SBS; sat+=MINPRNSBS-1; 
//    }
//    else sat=0;
//    if (prn) *prn=sat;
//    return sys;
//}

/* satellite id to satellite number --------------------------------------------
* convert satellite id to satellite number
* args   : char   *id       I   satellite id (nn,Gnn,Rnn,Enn,Jnn,Cnn,Inn or Snn)
* return : satellite number (0: error)
* notes  : 120-142 and 193-199 are also recognized as sbas and qzss
*-----------------------------------------------------------------------------*/
//int nl_satid2no(const char *id)
//{
//    int sys,prn;
//    char code;
//    
//    if (sscanf(id,"%d",&prn)==1) {
//        if      (MINPRNGPS<=prn&&prn<=MAXPRNGPS) sys=SYS_GPS;
//        else if (MINPRNSBS<=prn&&prn<=MAXPRNSBS) sys=SYS_SBS;
//        else if (MINPRNQZS<=prn&&prn<=MAXPRNQZS) sys=SYS_QZS;
//        else return 0;
//        return nl_satno(sys,prn);
//    }
//    if (sscanf(id,"%c%d",&code,&prn)<2) return 0;
//    
//    switch (code) {
//        case 'G': sys=SYS_GPS; prn+=MINPRNGPS-1; break;
//        case 'R': sys=SYS_GLO; prn+=MINPRNGLO-1; break;
//        case 'E': sys=SYS_GAL; prn+=MINPRNGAL-1; break;
//        case 'J': sys=SYS_QZS; prn+=MINPRNQZS-1; break;
//        case 'C': sys=SYS_CMP; prn+=MINPRNCMP-1; break;
//        case 'L': sys=SYS_LEO; prn+=MINPRNLEO-1; break;
//        case 'S': sys=SYS_SBS; prn+=100; break;
//        default: return 0;
//    }
//    return nl_satno(sys,prn);
//}

/* satellite number to satellite id --------------------------------------------
* convert satellite number to satellite id
* args   : int    sat       I   satellite number
*          char   *id       O   satellite id (Gnn,Rnn,Enn,Jnn,Cnn,Inn or nnn)
* return : none
*-----------------------------------------------------------------------------*/
//void nl_satno2id(int sat, char *id)
//{
//    int prn;
//    switch (nl_satsys(sat,&prn)) {
//        case SYS_GPS: sprintf(id,"G%02d",prn-MINPRNGPS+1); return;
//        case SYS_GLO: sprintf(id,"R%02d",prn-MINPRNGLO+1); return;
//        case SYS_GAL: sprintf(id,"E%02d",prn-MINPRNGAL+1); return;
//        case SYS_QZS: sprintf(id,"J%02d",prn-MINPRNQZS+1); return;
//        case SYS_CMP: sprintf(id,"C%02d",prn-MINPRNCMP+1); return;
//        case SYS_LEO: sprintf(id,"L%02d",prn-MINPRNLEO+1); return;
//        case SYS_SBS: sprintf(id,"%03d" ,prn); return;
//    }
//    strcpy(id,"");
//}

/* 
    calculate baseline heading, pitch, length, vector
*/
//int baseline_hplv(double *rb, double *rr, double *enu, double *heading, double *pitch, double *length, double *normlz_enu)
//{
//    int i;
//    double baseline_ecef[3], pos[3];
//    for (i = 0; i < 3; i++)
//    baseline_ecef[i] = rr[i] - rb[i]; /* rb(base): antB/base/rtklib:instance1, rr(user): user/antA/rtklib:instance0 */
//    ecef2pos(rb, pos);
//    ecef2enu(pos, baseline_ecef, enu);

//    nl_vec2hpl(enu, heading, pitch, length);

//    normlz_enu[0] = enu[0] / *length;
//    normlz_enu[1] = enu[1] / *length;
//    normlz_enu[2] = enu[2] / *length;
//}

/**
 * @brief Convert heading, pitch and length to ENU vector
 * @details Converts spherical coordinates (heading, pitch, length) to cartesian ENU coordinates
 * 
 * @param[in] heading Heading angle in radians, measured from North(y) to East(x), range [-p, p]
 *                    0 = North, p/2 = East, p = South, -p/2 = West
 * @param[in] pitch   Pitch angle in radians, measured from horizontal plane, range [-p/2, p/2]
 *                    positive = up, negative = down
 * @param[in] len     Vector length in meters
 * @param[out] enu    ENU vector output [3]
 *                    enu[0]: East component
 *                    enu[1]: North component
 *                    enu[2]: Up component
 * 
 * @note The conversion follows these equations:
 *       E = len * sin(heading) * cos(pitch)
 *       N = len * cos(heading) * cos(pitch)
 *       U = len * sin(pitch)
 */
void nl_hpl2vec(nl_t heading, nl_t pitch, nl_t len, nl_t *enu)
{
    if (!enu) return;
    
    /* Clamp pitch to valid range to prevent numerical issues */
    pitch = fmax(fmin(pitch, M_PI_2), -M_PI_2);
    
    /* Calculate ENU components */
    nl_t cos_pitch = cos(pitch);
    enu[0] = sin(heading) * cos_pitch * len;  /* East */
    enu[1] = cos(heading) * cos_pitch * len;  /* North */
    enu[2] = sin(pitch) * len;                /* Up */
}

/**
 * @brief Convert ENU vector to heading, pitch and length
 * @details Converts cartesian ENU coordinates to spherical coordinates (heading, pitch, length)
 * 
 * @param[in] enu     ENU vector input [3]
 *                    enu[0]: East component
 *                    enu[1]: North component
 *                    enu[2]: Up component
 * @param[out] heading Heading angle in radians, range [-p, p]
 *                     0 = North, p/2 = East, p = South, -p/2 = West
 * @param[out] pitch   Pitch angle in radians, range [-p/2, p/2]
 *                     positive = up, negative = down
 * @param[out] len     Vector length in meters
 * 
 * @note The conversion follows these equations:
 *       heading = atan2(E, N)
 *       pitch = asin(U/length)
 *       length = sqrt(E² + N² + U²)
 * 
 * @return void, but if input vector length is near zero, heading and pitch are set to 0
 */
void nl_vec2hpl(nl_t *enu, nl_t *heading, nl_t *pitch, nl_t *len)
{
    const nl_t MIN_LENGTH = 1e-8;  /* Minimum valid vector length */
    
    /* Parameter validation */
    if (!enu || !heading || !pitch || !len) return;
    
    /* Calculate vector length */
    *len = v3norm(enu);
    
    /* Handle near-zero length case */
    if (*len < MIN_LENGTH) {
        *heading = 0.0;
        *pitch = 0.0;
        *len = 0.0;
        return;
    }
    
    /* Calculate heading and pitch */
    *heading = atan2(enu[0], enu[1]);     /* Range: [-p, p] */
    *pitch = asin(enu[2] / *len);         /* Range: [-p/2, p/2] */
}

/**
 * @brief Convert angle from CCW [-p, p] to CW [0, 2p]
 * @param[in] angle Angle in radians, CCW system [-p, p]
 * @return Angle in radians, CW system [0, 2p]
 * 
 * Conversion steps:
 * 1. Normalize input to CCW [-p, p]
 * 2. Convert CCW to CW by negating (CCW positive -> CW negative)
 * 3. Map to [0, 2p] range
 */
nl_t yaw_ccw_to_cw_2pi(nl_t angle)
{
    /* Step 1: Normalize to [-p, p] */
    angle = angle_normalize_pi(angle);
    
    /* Step 2: Convert CCW to CW by negating */
    angle = -angle;
    
    /* Step 3: Map to [0, 2p] range */
    return yaw_convert_to_2pi(angle);
}

/**
 * @brief Normalize any angle to [0, 2p] range
 * @param[in] angle Any angle in radians
 * @return Normalized angle in radians within [0, 2p]
 * 
 */
nl_t yaw_convert_to_2pi(nl_t angle)
{
    while(angle < 0) {
        angle += 2 * M_PI;
    }
    while(angle >= 2 * M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

/**
 * @brief Normalize any angle to [-p, p] range
 * @param[in] angle Any angle in radians (can be CW or CCW, any magnitude)
 * @return Normalized angle in radians within [-p, p]
 * 
 * @note 
 * - Input can be any angle value, positive or negative, multiple turns
 * - Output is always in [-p, p] range
 * - Does not change the actual angle's meaning, just normalizes its representation
 * 
 */
nl_t angle_normalize_pi(nl_t angle)
{
    while (angle <= -M_PI) {
        angle += 2 * M_PI;
    }
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    return angle;
}

/**
 * @brief Transform ECEF coordinates to geodetic coordinates (WGS84)
 * 
 * @details Converts Earth-Centered Earth-Fixed (ECEF) cartesian coordinates 
 *          to geodetic coordinates using iterative method
 * 
 * @param[in]  r   ECEF position vector [3] {x,y,z} in meters
 *                 r[0]: X-coordinate (m)
 *                 r[1]: Y-coordinate (m)
 *                 r[2]: Z-coordinate (m)
 * @param[out] lla Geodetic coordinates [3] {latitude,longitude,height}
 *                 lla[0]: Latitude (rad) [-p/2, p/2]
 *                 lla[1]: Longitude (rad) [-p, p]
 *                 lla[2]: Ellipsoidal height (m)
 * 
 * @note Uses WGS84 ellipsoid parameters:
 *       - Semi-major axis (RE_WGS84)
 *       - First eccentricity squared (FE_WGS84)
 *       - Convergence criterion: 0.1mm
 */
void ecef2lla(const double *r, double *lla)
{
    if (!r || !lla) return;
    
    const double e2 = FE_WGS84 * (2.0 - FE_WGS84);  /* First eccentricity squared */
    const double r2 = POW2(r[0]) + POW2(r[1]);      /* Horizontal distance squared */
    const double CONV_CRITERION = 1E-4;              /* Convergence criterion (m) */
    
    /* Iterative calculation of height */
    double z = r[2], zk = 0.0;
    double v = RE_WGS84;  /* Initial meridian radius of curvature */
    
    while (fabs(z - zk) >= CONV_CRITERION) {
        zk = z;
        double sinp = z / sqrt(r2 + POW2(z));
        v = RE_WGS84 / sqrt(1.0 - e2 * POW2(sinp));
        z = r[2] + v * e2 * sinp;
    }
    
    /* Calculate final coordinates */
    if (r2 > 1E-12) {
        lla[0] = atan(z / sqrt(r2));         /* Latitude */
        lla[1] = atan2(r[1], r[0]);          /* Longitude */
    } else {
        lla[0] = (r[2] > 0.0) ? M_PI_2 : -M_PI_2;  /* Poles */
        lla[1] = 0.0;
    }
    lla[2] = sqrt(r2 + POW2(z)) - v;         /* Height */
}

/**
 * @brief Convert geodetic coordinates (LLA) to ECEF coordinates
 * 
 * @details Transforms latitude, longitude, and altitude coordinates 
 *          in WGS84 system to Earth-Centered Earth-Fixed (ECEF) 
 *          cartesian coordinates
 * 
 * @param[in]  lla Geodetic coordinates array [3]
 *                 lla[0]: Latitude (rad) [-p/2, p/2]
 *                 lla[1]: Longitude (rad) [-p, p]
 *                 lla[2]: Height above ellipsoid (m)
 * @param[out] xyz ECEF coordinates array [3] (meters)
 *                 xyz[0]: X-coordinate
 *                 xyz[1]: Y-coordinate
 *                 xyz[2]: Z-coordinate
 * 
 * @note Uses WGS84 ellipsoid parameters:
 *       - Semi-major axis (RE_WGS84)
 *       - First eccentricity squared (FE_WGS84)
 */
void lla2ecef(const double *lla, double *xyz)
{
    if (!lla || !xyz) return;

    const double lat = lla[0];
    const double lon = lla[1];
    const double alt = lla[2];

    /* Pre-compute trigonometric values */
    const double sin_lat = sin(lat);
    const double cos_lat = cos(lat);
    const double sin_lon = sin(lon);
    const double cos_lon = cos(lon);

    /* Calculate ellipsoid parameters */
    const double ecc2 = FE_WGS84 * (2.0 - FE_WGS84);  /* First eccentricity squared */
    
    /* Calculate radius of curvature in prime vertical (N) */
    const double N = RE_WGS84 / sqrt(1.0 - ecc2 * sin_lat * sin_lat);

    /* Calculate ECEF coordinates */
    xyz[0] = (N + alt) * cos_lat * cos_lon;  /* X = (N + h)cos(f)cos(?) */
    xyz[1] = (N + alt) * cos_lat * sin_lon;  /* Y = (N + h)cos(f)sin(?) */
    xyz[2] = (N * (1.0 - ecc2) + alt) * sin_lat;  /* Z = (N(1-e²) + h)sin(f) */
}

/**
 * @brief Calculate approximate distance between two geodetic points
 * 
 * @details Computes the straight-line distance between two points given in 
 *          geodetic coordinates using a simplified local tangent plane approximation.
 *          Suitable for relatively short distances (<100km) where Earth's curvature
 *          effects are negligible.
 * 
 * @param[in] lat0 Latitude of reference point (rad)
 * @param[in] lon0 Longitude of reference point (rad)
 * @param[in] hgt0 Height of reference point (m)
 * @param[in] lat1 Latitude of target point (rad)
 * @param[in] lon1 Longitude of target point (rad)
 * @param[in] hgt1 Height of target point (m)
 * 
 * @return Approximate distance between points in meters
 * 
 * @note - This is a simplified calculation using local tangent plane approximation
 *       - For more accurate results over longer distances, consider using 
 *         great circle distance or more sophisticated geodetic formulas
 *       - Reference: GPS Theory and Practice (Xu Xigang), Section 3.21
 */
nl_t calc_lla_distance(double lat0, double lon0, double hgt0, 
                      double lat1, double lon1, double hgt1)
{
    /* Calculate local tangent plane differences */
    nl_t delta_e = RE_WGS84 * (lon1 - lon0) * cos(lat0);  /* East difference */
    nl_t delta_n = RE_WGS84 * (lat1 - lat0);              /* North difference */
    nl_t delta_u = hgt1 - hgt0;                           /* Up difference */
    
    /* Return 3D Euclidean distance */
    return sqrt(POW2(delta_e) + POW2(delta_n) + POW2(delta_u));
}
