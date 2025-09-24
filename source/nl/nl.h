#ifndef _NL_H
#define _NL_H

#ifdef __cplusplus
extern "C"{
#endif

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <ctype.h>

#include "nl_cfg.h"
#include "nl_def.h"


typedef struct          /* time struct */
{
    time_t time;        /* time (s) expressed by standard time_t */
    double sec;         /* fraction of second under 1 s */
} nl_gtime_t;



typedef struct
{
    nl_t sum_x;
    nl_t sum_y;
    nl_t sum_xy;
    nl_t sum_x2;
    nl_t p0;            /* slope */
    nl_t p1;            /* intercept */
    uint32_t n;
}linreg_t;



//typedef struct
//{
//    obs_t obs[3]; /* Rover Master:0, Rover Slave:1, Base:2 */
//    nav_t nav;
//} gnss_raw_t;

//typedef struct
//{
//    nl_t p[3]; /* Position(m) in ecef */
//    nl_t v[3]; /* Velocity(m/s) in ecef */
//} gnss_sat_pv_t;

//typedef struct
//{
//    nl_t bias;  /* Satellite clock bias (s) */
//    nl_t drift; /* Satellite clock drift (s/s) */
//} gnss_sat_dts_t;

//typedef struct
//{
//    gnss_sat_pv_t pv[MAX_OBS_NS];   /* Satellite position and velocity */
//    gnss_sat_dts_t dts[MAX_OBS_NS]; /* Satellite clock bias and drift */
//    nl_t var[MAX_OBS_NS];           /* Satellite position and clock error variance (m^2) */
//    int svh[MAX_OBS_NS];            /* Satellite health flag (-1:correction not available) */
//    int eph[MAX_OBS_NS];            /* Satellite ephemeris flag (-1:correction not available) */
//    int spp_idx[MAX_OBS_NS];        /* Satellite index for spp */
//    int rtk_idx[MAX_OBS_NS];        /* Satellite index for rtk */

//    nl_t e[MAX_OBS_NS][3];                   /* Direction vector in ecef */
//    nl_t dist[MAX_OBS_NS];                   /* Distance(m) */
//    nl_t az[MAX_OBS_NS];                     /* Azimuth(rad) */
//    nl_t el[MAX_OBS_NS];                     /* Elevation(rad) */
//    nl_t lambda[MAX_OBS_NS][GNSS_FREQ_NUM];  /* Wavelength(m) */
//    nl_t freq[MAX_OBS_NS][GNSS_FREQ_NUM];    /* Frequency(Hz) */

//    size_t obs_n; /* Number of observations */
//    size_t spp_n; /* Number of satellites for spp */
//    size_t rtk_n; /* Number of satellites for rtk */
//} gnss_sat_t;

typedef struct /* RTCM control struct type */
{        
    int nbyte;          /* number of bytes in message buffer */ 
    int len;            /* message length (bytes) */
    uint8_t buf[1200];  /* message buffer */
    uint16_t type;
} nl_rtcm_t;


/* GNSS solution data validity flags */
#define GNSS_DATA_VALID_TIME        (1 << 0)   /* GPS time valid */
#define GNSS_DATA_VALID_POS         (1 << 1)   /* Position (lla) valid */
#define GNSS_DATA_VALID_VEL         (1 << 2)   /* Velocity (vel_enu) valid */
#define GNSS_DATA_VALID_POS_STD     (1 << 3)   /* Position std valid */
#define GNSS_DATA_VALID_VEL_STD     (1 << 4)   /* Velocity std valid */
#define GNSS_DATA_VALID_HEADING     (1 << 5)   /* Dual antenna heading valid */
#define GNSS_DATA_VALID_DOP         (1 << 6)   /* DOP values valid */
#define GNSS_DATA_VALID_UNDULATION  (1 << 7)   /* Undulation valid */

typedef struct
{
    nl_gtime_t gpst;
    uint8_t nv;
    uint8_t nv_heading;
    uint8_t solq;          /* aligned with RTKLIB */
    uint8_t solq_heading;  /* dual_heading solq, alighed with RTKLIB, see: SOLQ_SINGLE */
    double lla[3];         /* lat(rad), lon(rad), m(eplispoid hight(HAE, h_ref) = msl + undulation)： 椭球高=海拔高+undulation */
    nl_t pos_enu_std[3];
    nl_t vel_enu_std[3];
    nl_t vel_enu[3];     /* m/s */

    nl_t dual_gnss_pitch_deg;
    nl_t dual_gnss_base_len;
    nl_t dual_gnss_heading_deg;
    
    nl_t cog; /* RMC: course over ground */
    nl_t sog; /* RMC: speed over ground */
    nl_t hdop;
    nl_t vdop;
    nl_t pdop;
    nl_t gdop;
    nl_t tdop;
    
    nl_t undulation; /* The difference between ellipsoid height and mean sea level. Equals (h_ref - msl) */
    nl_t diff_age;
    uint8_t leap_sec;
    
    /* New fields for data validity and message type */
    uint32_t valid_flags;    /* Data validity flags (GNSS_DATA_VALID_xxx) */
} gnss_sol_t;



typedef struct
{
    /* Initial diagonal elements of matrix P */
    nl_t P0_att[3];         /* attitude [0]=pitch, [1]=roll) [2]=yaw,rad */
    nl_t P0_vel;            /* velocity m/s */

    nl_t P0_pos;            /* position m */
    nl_t P0_gyr_bias;       /* gyr bias rad/s */
    nl_t P0_acc_bias;       /* acc bias m/s^(2) */
    nl_t P0_installangle;   /* installangle err rad */
    nl_t P0_timedelay;      /* timedelay err rad */

    /* Max restriction diagonal elements of matrix P */
    nl_t Pmax_att;     /* follow P0 */
    nl_t Pmax_vel;
    nl_t Pmax_pos;
    nl_t Pmax_gyr_bias;
    nl_t Pmax_acc_bias;

    /* Min restriction elements of matrix P */
    nl_t Pmin_att;    /* follow P0 */
    nl_t Pmin_vel;
    nl_t Pmin_pos;
    nl_t Pmin_gyr_bias;
    nl_t Pmin_acc_bias;


    /* Process noise */
    nl_t Qgyr_wb;       /* Gyroscope white noise          rad/s   */
    nl_t Qacc_wb;       /* Accelerometer white noise      m/s^(2) */
    nl_t Qgyr_wbb;      /* Gyroscope bias white noise     rad/s   */
    nl_t Qacc_wbb;      /* Accelerometer bias white noise m/s^(2) */

    nl_t Q_installangle;    /*installangle white noise rad */
    nl_t Q_timedelay;    /*timedelay white noise rad */

    /* Measurement noise */
    nl_t R0_gnss_vel; /* GNSS's horizontal velocity, vertical velocity, m/s*/
    nl_t R0_gnss_pos; /* GNSS's horizontal position, vertical position, m  */

    nl_t R0_dualgnss;    /* Dual GNSS */
    nl_t R0_dualgnss_heading; /* Dual GNSS heading*/
    nl_t R0_zupt;        /* ZUPT */
    nl_t R0_nhc;         /* NHC */
    nl_t R0_gravity;      /* Garivy */
    nl_t R0_od;          /* 里程计 */
    nl_t R0_zaru;
    nl_t R0_zihr;        /* ZIHR, 静止Yaw防漂移,  Zero Integrated Heading Rate，ZIHR, 等红绿灯时，yaw会漂移，类似ZUPT的方法，构造虚拟观测量，减小漂移。 see: 1005-6734(2020)06-0701-08： 基于手机内置传感器的车辆组合定位方法, and https://zhuanlan.zhihu.com/p/115529319 */
    nl_t gnss_delay;     /* 时间不同步补偿: s: 最近的IMU采样时间  - 最近的GPS采样时间 */
} eskf156_opt_t;


typedef struct
{
    kf_state_t s;
    uint8_t idx_a;
    uint8_t idx_v;
    uint8_t idx_p;
    uint8_t idx_wb;
    uint8_t idx_gb;
    uint8_t idx_installangle;
    uint8_t idx_timedelay;
}lc_eskf_t;


typedef struct
{
    /* Initial diagonal elements of matrix P */
    nl_t P0_att[3];          /* attitude rad */
    nl_t P0_gyr_bias;        /* gyr biasrad/s */
    nl_t Pmax_att;           /* follow P0 */
    nl_t Pmin_att;           /* follow P0 */
    nl_t Pmax_gyr_bias;
    nl_t Pmin_gyr_bias;
    
    /* Process noise */
    nl_t Qgyr_wb;            /* Gyroscope white noise          rad/s   */
    nl_t Qgyr_wbb;           /* Gyroscope bias white noise     rad/s   */
    
    /* Measurement noise */
    nl_t R0_gravity;            /* R0 Garivy */
    nl_t R0_mag;                /* R0 mag */
    nl_t R0_zaru;
    nl_t R0_zihr;
} eskfatt_opt_t;

typedef struct
{
    /* Initial diagonal elements of matrix P */
    nl_t P0_mag_n[3];          /* mag_n uT */
    nl_t P0_mag_bias;        /* mag_b uT */
    nl_t Pmax_mag_n;           /* follow P0 */
    nl_t Pmin_mag_n;           /* follow P0 */
    nl_t Pmax_mag_bias;
    nl_t Pmin_mag_bias;
    
    /* Process noise */
    nl_t Q_mag_n;               /* */
    nl_t Q_mag_bias;            /* */
    
    /* Measurement noise */
    nl_t R0_mag;                /* R0 mag */
} kfmag_opt_t;

uint32_t nl_getbitu(const uint8_t *buf, int pos, int len);


nl_t *vcreate(size_t num);
void vprint(nl_t *A, size_t n, size_t q);
void vcopy(nl_t *A, nl_t *B, size_t len);
void vfill(nl_t *A, nl_t val, size_t len);
void vadd(nl_t *C, nl_t *A, nl_t *B, size_t len);
void vadd2(nl_t *A, nl_t *B, size_t len);
void vsub(nl_t *C, nl_t *A, nl_t *B, size_t len);
void vsub2(nl_t *A, nl_t *B, size_t len);
void vscale(nl_t *A, nl_t *B, nl_t k, size_t len);
void vscale2(nl_t *A, nl_t k, size_t len);
void vnormlz(nl_t *A, size_t len);
nl_t vnorm(nl_t *A, size_t len);
nl_t mean(nl_t *V, size_t len);
nl_t vdot(nl_t *A, nl_t *B, size_t len);
void vconstrain(nl_t *X, nl_t max, nl_t min, size_t len);
void v3constrain(nl_t *X, nl_t max, nl_t min);

uint32_t vec2str(char *buf, uint32_t size, nl_t *A, size_t n, size_t q);
uint32_t vec2str_scale(char *buf, uint32_t size, nl_t *A, nl_t k, size_t n, size_t q);
uint32_t mat2str(char *buf, uint32_t size, m_t *A, size_t q);
void nl_time2str(nl_gtime_t t, char *s, int n);
void v3print(nl_t *A, size_t q);
void v3print_scale(nl_t A[], size_t q, nl_t scale);
void v3copy(nl_t *A, nl_t *B);
void v3fill(nl_t *A, nl_t val);
void v3add(nl_t *C, nl_t *A, nl_t *B);
void v3add2(nl_t *A, nl_t *B);
void v3sub(nl_t *C, nl_t *A, nl_t *B);
void v3sub2(nl_t *A, nl_t *B);
void v3scale(nl_t *A, nl_t *B, nl_t k);
void v3scale2(nl_t *A, nl_t k);
void v3cross(nl_t *A, nl_t *B, nl_t *C);
void v3normlz(nl_t *A);
nl_t v3norm(nl_t *A);
nl_t v3dot(nl_t *A, nl_t *B);

m_t *mcreate(size_t r, size_t c);
m_t *mcreate2(size_t r, size_t c, nl_t *p);
void meye(m_t *A);
void minit(m_t *A, size_t r, size_t c, nl_t *p);
void mprint(m_t *A, size_t q);
void nl_skew(m_t *m, nl_t *v);
void mfill(m_t *A, nl_t val);
void mfilldiag(m_t *A, nl_t val);
void msetdiag(m_t *A, nl_t *V);
void mcopy(m_t *A, m_t *B);
void mbcopy(m_t *A, m_t *m, size_t start_r, size_t start_c);
void madd(m_t *A, m_t *B, m_t *C);
void madd2(m_t *A, m_t *B);
void msub(m_t *A, m_t *B, m_t *C);
void msub2(m_t *A, m_t *B);
void mscale(m_t *A, m_t *B, nl_t k);
void mscale2(m_t *A, nl_t k);
void msetrow(m_t *A, int r, nl_t* V);
void mgetrow(m_t *A, int r, nl_t* V);
void mgetcol(m_t *A, int c, nl_t *V);
void msetcol(m_t *A, int c, nl_t* V);
void madddiag(m_t *A, nl_t *d);
void madddiag2(m_t *A, nl_t alpha, nl_t *d);
void maddeye(m_t *A);
void mvmul(m_t *A, nl_t *v, nl_t *s);
void mmul(m_t *A, m_t *B, m_t *C);
void mmul3(const char *tr, nl_t alpha, m_t *matA, m_t *matB, nl_t beta, m_t *matC);
nl_t mtrace(m_t *A);
void mtrans(m_t *A, m_t *AT);
int minv(m_t *A);
nl_t det(m_t *A);

/* nl_mat2 */
void jcbj(m_t *A, m_t *V, nl_t eps);


nl_t *nl_mat(int n, int m);
int *nl_imat(int n, int m);
void nl_matcpy(nl_t *A, const nl_t *B, int n, int m);

void qmul(const nl_t *q1, const nl_t *q2, nl_t *r);
void qmul2(nl_t *q1, const nl_t *q2);
void qmul3(const nl_t *q1, nl_t *q2);
void qnormlz(nl_t *q);
void qidentity(nl_t *q);
void qconj(nl_t *q, nl_t *qj);
void qconj2(nl_t *q);
void qmulv(const nl_t *q, const nl_t *v, nl_t *r);
void rv2q(nl_t *rv, nl_t *q);
void q2att(nl_t *q, att_t *att);
void att2q(att_t *att, nl_t *q);
void q2dcm(nl_t *q, nl_t *p);

int nl_lsq(m_t *A, nl_t *y, nl_t *x, m_t *Q);
int nl_lsq2(m_t *A, m_t *Y, m_t *X, m_t *Q);

linreg_t *linreg_create(void);
void linreg_clear(linreg_t *lr);
void linreg_fit(linreg_t *lr);
void linreg_add(linreg_t *lr, nl_t x, nl_t y);


void ins_init(ins_t *ins);
void ins_set_coord(ins_t *ins, uint8_t coord);
void ins_align(ins_t *ins, nl_t *acc, nl_t yaw);
void ins_set_yaw(ins_t *ins, nl_t yaw);
void ins_set_inital_lla(ins_t *ins, double *lla);
void ins_update(ins_t *ins, nl_t *gyro, nl_t *acc, nl_t dt);
int ins2str(char *buf, ins_t *ins);

void kf_state_create(kf_state_t *kf, size_t n);
void kf_meas_create(kf_meas_t *meas, kf_state_t *s, size_t m);
void kf_set_meas_r(kf_meas_t *meas, nl_t R_std);
void kf_p_constrain(kf_state_t *s);
void kf_state_update(kf_state_t *s, nl_t dt);
void kf_get_state_std(kf_state_t *s, int start_idx, int count, nl_t *std);
void kf_meas_update(kf_meas_t *m);
nl_t kf_meas_get_chi_lambda(kf_meas_t *m);
void Scalar_KalmanFilte(nl_t* x, nl_t zk, nl_t* P, nl_t Q, nl_t R, nl_t MaxError);
void lc_eskf_reset_p(kf_state_t *s);
void lc_eskf15_state_model(kf_state_t *s, ins_t *ins, nl_t dt);
void lc_eskf_feedback(lc_eskf_t *lc, ins_t *ins, uint32_t opt, nl_t alpha_base);

void lc_eskf_create(lc_eskf_t *lc, const eskf156_opt_t *opt);
void lc_eskf_att_create(lc_eskf_t *lc, const eskfatt_opt_t *opt);

void lc_eskf_create_meas_pos(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_vel(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_zupt(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_zaru(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_zihr(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_mag(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_gravity(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_dual_gnss(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_dual_gnss_heading(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_nhc(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);
void lc_eskf_create_meas_odo(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r);


void lc_eskf_meas_set_pos_std(kf_meas_t *m, nl_t *RposENU);
void lc_eskf_meas_set_vel_std(kf_meas_t *m, nl_t *RvelENU);
void lc_eskf_meas_set_zaru_std(kf_meas_t *m, nl_t R);
void lc_eskf_meas_set_pos(kf_meas_t *m, ins_t *ins, nl_t *pos_enu, nl_t gnss_delay, nl_t *lever);
void lc_eskf_meas_set_vel(kf_meas_t *m, ins_t *ins, nl_t *gyr, nl_t *vel_enu, nl_t gnss_delay, nl_t *lever);
void lc_eskf_meas_set_dualgnss_h(kf_meas_t *m,  nl_t *vec_n);
void lc_eskf_meas_set_dualgnss(kf_meas_t *m, ins_t *ins,  nl_t *bl_vec,  nl_t *antA,  nl_t *antB);
void lc_eskf_meas_set_dualgnss_heading(kf_meas_t *m, ins_t *ins,nl_t heading_dual, nl_t heading_v2dual, nl_t ante_heading, nl_t gnss_delay);
void lc_eskf_meas_set_gravity(kf_meas_t *m, ins_t *ins, nl_t *acc);
void lc_eskf_meas_set_mag(kf_meas_t *m, ins_t *ins, nl_t *mag);

void lc_eskf_meas_set_zupt(kf_meas_t *m, ins_t *ins);
void lc_eskf_meas_set_zaru(kf_meas_t *m, ins_t *ins, nl_t *wb);
void lc_eskf_meas_set_zihr(kf_meas_t *m, ins_t *ins, nl_t yaw_zihr);
void lc_eskf_meas_set_nhc(kf_meas_t *m, ins_t *ins, uint8_t est_install_angle);
void lc_eskf_meas_set_od(kf_meas_t *m, ins_t *ins, nl_t od_vel, nl_t od_scale_factor);

void lc_eskf_get_std_att(lc_eskf_t *lc, nl_t *std);
void lc_eskf_get_std_wb(lc_eskf_t *lc, nl_t *std);
void lc_eskf_get_std_v(lc_eskf_t *lc, nl_t *std);
void lc_eskf_get_std_p(lc_eskf_t *lc, nl_t *std);
void lc_eskf_get_std_gb(lc_eskf_t *lc, nl_t *std);
nl_t lc_eskf_get_std_install_angle_yaw(lc_eskf_t *lc);
    
void eskfatt_create(kf_state_t *s, kf_meas_t *gravity, kf_meas_t *mag, kf_meas_t *zaru, const eskfatt_opt_t *opt);
void eskfatt_state_model(kf_state_t *s, ins_t *ins, nl_t dt);
void eskfatt_fb(kf_state_t *s, ins_t *ins, uint32_t opt);
void direct_nonlinear_correction(ins_t *ins, nl_t *acc);

//void gnss_spp(obs_t *obs, nav_t *nav, gnss_sat_t *sat, gnss_sol_t *sol);
//void gnss_rtk(obs_t *obs_rover, obs_t *obs_base, nav_t *nav);


void lla2enu(ins_t *ins, double *lla, double *enu);
nl_t calc_lla_distance(double lat0, double lon0, double hgt0, double lat1, double lon1, double hgt1);
void enu2lla(ins_t *ins, double *enu, double *lla);
void ecef2lla(const double *r, double *lla);
void lla2ecef(const double *lla, double *r);
void nl_hpl2vec(nl_t heading, nl_t pitch, nl_t len, nl_t *enu);
void nl_vec2hpl(nl_t *enu, nl_t *heading, nl_t *pitch, nl_t *len);
nl_t yaw_ccw_to_cw_2pi(nl_t yaw_npi_pi);
nl_t yaw_convert_to_2pi(nl_t yaw_npi_pi);
nl_t angle_normalize_pi(nl_t angle);
void nl_deg2dms(double deg, double *dms, int ndec);
void nl_time2epoch(nl_gtime_t t, nl_t *ep);
nl_gtime_t nl_epoch2time(const nl_t *ep);
nl_gtime_t nl_gpst2time(int week, double sec);
double nl_time2gpst(nl_gtime_t t, int *week);
nl_gtime_t nl_utc2gpst(nl_gtime_t t);
double nl_timediff(nl_gtime_t t1, nl_gtime_t t2);
nl_gtime_t nl_gpst2utc(nl_gtime_t t);
nl_gtime_t nl_timeadd(nl_gtime_t t, double sec);


int nl_satno(int sys, int prn);
int nl_satsys(int sat, int *prn);
int nl_satid2no(const char *id);
void nl_satno2id(int sat, char *id);

uint32_t nl_gnss_sol2str(char *buf, gnss_sol_t *sol);

/* porting */
uint32_t nl_get_sys_ms(void);
void *nl_malloc(int nbytes);
void nl_free(void *ptr);
void nl_printf(const char *format, ...);
void nl_enter_critical(void);
void nl_exit_critical(void);
int nl_mdelay(int ms);

double lla2err(double *lla, double *lla0);

int nl_sphere_fit (m_t *D, m_t *C, nl_t *B, nl_t *norm, nl_t *fit_err);
int nl_ellipsoid_fit(m_t *D, m_t *C, nl_t *B, nl_t *norm, nl_t *fit_err);
nl_t nl_ellipsoid_fit_get_residuals(nl_t *data, uint32_t size, m_t *C, nl_t *B, nl_t n);

void mahony_gravity_fb(nl_t *q, nl_t *acc, nl_t *e);
void mahony_set_kp_gravity(nl_t kp);
void mahony_mag_fb(nl_t *q, nl_t *mag, nl_t *e);

uint32_t nl_check_vec(nl_t *v1, nl_t *v2, uint32_t len, nl_t tol);


/* nl_app */
void nl_adj_att(nl_t *qin, att_t *att_tgt, nl_t *qout);

#ifdef __cplusplus
}
#endif

#endif
