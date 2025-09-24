#ifndef __NL_DEF_H__
#define __NL_DEF_H__

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "nl_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif



#if !defined(RT_EOK)
#define RT_EOK                          0               /**< There is no error */
#endif

#if !defined(RT_ERROR)
#define RT_ERROR                        1               /**< A generic error happens */
#endif

#if !defined(ARRAY_SIZE)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define POW2(x)                 ((x) * (x))
#define MIDX(m, row, col)       ((row) * (m->c) + (col))
#define MELEMENT(m, row, col)   (m)->dat[MIDX((m), (row), (col))]
#define M2V(m)                  ((nl_t *)m->dat)
#define M2R(m, row)             ((nl_t *)m->dat + m->c * (row))

/* nl math const */
#ifndef D2R
#define D2R     (0.0174532925199433f)
#endif



#ifndef R2D
#define R2D     (57.2957795130823f)
#endif

#ifndef M_PI
#define M_PI      (3.1415926535897932)
#endif

#ifndef CLIGHT
#define CLIGHT  (299792458.0)         /* speed of light (m/s) */
#endif

#ifndef M_PI_2
#define M_PI_2    (1.57079632679489661923)
#endif

#ifndef M_SQRT2
    #define M_SQRT2 1.4142135623730951f  // Value of sqrt(2)
#endif

#ifndef GRAVITY
#define GRAVITY (9.8f)
#endif

#ifndef M2DM
#define M2DM (10.0f)
#endif

#ifndef OMEGA_E
#define OMEGA_E (7.2921151467E-5f) /* earth angular velocity (IS-GPS) (rad/s) */
#endif

#ifndef RE_WGS84
#define RE_WGS84 (6378137.0f) /* earth semimajor axis (WGS84) (m) */
#endif

#ifndef FE_WGS84
#define FE_WGS84 (0.00335281066474748f) /* earth flattening (WGS84) */
#endif

#ifndef MAXLEAPS
#define MAXLEAPS    18                  /* max number of leap seconds table */
#endif


#ifndef NL_PRINT_WIDTH
#define NL_PRINT_WIDTH  (8)
#endif

#ifndef MAXRAWLEN
#define MAXRAWLEN   256               /* max length of receiver raw message */
#endif

/* nl error code definitions */
#ifndef RT_EOK
#define RT_EOK      0
#endif

#ifndef RT_NULL
#define RT_NULL      0
#endif

/* nl GNSS solution quality */
#ifndef SOLQ_NONE
#define SOLQ_NONE 0
#endif

#ifndef SOLQ_FIX
#define SOLQ_FIX 1
#endif

#ifndef SOLQ_FLOAT
#define SOLQ_FLOAT 2
#endif

#ifndef SOLQ_SINGLE
#define SOLQ_SINGLE 5
#endif

#ifndef SOLQ_DGPS
#define SOLQ_DGPS 4     /* solution status: DGPS/DGNSS */
#endif

#ifndef SOLQ_DR
#define SOLQ_DR 6
#endif

#ifndef SOLQ_BASE
#define SOLQ_BASE 7
#endif

#ifndef SOLQ_PPP
#define SOLQ_PPP    8   /* solution status: PPP */
#endif

#ifndef SOLQ_SBAS
#define SOLQ_SBAS   3   /* solution status: SBAS */
#endif

#define NL_COORD_ENU     (0)
#define NL_COORD_NED     (2)
#define NL_COORD_NWU     (4)

/* nl INS and ESKF status */
#define INS_INACTIVE (0)
#define INS_ALIGNING (1)
//#define INS_HIGH_VARIANCE (2)
#define INS_SOLUTION_GOOD (3)
#define INS_SOLUTION_FREE (6)
//#define INS_ALIGNMENT_COMPLETE (7)
//#define INS_DETERMINING_ORIENTATION (8)
//#define INS_WAITING_INITIALPOS (9)
//#define INS_WAITING_AZIMUTH (10)


///* GNSS config */
//#define GPS_NS          32                             /* GPS max number of satellites */
//#define BDS_NS          46                             /* BDS max number of satellites */
//#define MAX_SAT_NUM     (GPS_NS+BDS_NS)                /* max number of satellites */
//#define GNSS_SYS_NUM    2                              /* GPS + BDS */
//#define GNSS_FREQ_NUM   2                              /* L1  + L5 */

//#define KF_STATE_NUM    (3+MAX_SAT_NUM*GNSS_FREQ_NUM)  /* state number of kalman filter */

//#define MIN_SOL_NS      5                              /* minimum number of satellites for solution */
//#define MAX_OBS_NS      36                             /* maximum number of satellites for observation */
//#define MIN_COMM_NS     5                              /* minimum number of satellites for RTK */
//#define MAX_COMM_NS     20                             /* maximum number of satellites for RTK */
//#define MIN_EL          (15 * D2R)                     /* minimum elevation of satellite for solution */
//#define MIN_SNR         30                             /* minimum SNR of satellite for solution */
//#define MAX_EPH_VAR     POW2(300.0)                    /* max variance eph to reject satellite (m^2) */
//#define MAX_SPP_ITER    10                             /* max number of iterations for single point positioning */

//#define ERR_ION         5.0                            /* ionospheric delay std (m) */
//#define ERR_TROP        3.0                            /* tropspheric delay std (m) */
//#define ERR_SAAS        0.3                            /* Saastamoinen model error std (m) */
//#define ERR_BRDCI       0.5                            /* broadcast ionosphere model error factor */
//#define ERR_CBIAS       0.3                            /* code bias error std (m) */
//#define REL_HUMI        0.7                            /* relative humidity for Saastamoinen model */


#define INS_UPDATE_A        (1<<0)                                                  /**< INS update att. */
#define INS_UPDATE_V        (1<<1)                                                  /**< INS update vel. */
#define INS_UPDATE_P        (1<<2)                                                  /**< INS update pos. */
#define INS_UPDATE_ALL      (INS_UPDATE_A | INS_UPDATE_V | INS_UPDATE_P)


#define ESKF_FB_A_XY            (1<<0)                                              /**< only fb XY aixs. */
#define ESKF_FB_A_XYZ           (1<<1)                                              /**< fb all XYZ. */
#define ESKF_FB_V_XYZ           (1<<3)
#define ESKF_FB_P_XYZ           (1<<5)
#define ESKF_FB_G_XYZ           (1<<7)
#define ESKF_FB_W_XY            (1<<8)
#define ESKF_FB_W_XYZ           (1<<9)
#define ESKF_FB_INSTALL_CBV     (1<<10) /* veilicle install angle feedback */
#define ESKF_FB_TIMEDELAY       (1<<11) /* timedelay feedback */
#define ESKF_FB_ALL             (ESKF_FB_A_XYZ | ESKF_FB_V_XYZ | ESKF_FB_P_XYZ | ESKF_FB_G_XYZ | ESKF_FB_W_XYZ | ESKF_FB_TIMEDELAY)

/* basic navigation state and math struct */

/*
matrix format:
    m = 3, n = 3

       n      n      n
m   |[ 0]  |[ 1]  |[ 2]
m   |[ 3]  |[ 4]  |[ 5]
m   |[ 6]  |[ 7]  |[ 8]
*/

typedef struct
{
    uint8_t     r;      /* number of rows of the matrix.    also known as m */
    uint8_t     c;      /* number of columns of the matrix. also known as n */
    uint16_t    s;      /* total size in the matrix.        m*n */
    nl_t *dat;          /* points to the data of the matrix. */
} m_t;

typedef struct
{
    nl_t        pitch;  /* -pi/2 : pi/2. */
    nl_t        roll;   /*  -pi : pi. */
    nl_t        yaw;    /*  -pi : pi. */
    uint8_t     coord;  /* NL_COORD_ENU, NL_COORD_NED, NL_COORD_NWU */
} att_t;


typedef struct
{
    double p[3];                /* Position(m) in the direction of east, north and up(relative to lla0) */
    nl_t v[3];                  /* Velocity(m/s) in the direction of east, north and up */
    nl_t q[4];                  /* Attitude quaternion (n->b) */
    nl_t a[3];                  /* Acceleration (m/s^2) in the direction of east, north and up */
    nl_t f[3];                  /* Specific force (m/s^2) in the direction of east, north and up */
    nl_t wb[3];                 /* gyr bias, rad, estmiated by KF */
    nl_t gb[3];                 /* acc bias, m/s^(2),  estmiated by KF * */
    att_t att;                  /* Attitude(Euler) n2b*/ 
    nl_t TrackAngle;
    nl_t gyr_comp[3];           /* gyro rate after bias compensation, gyr_comp = (gyr - wb) rad/s */
    nl_t acc_comp[3];           /* acc data after bias compensation, m/s^2 */
    
    double lla0[3];             /* Initial latitude(rad), longitude(rad), altitude(m) */
    double lla[3];              /* real time latitude(rad), longitude(rad), altitude(, elpsipod height, m) */
    double Rew;                 /* East-West earth radius(m), only set when initalized, (Rn) 东西向地球曲率半径, 卯酉圈曲率半径(横着的)*/
    double Rns;                 /* North-South earth radius(m), only set when initalized ,(Rm) 南北向地球曲率半径, 子午圈曲率半径(竖着的) */
    uint32_t ins_update_flag;   /* ins_update option, see:  INS_UPDATE_A */
    uint32_t ins_stat;          /* see INS_SOLUTION_GOOD */
    
    /* vichle install angle estimiate */
    nl_t Qb2v[4];               /* Attitude quaternion (b->v) */  
    m_t *Cb2v;
    att_t att_b2v;              /* Attitude(Euler) b->v*/
    
    nl_t timedelay;             /* gnss timedelay */
    /* ENU or NED */
    uint8_t coord;              /* ENU or NED */
} ins_t;


/* Kalman filter struct 
n: # of state
m: # of measurement
*/
typedef struct
{
    m_t *X;
    m_t *Xtmp;
    m_t *P;
    m_t *Ptmp;
    m_t *F;
    m_t *KH;
    nl_t *Q;
    nl_t *Pmax; /* MAX limit of P */
    nl_t *Pmin; /* MIN limit of P */
} kf_state_t;

typedef struct
{
    m_t *Z;         /* m x 1 */
    m_t *H;         /* m x n */
    m_t *PHt;
    m_t *S;         /* H*P*H' + R : n x n */
    m_t *K;
    nl_t *R;        /* m */
    
    /* use for compute chi_lambda */
    m_t *chi_tmp;   /* m x 1 */
    m_t *innov;     /* m x 1 */
    m_t *lambda;    /* 1 x 1 */
    
    kf_state_t *kf_state;  /* pointer to the kf status */
} kf_meas_t;



#ifdef __cplusplus
}
#endif

#endif
