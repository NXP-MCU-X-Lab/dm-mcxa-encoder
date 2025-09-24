#include "nl.h"
#include "nl_ut.h"
#include "rtklib.h"

#include "rtcm_sample.h"

int rtkpos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav);
void rtkinit(rtk_t *rtk, const prcopt_t *opt);
void update_svr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav, int ephsat, int ephset, sbsmsg_t *sbsmsg, int index, int iobs);

static rtksvr_t rtksvr = {0};
static obsd_t obsd[MAXOBS * 2];


#define RTK_BASE_INSTANCE   (1)
#define RTK_ROVER_INSTANCE  (0)


const prcopt_t rtkopt =
{
    .mode = PMODE_KINEMA,        /* positioning mode (PMODE_???) */
    .nf = 1,                     /* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
    .navsys = SYS_GPS | SYS_CMP, /* navigation system */

    .elmin = 15.0 * D2R,                                                                          /* elevation mask angle (rad) */
    .snrmask.ena = {1, 1},                                                                        /* SNR mask */
    .snrmask.mask = {{30, 30, 30, 30, 30, 30, 30, 30, 30}, {30, 30, 30, 30, 30, 30, 30, 30, 30}}, /* mask (dBHz) at 5,10,...85 deg */

    .sateph = EPHOPT_BRDC,       /* satellite ephemeris/clock (EPHOPT_???) */
    .modear = ARMODE_FIXHOLD,    /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    .glomodear = GLO_ARMODE_OFF, /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
    .gpsmodear = 1,              /* GPS AR mode, debug/learning only (0:off,1:on) */
    .bdsmodear = 1,              /* BeiDou AR mode (0:off,1:on) */
    .arfilter = 1,               /* AR filtering to reject bad sats (0:off,1:on) */

    .maxout = 30,            /* obs outage count to reset bias */
    .minlock = 6,            /* min lock count to fix ambiguity */
    .minfixsats = 6,         /* min sats to fix integer ambiguities */
    .minholdsats = 6,        /* min sats to hold integer ambiguities */
    .mindropsats = 10,       /* min sats to drop sats in AR */
    .minfix = 10,            /* min fix count to hold ambiguity */
    .armaxiter = 1,          /* max iteration to resolve ambiguity */
    .ionoopt = IONOOPT_BRDC, /* ionosphere option (IONOOPT_???) */
    .tropopt = TROPOPT_SAAS, /* troposphere option (TROPOPT_???) */
    .dynamics = 1,           /* dynamics model (0:none,1:velociy,2:accel) */

    .niter = 1,   /* number of filter iteration */
    .intpref = 0, /* interpolate reference obs (for post mission) */

    .refpos = POSOPT_RTCM,                                  /* base position for relative mode  (0:pos in prcopt,  1:average of single pos, 2:read from file, 3:rinex header, 4:rtcm pos*/
    .outsingle = 0,                                         /* 0: supress SINGLE, 1: keep SINGLE stage */
    .eratio = {300.0, 300.0},                               /* code/phase error ratio */
    .err = {100.0, 0.003, 0.003, 0.0, 1.0, 52.0, 0.0, 0.0}, /* measurement error factor  [0]:reserved [1-3]:error factor a/b/c of phase (m) [4]:doppler frequency (hz) [5]: snr max value (dB.Hz) */

    .std = {30.0, 0.03, 0.3},                               /* initial-state std [0]bias,[1]iono [2]trop */
    .prn = {1E-4, 1E-3, 1E-4, 3E1, 1E1, 0.0},               /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
    .sclkstab = 5E-12,                                      /* satellite clock stability (sec/sec) */
    .thresar = {3.0, 0.4, 0.0, 1E-7, 0.001, 0.0, 0.0, 0.0}, /* AR validation threshold [0]:AR ratio, [1]:pos var */

    .elmaskar = 15 * D2R,   /* elevation mask of AR for rising satellite (deg) */
    .elmaskhold = 15 * D2R, /* elevation mask to hold ambiguity (deg) */
    .thresslip = 0.05,      /* slip threshold of geometry-free phase (m) */
    .thresdop = 0,
    .varholdamb = 0.1,   /* variance for fix-and-hold psuedo measurements (cycle^2) */
    .gainholdamb = 0.01, /* gain used for GLO and SBAS sats to adjust ambiguity */

    .maxtdiff = 30.0,  /* max difference of time (sec) */
    .maxinno = 1000.0, /* reject threshold of innovation (m) */
    .maxgdop = 30.0,   /* reject threshold of gdop */

    .baseline = {0.0, 0.0}, /* baseline length constraint {const,sigma} (m) */
    .ru = {0},              /* rover position for fixed mode {x,y,z} (ecef) (m) */
    .rb = {0},              /* base position for relative mode {x,y,z} (ecef) (m) */

    .anttype = {"", ""}, /* antenna types {rover,base} */
    .antdel = {{0}},     /* antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
    .pcvr = {{0}},       /* receiver antenna parameters {rov,base} */
    .posopt = {0, 0, 0, 0, 1, 0},
};



static int decoderaw(rtksvr_t *svr, int index, unsigned char *buff, int len)
{
    sbsmsg_t *sbsmsg = NULL;
    int i, ret, ephsat, ephset, fobs = 0;

    for (i = 0; i < len; i++)
    {
        ret = input_rtcm3(svr->rtcm + index, buff[i]);
        if (ret)
        {
            ephsat = svr->rtcm[index].ephsat;
            ephset = svr->rtcm[index].ephset;
            update_svr(svr, ret, &svr->rtcm[index].obs, &svr->rtcm[index].nav, ephsat, ephset, sbsmsg, index, fobs);
        }

        if (ret == 1)
            fobs = 1;
    }

    return fobs;
}


/*
 * input RTCM stream
 * sta: see RTK_ROVER_INSTANCE or RTK_BASE_INSTANCE
 */
void nl_ut_gnss_rtk_input(uint8_t sta, uint8_t *buf, uint32_t len)
{
    int fobs[2] = {0} , n, j;
    uint8_t sol_msg[MAXSOLMSG] = {0};
    double dt;

    if(sta == RTK_ROVER_INSTANCE)
    {
       // printf("[ROVER]:%d\r\n", len);

        fobs[RTK_ROVER_INSTANCE] = decoderaw(&rtksvr, RTK_ROVER_INSTANCE, buf, len);
        if(fobs[RTK_ROVER_INSTANCE] > 0)
        {
            dt = timediff(rtksvr.obs[RTK_ROVER_INSTANCE].data[0].time, rtksvr.obs[RTK_BASE_INSTANCE].data[0].time);

            n = 0;
            for (j = 0; j < rtksvr.obs[RTK_ROVER_INSTANCE].n; j++)
            {
                obsd[n++] = rtksvr.obs[RTK_ROVER_INSTANCE].data[j];
            }

            for (j = 0; j < rtksvr.obs[RTK_BASE_INSTANCE].n; j++)
            {
                obsd[n++] = rtksvr.obs[RTK_BASE_INSTANCE].data[j];
            }

            printf("R:%d,B:%d,dt:%.2f\r\n", rtksvr.obs[RTK_ROVER_INSTANCE].n, rtksvr.obs[RTK_BASE_INSTANCE].n, dt);
            rtkpos(&rtksvr.rtkqx, obsd, n, &rtksvr.nav);

            if (rtksvr.rtkqx.sol.stat != SOLQ_NONE)
            {
                // rtksvr.solopt.posf = SOLF_LLH;
                // len                = outsols(sol_msg, &rtksvr.rtkqx.sol, rtksvr.rtkqx.rb, &rtksvr.solopt);
                // sol_msg[len]       = 0;
                // printf("%s", sol_msg);

                rtksvr.solopt.posf = SOLF_ENU;
                len                = outsols(sol_msg, &rtksvr.rtkqx.sol, rtksvr.rtkqx.rb, &rtksvr.solopt);
                sol_msg[len]       = 0;
                printf("%s", sol_msg);
            }

        }
    }

    if(sta == RTK_BASE_INSTANCE)
    {
       // printf("[BASE]:%d\r\n", len);
        fobs[RTK_BASE_INSTANCE] = decoderaw(&rtksvr, RTK_BASE_INSTANCE, buf, len);
    }
}

/* init */
void nl_ut_gnss_rtk_init(void)
{
    eph_t eph0 = {0};
    int i;

    rtksvr.solopt = solopt_default;
    rtkinit(& rtksvr.rtkqx, &rtkopt);
    memset(& rtksvr.nav, 0, sizeof(nav_t));
    rtksvr.nav.eph = (eph_t *)malloc(sizeof(eph_t) * MAXSAT * 4);

    for (i = 0; i < MAXSAT * 4; i++)
         rtksvr.nav.eph[i] = eph0;
     rtksvr.nav.n = MAXSAT * 2;

    for (i = 0; i < ARRAY_SIZE(rtksvr.rtcm); i++)
    {
        init_rtcm(& rtksvr.rtcm[i]);
        rtksvr.obs[i] = rtksvr.rtcm[i].obs;
    }
}



int nl_ut_gnss_rtk(int argc, char **argv)
{
    nl_ut_gnss_rtk_init();

    uint32_t base_data_remain = base_rtcm_sample_len();
    uint32_t rover_data_remain = rover_rtcm_sample_len();
    uint8_t *base_ptr = base_rtcm_sample;
    uint8_t *rover_ptr = rover_rtcm_sample;

    while(1)
    {
        /* recv rover stram */
        if(rover_data_remain)
        {
            nl_ut_gnss_rtk_input(RTK_ROVER_INSTANCE, rover_ptr++, 1);
            rover_data_remain--;
        }

        /* recv base stram */
        if(base_data_remain)
        {
            nl_ut_gnss_rtk_input(RTK_BASE_INSTANCE, rover_ptr++, 1);
            base_data_remain--;
        }

        if(!rover_data_remain && !base_data_remain) break;
    }

//        printf("\r\n\r\nRTKLIB:\r\n");
//        rtkpos(&rtksvr.rtkqx, obsd, n, &rtksvr.nav);
//        // printf("\r\n\r\nRTKLIB:\r\n");
//        // rtkpos(&rtksvr.rtkqx, obsd, n, &rtksvr.nav);

//        printf("\r\n\r\nNL_GNSS_RTK:\r\n");
//        gnss_rtk(&rtksvr.obs[RTK_ROVER_INSTANCE], &rtksvr.obs[RTK_BASE_INSTANCE], &rtksvr.nav);

    return 0;
}




