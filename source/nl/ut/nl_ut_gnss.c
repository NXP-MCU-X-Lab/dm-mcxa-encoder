#include "nl.h"
#include "nl_ut.h"
//#include "rtklib.h"

#include "rtcm_sample.h"



static void dump_obs(obs_t *obs)
{
    int i = 0, len = 0;
    uint8_t buf[512];
    for (i = 0; i < obs->n; i++)
    {
        len = nl_obsd2str(buf, &obs->data[i], 'N');
        printf("%s", buf);
    }
}

static void dump_nav(nav_t *nav)
{
    int i = 0, len = 0;
    uint8_t buf[512];
    for (i = 0; i < nav->n; i++)
    {
        len = nl_eph2str(buf, &nav->eph[i], 'S');
        printf("%s", buf);
    }
}




const prcopt_t spp_rtkopt =
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



int nl_ut_gnss_spp(int argc, char **argv)
{
    size_t i;
    static rtcm_t rtcm_rover = {0}, rtcm_base = {0};
    char log[1024];

    init_rtcm(&rtcm_rover);
    for (i = 0; i < rover_rtcm_sample_len(); i++)
    {
        input_rtcm3(&rtcm_rover, rover_rtcm_sample[i]);
    }

    init_rtcm(&rtcm_base);
    for (i = 0; i < base_rtcm_sample_len(); i++)
    {
        input_rtcm3(&rtcm_base, base_rtcm_sample[i]);
    }

    printf("\r\nBase OBS:\r\n");
    dump_obs(&rtcm_base.obs);
    printf("\r\nRover OBS:\r\n");
    dump_obs(&rtcm_rover.obs);
    printf("\r\nRover NAVI:\r\n");
    dump_nav(&rtcm_rover.nav);

    double lla0[3] = {39.94010831 * D2R, 116.37498051 * D2R, 47}; // true position

    /* try RTKLIB SPP */
    sol_t sol = {0};
    char err_msg[MAXSOLMSG];

    nl_t rtcm1005_lla[3];
    ecef2pos(rtcm_base.sta.pos, rtcm1005_lla);

    printf("\r\nRTKLIB:\r\n");
    pntpos(rtcm_base.obs.data, rtcm_base.obs.n, &rtcm_rover.nav, &spp_rtkopt, &sol, NULL, NULL, err_msg);
    if (sol.stat > SOLQ_NONE)
    {
        nl_t lla[3];
        ecef2pos(sol.rr, lla);
        printf("BASE_SPP:  %.7f, %.7f, %.3f\t", lla[0] * R2D, lla[1] * R2D, lla[2]);
        printf("POS_ERROR: %.3fm\r\n", lla2err(lla, rtcm1005_lla));
    }

    pntpos(rtcm_rover.obs.data, rtcm_rover.obs.n, &rtcm_rover.nav, &spp_rtkopt, &sol, NULL, NULL, err_msg);
    if (sol.stat > SOLQ_NONE)
    {
        nl_t lla[3];
        ecef2pos(sol.rr, lla);
        printf("ROVER_SPP: %.7f, %.7f, %.3f\t", lla[0] * R2D, lla[1] * R2D, lla[2]);
        printf("POS_ERROR: %.3fm\r\n", lla2err(lla, lla0));
    }

    printf("\r\nNL:\r\n");

    /* try NL SPP */
    gnss_sat_t rover_sat = {0}, base_sat = {0};
    gnss_sol_t rover_sol = {0}, base_sol = {0};
    gnss_spp(&rtcm_rover.obs, &rtcm_rover.nav, &rover_sat, &rover_sol);
    gnss_spp(&rtcm_base.obs, &rtcm_rover.nav, &base_sat, &base_sol);

    printf("BASE_SPP:  %.7f, %.7f, %.3f\t", base_sol.lla[0] * R2D, base_sol.lla[1] * R2D, base_sol.lla[2]);
    printf("POS_ERROR: %.3fm\r\n", lla2err(base_sol.lla, rtcm1005_lla));
    printf("ROVER_SPP: %.7f, %.7f, %.3f\t", rover_sol.lla[0] * R2D, rover_sol.lla[1] * R2D, rover_sol.lla[2]);
    printf("POS_ERROR: %.3fm\r\n", lla2err(rover_sol.lla, lla0));

    printf("\r\nBASE_ROVER_DIS: %.3fm\r\n", lla2err(lla0, rtcm1005_lla));

    // nl_gnss_sol2str(log, &gnss_sol);
    // printf("GNSS_SPP: %s\r\n", log);

    return 0;
}

