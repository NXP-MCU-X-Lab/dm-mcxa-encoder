#include "nl.h"

/* satellite number to satellite system ----------------------------------------
 * convert satellite number to satellite system
 * args   : int    sat       I   satellite number (1-MAXSAT)
 *          int    *prn      IO  satellite prn/slot number (NULL: no output)
 * return : satellite system (SYS_GPS,SYS_GLO,...)
 *-----------------------------------------------------------------------------*/
int sat_sys(int sat)
{
    int sys = SYS_NONE;
    if (sat <= 0 || MAXSAT < sat)
        sat = 0;
    else if (sat <= NSATGPS)
    {
        sys = SYS_GPS;
        sat += MINPRNGPS - 1;
    }
    else if ((sat -= NSATGPS) <= NSATGLO)
    {
        sys = SYS_GLO;
        sat += MINPRNGLO - 1;
    }
    else if ((sat -= NSATGLO) <= NSATGAL)
    {
        sys = SYS_GAL;
        sat += MINPRNGAL - 1;
    }
    else if ((sat -= NSATGAL) <= NSATQZS)
    {
        sys = SYS_QZS;
        sat += MINPRNQZS - 1;
    }
    else if ((sat -= NSATQZS) <= NSATCMP)
    {
        sys = SYS_CMP;
        sat += MINPRNCMP - 1;
    }
    else if ((sat -= NSATCMP) <= NSATIRN)
    {
        sys = SYS_IRN;
        sat += MINPRNIRN - 1;
    }
    else if ((sat -= NSATIRN) <= NSATLEO)
    {
        sys = SYS_LEO;
        sat += MINPRNLEO - 1;
    }
    else if ((sat -= NSATLEO) <= NSATSBS)
    {
        sys = SYS_SBS;
        sat += MINPRNSBS - 1;
    }
    else
        sat = 0;
    return sys;
}

/* test satellite system (m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN) ---------*/
static int test_sys(int sys, int m)
{
    switch (sys) {
        case SYS_GPS: return m==0;
        case SYS_SBS: return m==0;
        case SYS_GLO: return m==1;
        case SYS_GAL: return m==2;
        case SYS_CMP: return m==3;
        case SYS_QZS: return m==4;
        case SYS_IRN: return m==5;
    }
    return 0;
}

void sat_pvt(obs_t *obs, nav_t *nav, gnss_sat_t *sat)
{
    sat->obs_n = obs->n;
    satposs(obs->data[0].time, obs->data, sat->obs_n, nav, EPHOPT_BRDC, sat->pv, sat->dts, sat->var, sat->svh);
}

void sat_vec_azel(gnss_sol_t *sol, gnss_sat_t *sat)
{
    for (size_t i = 0; i < sat->obs_n; i++)
    {
        if (sat->pv[i].p[0] == 0.0 &&
            sat->pv[i].p[1] == 0.0 &&
            sat->pv[i].p[2] == 0.0 &&
            sat->pv[i].v[0] == 0.0 &&
            sat->pv[i].v[1] == 0.0 &&
            sat->pv[i].v[2] == 0.0)
        {
            sat->eph[i] = -1;
        }
        else
        {
            /* e: receiver-to-satellilte unit vevtor */
            v3sub(sat->e + i, sat->pv[i].p, sol->pos_ecef);
            sat->dist[i] = v3norm(sat->e + i);
            if (sat->dist[i] != 0)
            {
                v3scale2(sat->e + i, 1 / sat->dist[i]);
            }

            /* Satellilte geometric distance includes sagnac effect correction */
            sat->dist[i] += OMEGA_E * (sat->pv[i].p[0] * sol->pos_ecef[1] - sat->pv[i].p[1] * sol->pos_ecef[0]) / CLIGHT;

            /* Azimuth/Elevation angle */
            if (sol->lla[2] > -RE_WGS84)
            {
                nl_t enu[3];
                ecef2enu(sol->lla, sat->e + i, enu);
                sat->az[i] = v3dot(enu, enu) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
                if (sat->az[i] < 0.0)
                {
                    sat->az[i] += 2 * PI;
                }
                sat->el[i] = asin(enu[2]);
            }
            else
            {
                sat->az[i] = 0.0;
                sat->el[i] = PI / 2;
            }
        }
    }
}

/* Satellite ionospheric correction */
void sat_iono_corr(gtime_t time, const nav_t *nav, const double *lla, nl_t az, nl_t el, double *dion, double *vion)
{
    const double ion_default[] = {/* 2004/1/1 */
                                  0.1118E-07, -0.7451E-08, -0.5961E-07, 0.1192E-06,
                                  0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07};
    double tt, f, psi, phi, lam, amp, per, x;
    double *nav_ion = nav->ion_gps;
    int week;

    if (lla[2] < -1E3 || el <= 0)
    {
        *dion = 0.0;
        *vion = 0.0;
        return;
    }

    if (vnorm(nav_ion, 8) <= 0.0)
        nav_ion = ion_default;

    /* earth centered angle (semi-circle) */
    psi = 0.0137 / (el / PI + 0.11) - 0.022;

    /* subionospheric latitude/longitude (semi-circle) */
    phi = lla[0] / PI + psi * cos(az);
    if (phi > 0.416)
        phi = 0.416;
    else if (phi < -0.416)
        phi = -0.416;
    lam = lla[1] / PI + psi * sin(az) / cos(phi * PI);

    /* geomagnetic latitude (semi-circle) */
    phi += 0.064 * cos((lam - 1.617) * PI);

    /* local time (s) */
    tt = 43200.0 * lam + time2gpst(time, &week);
    tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

    /* slant factor */
    f = 1.0 + 16.0 * pow(0.53 - el / PI, 3.0);

    /* ionospheric delay */
    amp = nav_ion[0] + phi * (nav_ion[1] + phi * (nav_ion[2] + phi * nav_ion[3]));
    per = nav_ion[4] + phi * (nav_ion[5] + phi * (nav_ion[6] + phi * nav_ion[7]));
    amp = amp < 0.0 ? 0.0 : amp;
    per = per < 72000.0 ? 72000.0 : per;
    x = 2.0 * PI * (tt - 50400.0) / per;

    *dion = CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
    *vion = POW2(*dion * ERR_BRDCI);
}

/* Satellite tropospheric correction */
void sat_trop_corr(const double *lla, nl_t el, double *dtrp, double *vtrp)
{
    const double temp0 = 15.0; /* temparature at sea level */
    double hgt, pres, temp, e, z, trph, trpw;

    if (lla[2] < -100.0 || 1E4 < lla[2] || el <= 0)
    {
        *dtrp = 0.0;
        *vtrp = 0.0;
        return;
    }

    /* standard atmosphere */
    hgt = lla[2] < 0.0 ? 0.0 : lla[2];

    pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
    temp = temp0 - 6.5E-3 * hgt + 273.16;
    e = 6.108 * REL_HUMI * exp((17.15 * temp - 4684.0) / (temp - 38.45));

    /* saastamoninen model */
    z = PI / 2.0 - el;
    trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * lla[0]) - 0.00028 * hgt / 1E3) / cos(z);
    trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);

    *dtrp = trph + trpw;
    *vtrp = POW2(ERR_SAAS / (sin(el) + 0.1));
}

/* get group delay parameter (m) */
nl_t sat_gettgd(int sat, const nav_t *nav, int type)
{
    int i, sys = satsys(sat, NULL);

    for (i = 0; i < nav->n; i++)
    {
        if (nav->eph[i].sat == sat)
            break;
    }
    return (i >= nav->n) ? 0.0 : nav->eph[i].tgd[type] * CLIGHT;
}

/* iono-free or "pseudo iono-free" pseudorange with code bias correction */
nl_t sat_prange(const obsd_t *obs, const nav_t *nav, nl_t *var)
{
    double P1, P2, gamma, b1, b2;
    int sat, sys, f2;

    sat = obs->sat;
    sys = satsys(sat, NULL);
    P1 = obs->P[0];
    P2 = obs->P[1];

    *var = POW2(ERR_CBIAS);
    if (sys == SYS_GPS || sys == SYS_QZS) /* L1 */
    {
        b1 = sat_gettgd(sat, nav, 0); /* TGD (m) */
        return P1 - b1;
    }
    else if (sys == SYS_CMP)
    { /* B1I/B1Cp/B1Cd */
        if (obs->code[0] == CODE_L2I)
            b1 = sat_gettgd(sat, nav, 0); /* TGD_B1I */
        else if (obs->code[0] == CODE_L1P)
            b1 = sat_gettgd(sat, nav, 2); /* TGD_B1Cp */
        else
            b1 = sat_gettgd(sat, nav, 2) + sat_gettgd(sat, nav, 4); /* TGD_B1Cp+ISC_B1Cd */
        return P1 - b1;
    }

    return P1;
}

/* GNSS solution dops */
void sat_dops(gnss_sat_t *sat, gnss_sol_t *sol)
{
    nl_t h[MAX_OBS_NS][4], q[4][4];
    m_t H, Q;

    minit(&H, sat->spp_n, 4, h);
    minit(&Q, 4, 4, q);

    int i, n;
    for (i = n = 0; i < sat->spp_n && i < MAX_OBS_NS; i++)
    {
        int sat_idx = sat->spp_idx[i];

        nl_t sinel = sin(sat->el[sat_idx]);
        nl_t cosel = cos(sat->el[sat_idx]);
        nl_t sinaz = sin(sat->az[sat_idx]);
        nl_t cosaz = cos(sat->az[sat_idx]);

        h[i][0] = -cosel * sinaz;
        h[i][1] = -cosel * cosaz;
        h[i][2] = -sinel;
        h[i][3] = 1.0;
    }

    mmul3("TN", 1, &H, &H, 0, &Q);

    if (!minv(&Q))
    {
        sol->gdop = SAFE_SQRT(q[0][0] + q[1][1] + q[2][2] + q[3][3]); /* GDOP */
        sol->pdop = SAFE_SQRT(q[0][0] + q[1][1] + q[2][2]);           /* PDOP */
        sol->hdop = SAFE_SQRT(q[0][0] + q[1][1]);                     /* HDOP */
        sol->vdop = SAFE_SQRT(q[2][2]);                               /* VDOP */
    }
}

typedef struct
{
    nl_t P[GNSS_FREQ_NUM]; /* zero diff residuals [range - measured pseudorange] */
    nl_t L[GNSS_FREQ_NUM]; /* zero diff residuals [phase - measured carrier phase] */
} sat_zdres_t;

typedef struct
{
    sat_zdres_t sys[GNSS_SYS_NUM];
};



/* zero diff residuals [range - measured pseudorange] for rover (phase and code) */
void sat_zdres(obs_t *obs, nav_t *nav, gnss_sat_t *sat, gnss_sol_t *sol, nl_t zdres[])
{
    for (size_t i = 0; i < sat->rtk_n; i++)
    {
        int sat_idx = sat->rtk_idx[i];

        nl_t p, p_var;
        // p = sat_prange(obs->data + sat_idx, nav, &p_var);
        p = sat->dist[sat_idx];
        p += -CLIGHT * sat->dts[sat_idx].bias;

        nl_t zhd, zhd_var;
        sat_trop_corr(sol->lla, 90 * D2R, &zhd, &zhd_var);

        nl_t azel[2];
        azel[0] = sat->az[sat_idx];
        azel[1] = sat->el[sat_idx];
        nl_t mapfh = tropmapf(obs->data[i].time, sol->lla, azel, NULL);

        p += mapfh * zhd;

        for (size_t j = 0; j < GNSS_FREQ_NUM; j++)
        {
            /* extract frequency and wavelength from obs code */
            sat->freq[i][j] = sat2freq(obs->data[sat_idx].sat, obs->data[sat_idx].code[j], nav);
            sat->lambda[i][j] = CLIGHT / sat->freq[i][j];

            /* skip if no measurement */
            if (0.0 == sat->freq[i][j]) continue;

            /* zdres: [L1, P1, L5, P5] */
            if (obs->data[sat_idx].L[j] != 0.0)
            {
                zdres[i*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM] = obs->data[sat_idx].L[j] * sat->lambda[i][j] - p;
            }

            if (obs->data[sat_idx].P[j] != 0.0)
            {
                zdres[i*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM + 1] = obs->data[sat_idx].P[j] - p;
            }

            // printf("sat_idx:%d, p:%f, L:%f, zdres1:%f, zdres2:%f\n", sat_idx, obs->data[sat_idx].P[j], obs->data[sat_idx].L[j] * CLIGHT / freq, zdres[i*2+j], zdres[i*2+1+j]);
            // printf("zdres1:%f zdres2:%f\n", zdres[i*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM], zdres[i*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM + 1]);
        }
    }
}

/* double-differenced residuals and create state matrix from sat angles  */
void sat_ddres(obs_t *obs, nav_t *nav, gnss_sat_t *sat, gnss_sol_t *sol, nl_t rover_zdres[], nl_t base_zdres[], nl_t ddres[], nl_t H[], size_t *n)
{
    int gnss_sys[GNSS_SYS_NUM]={SYS_GPS, SYS_CMP}; //only GPS and BDS

    size_t ddres_idx = 0;

    for (size_t i = 0; i < GNSS_SYS_NUM; i++)
    {
        for (size_t j = 0; j < GNSS_FREQ_NUM; j++)
        {
            int high_idx = -1, high_k = -1;

            for (size_t k = 0; k < sat->rtk_n; k++)
            {
                int sat_idx = sat->rtk_idx[k];

                /* skip sat if system is not the current system */
                if (satsys(obs->data[sat_idx].sat, NULL) != gnss_sys[i]) continue;

                /* skip sat with slip unless no other valid sat */
                if (high_idx >= 0 && obs->data[sat_idx].LLI[j] & LLI_SLIP) continue;

                /* skip sat if no code or phase measurement */
                if (obs->data[sat_idx].L[j] == 0.0 || obs->data[sat_idx].P[j] == 0.0) continue;

                /* select highest elevation sat */
                if (high_idx < 0 || sat->el[sat_idx] > sat->el[high_idx])
                {
                    high_idx = sat_idx;
                    high_k = k;
                }
            }

            // if the current frequency of the current system does not find the highest sat, continue
            if (high_idx < 0) continue;

            NL_DEBUG_LOG(NL_DEBUG_GNSS, ("sys:%d, freq:%d, high_idx:%d, el:%.2f\n", i, j, high_idx, sat->el[high_idx] * R2D));

            /* calculate double differences of residuals (code/phase) for each sat */
            for (size_t k = 0; k < sat->rtk_n; k++)
            {
                int sat_idx = sat->rtk_idx[k];

                if (sat_idx == high_idx) continue;

                if (satsys(obs->data[sat_idx].sat, NULL) != gnss_sys[i]) continue;

                
                if (rover_zdres[k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM] == 0.0 || rover_zdres[high_k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM] == 0.0 ||
                    base_zdres[k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM] == 0.0 || base_zdres[high_k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM] == 0.0
                ) continue;
                
                // calculate the double difference of the code and phase residual of the current sat and the highest sat
                /* carrier phase double difference */
                double cp_sd_sat = rover_zdres[k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM] - base_zdres[k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM];
                double cp_sd_ref = rover_zdres[high_k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM] - base_zdres[high_k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM];
                ddres[ddres_idx] = cp_sd_ref - cp_sd_sat;
                // ddres[ddres_idx] /= sat->lambda[k][j];
                v3sub(H + ddres_idx*(MAX_SAT_NUM*GNSS_FREQ_NUM+3), sat->e + high_idx, sat->e + sat_idx);

                int x_sat_idx = 3 + MAX_SAT_NUM * j + sat_idx - 1;
                int x_ref_idx = 3 + MAX_SAT_NUM * j + high_idx - 1;

                H[x_sat_idx] = -sat->lambda[k][j];
                H[x_ref_idx] = sat->lambda[high_k][j];

                ddres_idx++;

                /* pseudorange double difference */
                double pr_sd_sat = rover_zdres[k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM + 1] - base_zdres[k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM + 1];
                double pr_sd_ref = rover_zdres[high_k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM + 1] - base_zdres[high_k*GNSS_FREQ_NUM*2 + j*GNSS_FREQ_NUM + 1];
                ddres[ddres_idx] = pr_sd_ref - pr_sd_sat;
                v3sub(H + ddres_idx*MAX_SAT_NUM*GNSS_FREQ_NUM, sat->e + high_idx, sat->e + sat_idx);

                ddres_idx++;
            }
        }
    }
    
    *n = ddres_idx;

    vprint(ddres, ddres_idx, 10);
    printf("ddres_idx:%d\n", ddres_idx);
}

void gnss_spp(obs_t *obs, nav_t *nav, gnss_sat_t *sat, gnss_sol_t *sol)
{
    sat_pvt(obs, nav, sat);

    m_t *x = mcreate(3 + GNSS_SYS_NUM, 1);
    m_t *dx = mcreate(3 + GNSS_SYS_NUM, 1);
    m_t *Q = mcreate(3 + GNSS_SYS_NUM, 3 + GNSS_SYS_NUM);
    mfilldiag(Q, 1);
    v3copy(M2V(x), sol->pos_ecef);
    ecef2lla(sol->pos_ecef, sol->lla);

    for (size_t i = 0; i < MAX_SPP_ITER; i++)
    {
        sat->spp_n = 0;

        sat_vec_azel(sol, sat);

        for (size_t j = 0; j < sat->obs_n; j++)
        {
            if (sat->eph[j] == -1 ||                      /* No ephemeris */
                sat->svh[j] != 0 ||                       /* Ephemeris unavailable or unhealty satellite */
                sat->var[j] > MAX_EPH_VAR ||              /* Limit variance of ephemeris */
                sat->el[j] < MIN_EL ||                    /* Limit elevation of satellite */
                obs->data[j].SNR[0] * SNR_UNIT < MIN_SNR) /* Limit SNR of satellite */
            {
                continue;
            }

            sat->spp_idx[sat->spp_n++] = j;
        }

        if (sat->spp_n < MIN_SOL_NS)
        {
            NL_DEBUG_LOG(NL_DEBUG_GNSS, ("lack of valid sats ns=%d\r\n", sat->spp_n));
            break;
        }

        m_t *z = mcreate(sat->spp_n, 1);
        m_t *H = mcreate(sat->spp_n, 3 + GNSS_SYS_NUM);
        for (size_t j = 0; j < sat->spp_n; j++)
        {
            int sat_idx = sat->spp_idx[j];

            nl_t dtr_gps = MELEMENT(x, 3, 0);
            nl_t dtr_bds = MELEMENT(x, 4, 0);

            nl_t freq;
            nl_t dion, vion;
            nl_t dtrp, vtrp;
            sat_iono_corr(obs->data[sat_idx].time, nav, sol->lla, sat->az[sat_idx], sat->el[sat_idx], &dion, &vion);
            if ((freq = sat2freq(obs->data[sat_idx].sat, obs->data[sat_idx].code[0], nav)) != 0.0)
            {
                dion *= POW2(FREQL1 / freq);
                vion *= POW2(FREQL1 / freq);
            }
            sat_trop_corr(sol->lla, sat->el[sat_idx], &dtrp, &vtrp);

            nl_t p, p_var;
            p = obs->data[sat_idx].P[0];
            p = sat_prange(obs->data + sat_idx, nav, &p_var);

            MELEMENT(z, j, 0) = p - (sat->dist[sat_idx] + dtr_gps - CLIGHT * sat->dts[sat_idx].bias + dion + dtrp);
            MELEMENT(H, j, 3) = 1;

            if (satsys(obs->data[sat_idx].sat, NULL) == SYS_CMP)
            {
                MELEMENT(z, j, 0) -= dtr_bds;
                MELEMENT(H, j, 4) = 1;
            }

            nl_t ee[3];
            v3scale(ee, (sat->e + sat_idx), -1);
            v3copy(M2R(H, j), ee);
        }

        nl_lsq(H, M2V(z), M2V(dx), Q);
        vadd2(M2V(x), M2V(dx), 3 + GNSS_SYS_NUM);

        v3copy(sol->pos_ecef, M2V(x));
        ecef2lla(sol->pos_ecef, sol->lla);

        nl_free(z);
        nl_free(H);

        if (vdot(M2V(dx), M2V(dx), 3 + GNSS_SYS_NUM) < 1e-3)
        {
            sat_dops(sat, sol);
            sol->solq = SOLQ_SINGLE;
            break;
        }
    }

    nl_free(Q);
    nl_free(x);
    nl_free(dx);
}

void gnss_rtk(obs_t *obs_rover, obs_t *obs_base, nav_t *nav)
{
    gnss_sat_t sat_rover = {0}, sat_base = {0};
    gnss_sol_t sol_rover = {0}, sol_base = {0};

    gnss_spp(obs_rover, nav, &sat_rover, &sol_rover);
    gnss_spp(obs_base, nav, &sat_base, &sol_base);

    NL_DEBUG_LOG(NL_DEBUG_GNSS, ("rover: %14.7f, %14.7f, %.3f\r\n", sol_rover.lla[0] * R2D, sol_rover.lla[1] * R2D, sol_rover.lla[2]));
    NL_DEBUG_LOG(NL_DEBUG_GNSS, ("base:  %14.7f, %14.7f, %.3f\r\n", sol_base.lla[0] * R2D, sol_base.lla[1] * R2D, sol_base.lla[2]));

    /* select common satellites */
    NL_DEBUG_LOG(NL_DEBUG_GNSS, ("\r\ncommon satellites:\r\n"));
    int sat_cn = 0, sat_id[MAX_OBS_NS] = {0};
    sat_rover.rtk_n = sat_base.rtk_n = 0;
    for (size_t i = 0; i < sat_rover.spp_n; i++)
    {
        int rover_idx = sat_rover.spp_idx[i];
        for (size_t j = 0; j < sat_base.spp_n; j++)
        {
            int base_idx = sat_base.spp_idx[j];
            if (obs_rover->data[rover_idx].sat == obs_base->data[base_idx].sat)
            {
                sat_id[sat_cn++] = obs_rover->data[rover_idx].sat;
                sat_rover.rtk_idx[sat_rover.rtk_n++] = rover_idx;
                sat_base.rtk_idx[sat_base.rtk_n++] = base_idx;
                NL_DEBUG_LOG(NL_DEBUG_GNSS, ("sat_id=%d, sat_idx_rover=%d, sat_idx_base=%d\r\n", sat_id[sat_cn - 1], sat_rover.rtk_idx[sat_cn - 1], sat_base.rtk_idx[sat_cn - 1]));
                break;
            }
        }
    }

    printf("sat_cn=%d, ", sat_cn);
    printf("sat_rover.rtk_n=%d, ", sat_rover.rtk_n);
    printf("sat_base.rtk_n=%d\r\n", sat_base.rtk_n);

    if (sat_cn < MIN_SOL_NS)
    {
        NL_DEBUG_LOG(NL_DEBUG_GNSS, ("lack of common sats ns=%d\r\n", sat_cn));
        return;
    }

    nl_t base_zdres[MAX_OBS_NS * 4] = {0};
    nl_t rover_zdres[MAX_OBS_NS * 4] = {0};
    sat_zdres(obs_base, nav, &sat_base, &sol_base, base_zdres);
    sat_zdres(obs_rover, nav, &sat_rover, &sol_rover, rover_zdres);

    NL_DEBUG_LOG(NL_DEBUG_GNSS, ("\r\nbase_zdres:\r\n"));
    vprint(base_zdres, sat_cn * 4, 10);
    NL_DEBUG_LOG(NL_DEBUG_GNSS, ("\r\nrover_zdres\r\n"));
    vprint(rover_zdres, sat_cn * 4, 10);

    size_t ddn = 0;
    nl_t ddres[MAX_OBS_NS * 4] = {0};
    nl_t H[KF_STATE_NUM][KF_STATE_NUM] = {0};
    sat_ddres(obs_rover, nav, &sat_rover, &sol_rover, rover_zdres, base_zdres, ddres, H, &ddn);

    // kf_state_t kf_s;
    // kf_meas_t kf_m;
    // kf_state_create(&kf_s, 3+MAX_SAT_NUM*GNSS_FREQ_NUM);
    // kf_meas_create(&kf_m, 3+MAX_SAT_NUM*GNSS_FREQ_NUM, ddn);
    // vcopy(kf_s.X, sol_rover.pos_ecef, 3);
    // vcopy(kf_m.Z, ddres, ddn);
    // mcopy(kf_m.H, H, ddn, 3+MAX_SAT_NUM*GNSS_FREQ_NUM);

    // kf_meas_update(&kf_s, &kf_m);

    // kf_state_free(&kf_s);
    // kf_meas_free(&kf_m);

    
}
