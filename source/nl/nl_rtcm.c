#include "nl.h"

#define RTCM3PREAMB 0xD3            /* rtcm ver.3 frame preamble */
#define PRUNIT_GPS  299792.458      /* rtcm ver.3 unit of gps pseudorange (m) */
#define PRUNIT_GLO  599584.916      /* rtcm ver.3 unit of glonass pseudorange (m) */
#define RANGE_MS    (CLIGHT*0.001)  /* range in 1 ms */

#define P2_10       0.0009765625          /* 2^-10 */
#define P2_28       3.725290298461914E-09 /* 2^-28 */
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_41       4.547473508864641E-13 /* 2^-41 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define P2_66       1.355252715606880E-20 /* 2^-66 */



/* MSM signal ID table -------------------------------------------------------*/
static const char *msm_sig_gps[32]=
{
    /* GPS: ref [17] table 3.5-91 */
    ""  ,"1C","1P","1W",""  ,""  ,""  ,"2C","2P","2W",""  ,""  , /*  1-12 */
    ""  ,""  ,"2S","2L","2X",""  ,""  ,""  ,""  ,"5I","5Q","5X", /* 13-24 */
    ""  ,""  ,""  ,""  ,""  ,"1S","1L","1X"                      /* 25-32 */
};

static const char *msm_sig_cmp[32]= /* RINEX obs code */
{
    /* BeiDou: ref [17] table 3.5-108 */
    ""  ,"2I","2Q","2X",""  ,""  ,""  ,"6I","6Q","6X",""  ,""  ,/*1-12*/
    ""  ,"7I","7Q","7X",""  ,""  ,""  ,""  ,""  ,"5D","5P","5X",/*13-24*/
    "7D",""  ,""  ,""  ,""  ,"1D"  ,"1P"  ,"1X"/*25-32*/
};

typedef struct /* multi-signal-message header type */
{
    uint8_t nsat, nsig;   /* number of satellites/signals */
    uint8_t sats[64];     /* satellites */
    uint8_t sigs[32];     /* signals */
    uint8_t cellmask[64]; /* cell mask */
} msm_h_t;


static double getbits_38(const uint8_t *buff, int pos)
{
    return (double)getbits(buff,pos,32)*64.0+getbitu(buff,pos+32,6);
}

/* adjust weekly rollover of GPS time ----------------------------------------*/
static void adjweek(rtcm_t *rtcm, double tow)
{
    double tow_p = 0;
    int week;

    /* if no time, get cpu time */
    if (rtcm->time.time == 0) rtcm->time = utc2gpst(timeget());
    tow_p = time2gpst(rtcm->time, &week);
    if (tow < tow_p - 302400.0) tow += 604800.0;
    else if (tow > tow_p + 302400.0) tow -= 604800.0;
    rtcm->time = gpst2time(week, tow);
}

static int nl_decode_msm_head(rtcm_t *rtcm, int sys, int *sync, msm_h_t *h, int *hsize)
{
    msm_h_t h0 = {0};
    double tow;
    char  tstr[64];
    int i = 24, j, mask, staid, type, ncell = 0;

    type = getbitu(rtcm->buff, i, 12);
    i += 12;

    *h = h0;
    if (i + 157 <= rtcm->len * 8)
    {
        staid = getbitu(rtcm->buff, i, 12);
        i += 12;
        if (sys == SYS_CMP)
        {
            tow = getbitu(rtcm->buff, i, 30) * 0.001;
            i += 30;
            tow += 14.0; /* BDT -> GPST */
            adjweek(rtcm, tow);
        }
        else
        {
            tow = getbitu(rtcm->buff, i, 30) * 0.001;
            i += 30;
            adjweek(rtcm, tow);
        }

        *sync = getbitu(rtcm->buff, i, 1); /* DF393: 1:more msg, 0: last msg */
        i += (1+3+7+2+2+1+3);

        for (j = 1; j <= 64; j++)
        {
            mask = getbitu(rtcm->buff, i, 1);
            i += 1;
            if (mask) h->sats[h->nsat++] = j;

        }

        for (j = 1; j <= 32; j++)
        {
            mask = getbitu(rtcm->buff, i, 1);
            i += 1;
            if (mask) h->sigs[h->nsig++] = j;
        }
    }
    else
    {
        NL_DEBUG_LOG(NL_DEBUG_RTCM, ("rtcm3 %d length error: len=%d\n", type, rtcm->len));
        return -1;
    }

    if (h->nsat * h->nsig > 64) return -1; /* rtcm3 %d number of sats and sigs error: nsat=%d nsig=%d\ */
    if (i + h->nsat * h->nsig > rtcm->len * 8) return -1; /* rtcm3 %d length error: len=%d nsat=%d nsig=%d\n */

    for (j = 0; j < h->nsat * h->nsig; j++)
    {
        h->cellmask[j] = getbitu(rtcm->buff, i, 1);
        i += 1;
        if (h->cellmask[j]) ncell++;
    }
    *hsize = i;

    time2str(rtcm->time, tstr, 2);
    NL_DEBUG_LOG(NL_DEBUG_RTCM, ("decode_head_msm: time=%s sys=%d staid=%d nsat=%d nsig=%d sync=%d ncell=%d\n", tstr, sys, staid, h->nsat, h->nsig, *sync, ncell));
    
    return ncell;
}

static int obsindex(obs_t *obs, nl_gtime_t time, int sat)
{
    int i,j;

    for (i=0;i<obs->n;i++)
    {
        if (obs->data[i].sat==sat) return i; /* field already exists */
    }
    if (i>=MAXOBS) return -1; /* overflow */

    /* add new field */
    obs->data[i].time=time;
    obs->data[i].sat=sat;
    for (j=0;j<NFREQ+0;j++)
    {
        obs->data[i].L[j]=obs->data[i].P[j]=0.0;
        obs->data[i].D[j]=0.0;
        obs->data[i].SNR[j]=obs->data[i].LLI[j]=obs->data[i].code[j]=0;
    }
    obs->n++;
    return i;
}

/* get signal index ----------------------------------------------------------*/
static void sigindex(int sys, const uint8_t *code, int n, const char *opt, int *idx)

{
    int i, nex, pri, pri_h[8]={0}, index[8]={0}, ex[32]={0};

    /* test code priority */
    for (i=0;i<n;i++)
    {
        if (!code[i]) continue;

        if (idx[i]>=NFREQ)
        { /* save as extended signal if idx >= NFREQ */
            ex[i]=1;
            continue;
        }
        /* code priority */
        pri = getcodepri(sys,code[i], opt);

        /* select highest priority signal */
        if (pri>pri_h[idx[i]])
        {
            if (index[idx[i]]) ex[index[idx[i]]-1]=1;
            pri_h[idx[i]]=pri;
            index[idx[i]]=i+1;
        }
        else ex[i]=1;
    }
    /* signal index in obs data */
    for (i=nex=0;i<n;i++)
    {
        if (ex[i]==0) ;
        else if (nex<0) idx[i]=NFREQ+nex++;
        else { /* no space in obs data */
            NL_DEBUG_LOG(NL_DEBUG_RTCM, ("rtcm msm: no space in obs data sys=%d code=%d\n",sys,code[i]));
            idx[i]=-1;
        }
#if 0 /* for debug */
        NL_DEBUG_LOG(NL_DEBUG_RTCM, ("sig pos: sys=%d code=%d ex=%d idx=%d\n",sys,code[i],ex[i],idx[i]));
#endif
    }
}

/* loss-of-lock indicator ----------------------------------------------------*/
static int lossoflock(rtcm_t *rtcm, int sat, int idx, int lock)
{
    int lli=(!lock&&!rtcm->lock[sat-1][idx])||lock<rtcm->lock[sat-1][idx];
    rtcm->lock[sat-1][idx]=(uint16_t)lock;
    return lli;
}

static void nl_save_msm_obs(rtcm_t *rtcm, int sys, msm_h_t *h, const double *r, const double *pr, const double *cp, const double *rr, const double *rrf, const double *cnr, const int *lock, const int *ex, const int *half)
{
    const char *sig[32];
    double tt, freq;
    uint8_t code[32];
    char *msm_type = "", *q = NULL;
    int i, j, k, type, prn, sat, fcn, index=0, idx[32];

    type = getbitu(rtcm->buff, 24, 12);

    switch (sys)
    {
        case SYS_GPS:
            msm_type = q = rtcm->msmtype[0];
            break;
        case SYS_CMP:
            msm_type = q = rtcm->msmtype[5];
            break;
    }

    /* id to signal */
    for (i=0; i<h->nsig; i++)
    {
        switch (sys)
        {
           case SYS_GPS: sig[i] = msm_sig_gps[h->sigs[i]-1]; break;
           case SYS_CMP: sig[i] = msm_sig_cmp[h->sigs[i]-1]; break;
           default:  sig[i] = ""; break;
        }

        /* signal to rinex obs type */
        code[i] = obs2code(sig[i]);
        idx[i] = code2idx(sys, code[i]);

        if (code[i] != CODE_NONE)
        {
            if (q) q += sprintf(q,"L%s%s", sig[i], i<h->nsig-1?",":"");
        }
        else
        {
            if (q) q += sprintf(q,"(%d)%s", h->sigs[i], i<h->nsig-1?",":"");
            NL_DEBUG_LOG(NL_DEBUG_RTCM, ("rtcm3 %d: unknown signal id=%2d\n", type, h->sigs[i]));
        }
    }

    NL_DEBUG_LOG(NL_DEBUG_RTCM, ("rtcm3 %d: signals=%s\n",type, msm_type));

    /* get signal index */
     sigindex(sys, code, h->nsig, rtcm->opt, idx);

    for (i=j=0; i<h->nsat; i++)
    {
        prn = h->sats[i];
        if      (sys == SYS_QZS) prn+=MINPRNQZS-1;
        else if (sys == SYS_SBS) prn+=MINPRNSBS-1;

        if ((sat = satno(sys, prn)))
        {
            tt = timediff(rtcm->obs.data[0].time, rtcm->time);
            if (rtcm->obsflag||fabs(tt)>1E-9)
            {
                rtcm->obs.n=rtcm->obsflag=0;
            }
              index = obsindex(&rtcm->obs, rtcm->time, sat);
        }
        else
        {
            NL_DEBUG_LOG(NL_DEBUG_RTCM, ("rtcm3 %d satellite error: prn=%d\n",type,prn));
        }
        fcn=0;
        for (k=0; k<h->nsig; k++)
        {
            if (!h->cellmask[k+i*h->nsig]) continue;

            if (sat && index>=0 && idx[k]>=0)
            {
                freq=fcn<-7?0.0:code2freq(sys,code[k],fcn);

                /* pseudorange (m) */
                if (r[i]!=0.0&&pr[j]>-1E12)
                {
                    rtcm->obs.data[index].P[idx[k]] = r[i]+pr[j];
                }

                /* carrier-phase (cycle) */
                if (r[i]!=0.0&&cp[j]>-1E12)
                {
                    rtcm->obs.data[index].L[idx[k]] = (r[i]+cp[j])*freq/CLIGHT;
                //    printf("k:%d, index:%d, idx[k]:%d, L:%f\r\n", k, index, idx[k], rtcm->obs.data[index].L[idx[k]]);
                }

                /* doppler (hz) */
                if (rr&&rrf&&rrf[j]>-1E12)
                {
                    rtcm->obs.data[index].D[idx[k]] = (float)(-(rr[i]+rrf[j])*freq/CLIGHT);
                }

                rtcm->obs.data[index].LLI[idx[k]] = lossoflock(rtcm, sat, idx[k], lock[j])+(half[j]?2:0);
                rtcm->obs.data[index].SNR [idx[k]]=(uint16_t)(cnr[j]/SNR_UNIT+0.5);
                rtcm->obs.data[index].code[idx[k]] = code[k];

            }
            j++;
        }
    }
}


static int nl_decode_msm4(rtcm_t *rtcm, int sys)
{
    NL_DEBUG_LOG(NL_DEBUG_RTCM, ("decode_msm4, sys:%d\r\n", sys));
    msm_h_t h = {0};
    double r[64], pr[64], cp[64], cnr[64];     /* r: range, pr:GNSS signal fine Pseudoranges, cp: phaserange */
    int i, j, sync, ncell, rng, rng_m, prv, cpv, lock[64], half[64];

    /* decode msm header */
    if ((ncell = nl_decode_msm_head(rtcm, sys, &sync, &h, &i)) < 0) return -1;
    if (i+h.nsat*18+ncell*48>rtcm->len*8) return -1; /* rtcm3 %d length error: */

    for (j=0; j<h.nsat; j++) r[j] = 0.0;
    for (j=0; j<ncell; j++) pr[j] = cp[j] = -1E16;

    /* decode satellite data */
    for (j=0; j<h.nsat; j++)    /* DF397: The number of integer milliseconds in GNSS Satellite rough ranges */
    {
        rng  = getbitu(rtcm->buff, i, 8);    /* range */
        i += 8;
        if (rng!=255) r[j] = rng*RANGE_MS;
    }

    for (j=0; j<h.nsat; j++)    /* DF398: GNSS Satellite rough ranges modulo 1 millisecond */
    {
        rng_m = getbitu(rtcm->buff, i, 10);
        i += 10;
        if (r[j]!=0.0) r[j] += rng_m*P2_10*RANGE_MS;
    }

    /* decode signal data */
    for (j=0; j<ncell; j++) /* DF400: signal fine Pseudorange */
    {
        prv = getbits(rtcm->buff, i, 15);
        i+=15;
        if (prv!=-16384) pr[j] = prv*P2_24*RANGE_MS;
    }

    for (j=0; j<ncell; j++) /* DF401: signal fine Phaserange data */
    {
        cpv=getbits(rtcm->buff, i, 22);
        i+=22;
        if (cpv!=-2097152) cp[j] = cpv*P2_29*RANGE_MS;
    }

    for (j=0; j<ncell; j++) /* DF402: GNSS Phaserange Lock Time Indicator */
    {
        lock[j] = getbitu(rtcm->buff, i, 4);
        i+=4;
    }

    for (j=0; j<ncell; j++) /* DF420: Half-cycle ambiguity indicator */
    {
        half[j] = getbitu(rtcm->buff,i,1);    /* half-cycle ambiguity */
        i+=1;
    }

    for (j=0; j<ncell; j++) /* DF403: GNSS signal CNR */
    {
        cnr[j] = getbitu(rtcm->buff,i,6)*1.0;    /* cnr */
        i+=6;
    }

    /* save obs data in msm message */
    nl_save_msm_obs(rtcm, sys, &h, r, pr, cp, NULL, NULL, cnr, lock, NULL, half);

    rtcm->obsflag = !sync;
    return sync?0:1;
}



/* decode type 1005: stationary RTK reference station ARP --------------------*/
static int nl_decode_type1005(rtcm_t *rtcm)
{
    double rr[3];
    int i = 24+12, j, staid, itrf;

    if (i+140 == rtcm->len*8)
    {
        staid = getbitu(rtcm->buff,i,12);
        i+=12;
        itrf = getbitu(rtcm->buff,i, 6);
        i+= 6+4;
        rr[0] = getbits_38(rtcm->buff,i);
        i+=38+2;
        rr[1] = getbits_38(rtcm->buff,i);
        i+=38+2;
        rr[2] = getbits_38(rtcm->buff,i);
    }
    else
    {
        NL_DEBUG_LOG(NL_DEBUG_RTCM, ("rtcm3 1005 length error: len=%d\r\n",rtcm->len));
        return -1;
    }

    sprintf(rtcm->sta.name,"%04d", staid);
    rtcm->sta.deltype = 0; /* xyz */
    rtcm->staid = staid;
    for (j=0; j<3; j++)
    {
        rtcm->sta.pos[j] = rr[j]*0.0001;
        rtcm->sta.del[j] = 0.0;
    }

    rtcm->sta.hgt = 0.0;
    rtcm->sta.itrf = itrf;

    return 5;
}

static int nl_decode_type1019(rtcm_t *rtcm)
{
    return 1;
}

static int nl_decode_type1042(rtcm_t *rtcm)
{
    return 1;
}

int nl_decode_rtcm3(rtcm_t *rtcm)
{
    int ret = 0;
    int16_t type = getbitu(rtcm->buff, 24, 12);

    switch (type)
    {
    case 1005:
        ret = nl_decode_type1005(rtcm);
        break;
    case 1019: /* GPS Ephemerides */
        ret = nl_decode_type1019(rtcm);
        break;
    case 1042: /* BDS Satellite Ephemeris Data */
        ret = nl_decode_type1042(rtcm);
        break;
    case 1074:  /* Full GPS Pseudoranges and PhaseRanges plus CNR */
        ret = nl_decode_msm4(rtcm, SYS_GPS);
        break;
    case 1124: /* Full BeiDou Pseudoranges and PhaseRanges plus CNR */
        ret = nl_decode_msm4(rtcm, SYS_CMP);
        break;
    default:
        NL_DEBUG_LOG(NL_DEBUG_RTCM, ("unknown type:%d\r\n", type));
        break;
    }

    return ret;
}

int nl_input_rtcm3(rtcm_t *rtcm, uint8_t data)
{
    /* synchronize frame */
    if (rtcm->nbyte == 0)
    {
        if (data != RTCM3PREAMB)
            return 0;
        rtcm->buff[rtcm->nbyte++] = data;
        return 0;
    }
    rtcm->buff[rtcm->nbyte++] = data;

    if (rtcm->nbyte == 2 && (rtcm->buff[1] & 0xFC) != 0)
    {
        rtcm->nbyte = 0;
        return 0;
    }

    if (rtcm->nbyte == 5)
    {
        rtcm->type = getbitu(rtcm->buff, 24, 12);
        if (rtcm->type < 1000 || rtcm->type > 1200)
        {
            // NL_DEBUG_LOG(NL_DEBUG_RTCM, ("rtcm3 type error:%d\r\n", rtcm->type));
            rtcm->nbyte = 0;
            return 0;
        }
    }

    if (rtcm->nbyte == 3) rtcm->len = getbitu(rtcm->buff, 14, 10) + 3; /* length without parity */

    if (rtcm->nbyte < 3 || rtcm->nbyte < rtcm->len + 3) return 0;

    rtcm->nbyte = 0;

    /* check parity */
    if (rtk_crc24q(rtcm->buff, rtcm->len) != getbitu(rtcm->buff, rtcm->len * 8, 24))
    {
        // NL_DEBUG_LOG(NL_DEBUG_RTCM, ("rtcm3 parity error: len=%d 0x%X 0x%X PARM2:0x%X, type:%d\n", rtcm->len, rtk_crc24q(rtcm->buff, rtcm->len), getbitu(rtcm->buff, rtcm->len * 8, 24), rtcm->buff[1], getbitu(rtcm->buff, 24, 12)));
        return 0;
    }

    /* decode rtcm3 message */
    return nl_decode_rtcm3(rtcm);
}
