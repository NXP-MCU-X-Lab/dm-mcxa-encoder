#include "nl.h"

#define NL_DBG_TAG    "nl.log"
#define NL_DBG_LVL    NL_DBG_INFO
#include "nl_log.h"

#define NL_LOG_BUF_SIZE     (256)

void nl_assert(const char *filename, int linenum)
{
    volatile char dummy = 0;
    NL_LOG_RAW("assertion failed at function:%s, line number:%d\r\n", filename, linenum);
    while (dummy == 0)
        ;
}


void vprint(nl_t *A, size_t n, size_t q)
{
    char buf[NL_LOG_BUF_SIZE];
    
    vec2str(buf, NL_LOG_BUF_SIZE, A, n, q);
    NL_LOG_RAW("%s", buf);
}

void v3print(nl_t A[], size_t q)
{
    vprint(A, 3, q);
}

void v3print_scale(nl_t A[], size_t q, nl_t scale)
{
    nl_t tmp[3];
    
    tmp[0] = A[0]*scale;
    tmp[1] = A[1]*scale;
    tmp[2] = A[2]*scale;
    
    v3print(tmp, q);
}

/* print all elements of a matrix A */
void mprint(m_t *A, size_t q)
{
    char buf[NL_LOG_BUF_SIZE];
    
    mat2str(buf, NL_LOG_BUF_SIZE, A, q);
    NL_LOG_RAW("%s", buf);
}

uint32_t nl_check_vec(nl_t *v1, nl_t *v2, uint32_t len, nl_t tol)
{
    nl_t *res = vcreate(len);
    vsub(res, v1, v2, len);
    NL_ASSERT(vnorm(res, len) < tol);
    nl_free(res);
    return 0;
}

uint32_t vec2str(char *buf, uint32_t size, nl_t *A, size_t n, size_t q)
{
    uint32_t len = 0;

    for (size_t i=0; i<n; i++)
    {
        len += snprintf(buf+len, size, " %*.*f\n", NL_PRINT_WIDTH, q, A[i]);
    }

    return len;
}

uint32_t vec2str_scale(char *buf, uint32_t size, nl_t *A, nl_t k, size_t n, size_t q)
{
    uint32_t len = 0;

    for (size_t i=0; i<n; i++)
    {
        len += snprintf(buf+len, size, " %*.*f\n", NL_PRINT_WIDTH, q, A[i]*k);
    }

    return len;
}

uint32_t mat2str(char *buf, uint32_t size, m_t *A, size_t q)
{
    size_t len = 0, written, remaining_size = size, i, j;
    
    for (i=0; i<A->c && remaining_size > 0; i++)
    {
        for (j=0; j<A->r && remaining_size > 0; j++)
        {

            written = snprintf(buf+len, remaining_size, " %*.*f", NL_PRINT_WIDTH, q, MELEMENT(A, i, j));
            len += written;
            remaining_size -= written;
        }

        if (remaining_size > 0)
        {
            written = snprintf(buf+len, remaining_size, "\n");
            len += written;
            remaining_size -= written;
        }
    }
    return len;
}

///* convert eph struct to eph string */
//uint32_t nl_eph2str(char *buf, eph_t *eph, char opt)
//{
//    char toe[32], toc[32], ttr[32], id[32];
//    if (eph->ttr.time == 0)
//    {
//        buf[0] = '\0';
//        return 0;
//    }

//    nl_satno2id(eph->sat, id);

//    if (opt == 'N') /* NORMAL mode */
//    {

//        return sprintf((char*)buf, "%s,%d,%d,%d,%d,%d,%d,%d,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,"
//                            "%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,%.14E,"
//                            "%.14E,%.14E,%.14E,%.14E,%.14E,%d,%d\r\n",
//                       id, eph->iode, eph->iodc, eph->sva,
//                       eph->svh, (int)eph->toe.time,
//                       (int)eph->toc.time, (int)eph->ttr.time,
//                       eph->A, eph->e, eph->i0, eph->OMG0,
//                       eph->omg, eph->M0, eph->deln, eph->OMGd,
//                       eph->idot, eph->crc, eph->crs, eph->cuc,
//                       eph->cus, eph->cic, eph->cis, eph->toes,
//                       eph->fit, eph->f0, eph->f1, eph->f2,
//                       eph->tgd[0], eph->code, eph->flag);
//    }

//    if (opt == 'S') /* SHORT mode */
//    {
//        nl_time2str(eph->toe, toe, 0);
//        nl_time2str(eph->toc, toc, 0);
//        nl_time2str(eph->ttr, ttr, 0);
//        return sprintf((char*)buf, "sat:%d(%s): toe:%s toc:%s ttr:%s iode:%3d iodc:%3d svh:%2d\r\n", eph->sat, id, toe, toc, ttr, eph->iode, eph->iodc, eph->svh);
//    }
//    return 0;
//}

uint32_t nl_gnss_sol2str(char *buf, gnss_sol_t *sol)
{
    uint32_t len = 0;
    
    char time[32];
    nl_time2str(sol->gpst, time, 3);
    int week;
    double tow = nl_time2gpst(sol->gpst, &week);

    if(sol->solq != SOLQ_NONE)
    {
      len += sprintf((char*)buf+len, "LLA:       %.8f,%.8f,%.1f UNDULATION:%.1f, Q:%d\r\n", sol->lla[0]*R2D, sol->lla[1]*R2D, sol->lla[2], sol->undulation, sol->solq);
      len += sprintf((char*)buf+len, "GPST:      %s (WN:%d TOW:%.1f)\r\n", time, week, tow);
      //len += sprintf((char*)buf+len, "VENU:      %.3f(%.2f),%.3f(%.2f),%.3f(%.2f)\r\n", sol->vel_enu[0], sol->vel_enu_std[0], sol->vel_enu[1], sol->vel_enu_std[1], sol->vel_enu[2], sol->vel_enu_std[2]);
      //len += sprintf((char*)buf+len, "DIFF_AGE:  %.1f\r\n", sol->diff_age);
      //len += sprintf((char*)buf+len, "LEAP:      %d NV:%d\r\n", sol->leap_sec, sol->nv);
      //if(sol->solq_heading == SOLQ_FIX)
      //{
      //   len += sprintf((char*)buf+len, "DUAL_ENU:   %.3f(%.2f),%.3f(%.2f),%.3f(%.2f)\r\n", sol->dual_enu[0], sol->dual_enu_std[0], sol->dual_enu[1], sol->dual_enu_std[1], sol->dual_enu[2], sol->dual_enu_std[2]);
      //}
    }
    else
    {
       len += sprintf((char*)buf+len, "SOLQ:%d", sol->solq);
    }

    return len;
}

//uint32_t nl_obsd2str(char *buf, obsd_t *obsd, char opt)
//{
//    char str[64], id[8];
//    nl_satno2id(obsd->sat, id);
//    nl_time2str(obsd->time, str, 0);
//    if (opt == 'N') /* NORMAL mode */
//    {
//        return sprintf((char*)buf, "sat:%2d(%s) %s L0:%13.1f P0:%13.1f LLI0:%d SNR0:%.0f L1:%13.1f P1:%13.1f LLI1:%d SNR1:%.0f\n", obsd->sat, id, str,
//                       obsd->L[0], obsd->P[0], obsd->LLI[0], obsd->SNR[0] * SNR_UNIT, obsd->L[1], obsd->P[1], obsd->LLI[1], obsd->SNR[1] * SNR_UNIT);
//    }
//    return 0;
//}


void nl_time2str(nl_gtime_t t, char *s, int n)
{
    nl_t ep[6];
    
    if (n<0) n=0; else if (n>12) n=12;
    if (1.0-t.sec<0.5/pow(10.0,n)) {t.time++; t.sec=0.0;};
    nl_time2epoch(t,ep);
    sprintf(s, "%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f",ep[0],ep[1],ep[2],ep[3],ep[4],n<=0?2:n+3,n<=0?0:n,ep[5]);
}

///* convert eph string to eph struct */
//void nl_str2eph(char *buf, eph_t *eph)
//{
//    int sat;
//    long toe_time, tof_time, toc_time, ttr_time;
//    eph_t eph0 = {0, -1, -1};
//    char *p = buf + 3;

//    sat = nl_satid2no((char*)buf);

//    *eph = eph0;
//    eph->sat = sat;
//    toe_time = toc_time = ttr_time = 0;
//    (void)sscanf((p + 1), "%d,%d,%d,%d,%ld,%ld,%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,"
//                        "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d",
//                 &eph->iode, &eph->iodc, &eph->sva,
//                 &eph->svh,
//                 &toe_time, &toc_time, &ttr_time,
//                 &eph->A, &eph->e, &eph->i0,
//                 &eph->OMG0, &eph->omg, &eph->M0,
//                 &eph->deln, &eph->OMGd, &eph->idot,
//                 &eph->crc, &eph->crs, &eph->cuc,
//                 &eph->cus, &eph->cic, &eph->cis,
//                 &eph->toes, &eph->fit, &eph->f0,
//                 &eph->f1, &eph->f2, &eph->tgd[0],
//                 &eph->code, &eph->flag);
//    eph->toe.time = toe_time;
//    eph->toc.time = toc_time;
//    eph->ttr.time = ttr_time;
//}








