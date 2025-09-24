#include "ubx.h"

#define UBXSYNC1 0xB5 /* ubx message sync code 1 */
#define UBXSYNC2 0x62 /* ubx message sync code 2 */

#define ID_NAVPVT 0x0107 /* ubx message id: nav solution info */

#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}
static uint32_t U4(uint8_t *p)
{
    uint32_t u;
    memcpy(&u, p, 4);
    return u;
}
static int32_t I4(uint8_t *p)
{
    int32_t i;
    memcpy(&i, p, 4);
    return i;
}
static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}
static double R8(uint8_t *p)
{
    double r;
    memcpy(&r, p, 8);
    return r;
}

static int ubx_checksum(uint8_t *buff, int len)
{
    uint8_t cka = 0, ckb = 0;
    int i;
    for (i = 2; i < len - 2; i++)
    {
        cka += buff[i];
        ckb += cka;
    }
    return cka == buff[len - 2] && ckb == buff[len - 1];
}

static int sync_ubx(uint8_t *buff, uint8_t data)
{
    buff[0] = buff[1];
    buff[1] = data;
    return buff[0] == UBXSYNC1 && buff[1] == UBXSYNC2;
}

static int decode_navpvt(ubx_raw_t *raw)
{
    memcpy(&raw->pvt, raw->buf + 6, sizeof(ubx_pvt_t));
    return 1;
}

void ubxpvt2gnss_sol(gnss_sol_t *sol, ubx_pvt_t *pvt)
{
    /* get UTC and leaps */
    nl_t epoch[6];
    epoch[0] = pvt->year;
    epoch[1] = pvt->month;
    epoch[2] = pvt->day;
    epoch[3] = pvt->hour;
    epoch[4] = pvt->min;
    epoch[5] = pvt->sec;
    nl_gtime_t utc = nl_epoch2time(epoch);
    sol->gpst = nl_utc2gpst(utc);
    int week;
    nl_time2gpst(sol->gpst, &week);
    sol->gpst = nl_gpst2time(week, ((double)pvt->iTOW) * 1e-3);

    sol->nv = pvt->numSV;
    sol->lla[0] = ((double)pvt->lat) * 1e-7 * D2R;
    sol->lla[1] = ((double)pvt->lon) * 1e-7 * D2R;
    sol->lla[2] = ((double)pvt->height) * 1e-3;
    sol->undulation = sol->lla[2] - ((double)pvt->hMSL) * 1e-3;

    sol->vel_enu[0] = pvt->velE * 1e-3;
    sol->vel_enu[1] = pvt->velN * 1e-3;
    sol->vel_enu[2] = -pvt->velD * 1e-3;

//    sol->pos_enu_std[0] = sol->pos_enu_std[1] = ((double)pvt->hAcc) * 1e-3;
//    sol->pos_enu_std[2] = ((double)pvt->vAcc) * 1e-3;

    if (pvt->fixType == 2 || pvt->fixType == 3 || pvt->fixType == 4)
        sol->solq = SOLQ_SINGLE;
    else
        sol->solq = SOLQ_NONE;

    sol->solq_heading = SOLQ_NONE;
}

static int decode_ubx(ubx_raw_t *raw)
{
    int type = (U1(raw->buf + 2) << 8) + U1(raw->buf + 3);
    if (!ubx_checksum(raw->buf, raw->len))
        return 0;

    switch (type)
    {
    case ID_NAVPVT:
        return decode_navpvt(raw);
        break;
    }
    return 0;
}

int input_ubx(ubx_raw_t *raw, uint8_t ch)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (sync_ubx(raw->buf, ch))
            raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = ch;

    if (raw->nbyte == 6)
    {
        if ((raw->len = U2(raw->buf + 4) + 8) > MAXRAWLEN)
        {
            raw->nbyte = 0;
            return 0;
        }
    }

    if (raw->nbyte < 6 || raw->nbyte < raw->len)
        return 0;
    raw->nbyte = 0;

    /* decode ublox raw message */
    return decode_ubx(raw);
}