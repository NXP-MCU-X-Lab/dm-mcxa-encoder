#include "airoha.h"

#define PREAMBLE1 (0x04)
#define PREAMBLE2 (0x24)
#define ENDWORD1 (0xAA)
#define ENDWORD2 (0x44)
#define MESSAGE_ID_SIZE (2)
#define LENGTH_SIZE (2)
#define HDR_SIZE (2 + MESSAGE_ID_SIZE + LENGTH_SIZE)

#define U1(p) (*((uint8_t *)(p)))
#define I1(p) (*((int8_t *)(p)))
#define I2(p) (*((int16_t *)(p)))
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

/* sync code */
static int sync_airoha(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == PREAMBLE1 && buf[1] == PREAMBLE2;
}

static int parse_data(airoha_raw_t *raw)
{

    switch (raw->type)
    {
       case AIROHA_MSG_ID_BIN_PVT:
        memcpy(&raw->pvt, &raw->buf[6], sizeof(airoha_pvt_t));
        break;
       case AIROHA_MSG_ID_BIN_PVT_ADD:
        memcpy(&raw->pvt_add, &raw->buf[6], sizeof(airoha_pvt_add_t));
        break;
       case AIROHA_MSG_ID_PBIN_TIME:
        memcpy(&raw->time, &raw->buf[6], sizeof(airoha_time_t));
        break;
    }

    // NL_TRACE("len:%d, id:%d\r\n", raw->len, raw->type);
    // if(raw->type == AIROHA_MSG_ID_RAW_MES_PVT)
    //{
    //     raw->pvt.fix_mode = raw->buf[1+6];
    //     raw->pvt.raw_meas_pvt_stat = raw->buf[11+6];

    //    if(raw->pvt.raw_meas_pvt_stat & (1<<0)) /* LLA valid */
    //    {
    //        raw->pvt.lat = ((double)I4(&raw->buf[12+6])) * 1e-7;
    //        raw->pvt.lat *= D2R;
    //        raw->pvt.lon = ((double)I4(&raw->buf[16+6])) * 1e-7;
    //        raw->pvt.lon *= D2R;
    //        raw->pvt.h_msl = ((nl_t)I4(&raw->buf[20+6])) * 1e-3f;
    //    }

    //    if(raw->pvt.raw_meas_pvt_stat & (1<<2)) /* KF solution valid */
    //    {
    //        raw->pvt.vel_enu[1] = ((nl_t)I4(&raw->buf[88+6])) * 1e-3f;
    //        raw->pvt.vel_enu[0] = ((nl_t)I4(&raw->buf[92+6])) * 1e-3f;
    //        raw->pvt.vel_enu[2] = -((nl_t)I4(&raw->buf[96+6])) * 1e-3f;
    //    }
    //    else if(raw->pvt.raw_meas_pvt_stat & (1<<1)) /* LSQ solution val  id */
    //    {
    //        raw->pvt.vel_enu[1] = ((nl_t)I4(&raw->buf[52+6])) * 1e-3f;
    //        raw->pvt.vel_enu[0] = ((nl_t)I4(&raw->buf[56+6])) * 1e-3f;
    //        raw->pvt.vel_enu[2] = -((nl_t)I4(&raw->buf[60+6])) * 1e-3f;
    //    }
    //   return 1;
    //}

    // if (raw->type == AIROHA_MSG_ID_BIN_PVT)
    //{
    //     uint8_t fix_qual = raw->buf[44 + 6];
    //     switch (fix_qual) /* 0: No fix, 1: Standard positioning service (SPS),2: Differential GNSS solution (DGPS), 4: RTK fixed, 5: RTK float */
    //     {
    //     case 0:
    //         raw->sol.solq = SOLQ_NONE;
    //         break;
    //     case 1:
    //         raw->sol.solq = SOLQ_SINGLE;
    //         break;
    //     case 2:
    //         raw->sol.solq = SOLQ_DGPS;
    //         break;
    //     case 4:
    //         raw->sol.solq = SOLQ_FIX;
    //         break;
    //     case 5:
    //         raw->sol.solq = SOLQ_FLOAT;
    //         break;
    //     };

    //    raw->sol.lla[0] = ((double)I4(&raw->buf[0 + 6])) * 1e-7 * D2R;
    //    raw->sol.lla[1] = ((double)I4(&raw->buf[4 + 6])) * 1e-7 * D2R;
    //    raw->sol.lla[2] = ((double)I4(&raw->buf[8 + 6])) * 1e-3f;
    //    raw->sol.undulation = ((double)I4(&raw->buf[12 + 6])) * 1e-3f;
    //    raw->sol.lla[2] += raw->sol.undulation; /* eplispid high */

    ////    raw->sol.undulation = geoidh(raw->sol.lla); /* FIXME DO NOT USE AG3335's undulation, use RTKLIB */

    //    raw->sol.hdop = (double)I2(&raw->buf[24 + 6]) * 1e-2f;
    //    raw->sol.vdop = (double)I2(&raw->buf[26 + 6]) * 1e-2f;
    //    raw->sol.pdop = (double)I2(&raw->buf[28 + 6]) * 1e-2f;

    //    raw->sol.nv = raw->buf[39 + 6];
    //    return 1;
    //}

    // if (raw->type == AIROHA_MSG_ID_BIN_PVT_ADD)
    //{

    //    raw->sol.pos_enu_std[1] = ((double)I4(&raw->buf[0 + 6])) * 1e-3f;
    //    raw->sol.pos_enu_std[0] = ((double)I4(&raw->buf[4 + 6])) * 1e-3f;
    //    raw->sol.pos_enu_std[2] = ((double)I4(&raw->buf[8 + 6])) * 1e-3f;
    //    raw->sol.vel_enu[1] = ((double)I4(&raw->buf[12 + 6])) * 1e-3f;
    //    raw->sol.vel_enu[0] = ((double)I4(&raw->buf[16 + 6])) * 1e-3f;
    //    raw->sol.vel_enu[2] = -((double)I4(&raw->buf[20 + 6])) * 1e-3f;
    //    raw->sol.vel_enu_std[1] = ((double)I4(&raw->buf[24 + 6])) * 1e-3f;
    //    raw->sol.vel_enu_std[0] = ((double)I4(&raw->buf[28 + 6])) * 1e-3f;
    //    raw->sol.vel_enu_std[2] = ((double)I4(&raw->buf[32 + 6])) * 1e-3f;

    //    return 1;
    //}

    // if (raw->type == AIROHA_MSG_ID_PBIN_TIME)
    //{

    //    int wn = U2(&raw->buf[4 + 6]);
    //    double tow = ((double)I4(&raw->buf[0 + 6])) * 1e-3f;
    //    raw->sol.gpst = gpst2time(wn, tow);
    //    raw->sol.leap_sec = U2(&raw->buf[8 + 6]);
    //    return 1;
    //}

    return 1;
}

static int decode_airoha(airoha_raw_t *raw)
{
    int i;
    uint8_t *p;
    uint8_t chksum, chksum2;
    if (!raw->len)
        return 0;

    raw->type = U2(raw->buf + 2);
    for (p = raw->buf + 2, chksum = 0, i = 0; i < raw->len + 4; i++)
        chksum ^= *p++;
    chksum2 = raw->buf[HDR_SIZE + raw->len];

    if (chksum2 != chksum)
    {
        NL_DEBUG_LOG(NL_DEBUG_RCV, ("airtha chksum err: len:%d msgid:%d sum:0x%02X sum2:0x%02X\r\n", raw->len, raw->type, chksum2, chksum));
        return -1;
    }

    if (raw->buf[HDR_SIZE + raw->len + 1] != ENDWORD1 || raw->buf[HDR_SIZE + raw->len + 2] != ENDWORD2)
        return -1;
    return parse_data(raw);
}

int airoha_input(airoha_raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_airoha(raw->buf, data))
            return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    if (raw->nbyte == 4)
    {
        uint16_t id = U2(raw->buf + 2);
        if (id != AIROHA_MSG_ID_RAW_MES_SV_INFO && id != AIROHA_MSG_ID_RAW_MES_PVT && id != AIROHA_MSG_ID_BIN_PVT && id != AIROHA_MSG_ID_BIN_PVT_ADD && id != AIROHA_MSG_ID_PBIN_TIME)
        {
            // NL_TRACE("aircha id error: id=%d\n",id);
            raw->nbyte = 0;
            return -1;
        }
    }

    if (raw->nbyte == HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf + 4)) > (MAXRAWLEN - HDR_SIZE))
        {
            NL_DEBUG_LOG(NL_DEBUG_RCV, ("aircha frame length error: len=%d\n", raw->len));
            raw->nbyte = 0;
            return -1;
        }
    }

    if (raw->nbyte < HDR_SIZE || raw->nbyte < (raw->len + HDR_SIZE + 3))
    {
        return 0;
    }

    raw->nbyte = 0;
    return decode_airoha(raw);
}