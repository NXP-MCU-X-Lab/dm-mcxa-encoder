#include "hipnuc.h"

#define NL_DBG_TAG    "hipnuc_dec"
#define NL_DBG_LVL    NL_DBG_WARNING
#include "nl_log.h"

/* legcy support of HI226/HI229 */
#define HIPNUC_NODE_ID          (0x90)
#define HIPNUC_ID_ACC_RAW       (0xA0)
#define HIPNUC_ID_ACC_USR       (0xA1)
#define HIPNUC_ID_GYR_RAW       (0xB0)
#define HIPNUC_ID_GYR_USR       (0xB1)
#define HIPNUC_ID_MAG_RAW       (0xC0)
#define HIPNUC_ID_MAG_USR       (0xC1)
#define HIPNUC_ID_EULER         (0xD0)
#define HIPNUC_ID_QUAT          (0xD1)
#define HIPNUC_ID_PRS           (0xF0)

/* new HiPNUC standard packet */
#define HIPNUC_ID_HI91          (0x91)
#define HIPNUC_ID_HI92          (0x92)
#define HIPNUC_ID_HI81          (0x81)
#define HIPNUC_ID_HI43          (0x43)
#define HIPNUC_ID_HI85          (0x85)

#ifndef D2R
#define D2R (0.0174532925199433F)
#endif

#ifndef R2D
#define R2D (57.2957795130823F)
#endif

#ifndef GRAVITY
#define GRAVITY (9.8F)
#endif


static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len);

/* common type conversion */
#define I2(p) (*((int16_t *)(p)))
static uint16_t U2(uint8_t *p)
{
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

static float R4(uint8_t *p)
{
    float r;
    memcpy(&r, p, 4);
    return r;
}

/* parse the payload of a frame and feed into data section */
static int parse_data(hipnuc_raw_t *raw)
{
    int ofs = 0;
    uint8_t *p = &raw->buf[CH_HDR_SIZE];
    
    /* ignore all previous data */
    raw->hi91.tag = 0;
    raw->hi81.tag = 0;
    raw->hi92.tag = 0;
    raw->hi43.tag = 0;
    raw->hi85.tag = 0;

    while (ofs < raw->len)
    {
        switch (p[ofs])
        {
        case HIPNUC_NODE_ID:
            ofs += 2;
            break;
        case HIPNUC_ID_ACC_RAW:
        case HIPNUC_ID_ACC_USR:
             raw->hi91.tag = 0x90;
             raw->hi91.acc[0] = (float)I2(p + ofs + 1) / 1000;
             raw->hi91.acc[1] = (float)I2(p + ofs + 3) / 1000;
             raw->hi91.acc[2] = (float)I2(p + ofs + 5) / 1000;
            ofs += 7;
            break;
        case HIPNUC_ID_GYR_RAW:
        case HIPNUC_ID_GYR_USR:
            raw->hi91.tag = 0x90;
            raw->hi91.gyr[0] = (float)I2(p + ofs + 1) / 10;
            raw->hi91.gyr[1] = (float)I2(p + ofs + 3) / 10;
            raw->hi91.gyr[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_MAG_RAW:
            raw->hi91.tag = 0x90;
            raw->hi91.mag[0] = (float)I2(p + ofs + 1) / 10;
            raw->hi91.mag[1] = (float)I2(p + ofs + 3) / 10;
            raw->hi91.mag[2] = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_EULER:
            raw->hi91.tag = 0x90;
            raw->hi91.pitch = (float)I2(p + ofs + 1) / 100;
            raw->hi91.roll = (float)I2(p + ofs + 3) / 100;
            raw->hi91.yaw = (float)I2(p + ofs + 5) / 10;
            ofs += 7;
            break;
        case HIPNUC_ID_QUAT:
            ofs += 17;
            break;
        case HIPNUC_ID_PRS:
            raw->hi91.tag = 0x90;
            raw->hi91.air_pressure = R4(p + ofs + 1);
            ofs += 5;
            break;
        case HIPNUC_ID_HI91:
            memcpy(&raw->hi91, p + ofs, sizeof(hi91_t));
            ofs += sizeof(hi91_t);
            break;
        case HIPNUC_ID_HI81:
            memcpy(&raw->hi81, p + ofs, sizeof(hi81_t));
            ofs += sizeof(hi81_t);
            break;
        case HIPNUC_ID_HI92:
            memcpy(&raw->hi92, p + ofs, sizeof(hi92_t));
            ofs += sizeof(hi92_t);
            break;
        case HIPNUC_ID_HI43:
            memcpy(&raw->hi43, p + ofs, sizeof(hi43_t));
            ofs += sizeof(hi43_t);
            break;
       case HIPNUC_ID_HI85:
            memcpy(&raw->hi85, p + ofs, sizeof(hi85_t));
            ofs += sizeof(hi85_t);
            break;
        default:
            ofs++;
            break;
        }
    }
    return 1;
}

static int decode_hipnuc(hipnuc_raw_t *raw)
{
    uint16_t crc = 0;

    /* checksum */
    hipnuc_crc16(&crc, raw->buf, (CH_HDR_SIZE-2));
    hipnuc_crc16(&crc, raw->buf + CH_HDR_SIZE, raw->len);
    if (crc != U2(raw->buf + (CH_HDR_SIZE-2)))
    {
        NL_LOG_W("ch checksum error: frame:0x%X calcuate:0x%X, len:%d\n", U2(raw->buf + 4), crc, raw->len);
        return -1;
    }

    return parse_data(raw);
}

/* sync code */
static int sync_hipnuc(uint8_t *buf, uint8_t data)
{
    buf[0] = buf[1];
    buf[1] = data;
    return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

/**
* @brief     hipnuc decoder input, read one byte at one time.
 *
 * @param    raw is the decoder struct.
 * @param    data is the one byte read from stram.
 * @param    buf is the log string buffer, ireturn > 0: decoder received a frame successfully, else: receiver not receive a frame successfully.
 *
 */
int hipnuc_input(hipnuc_raw_t *raw, uint8_t data)
{
    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (!sync_hipnuc(raw->buf, data))
            return 0;
        raw->nbyte = 2;
        return 0;
    }

    raw->buf[raw->nbyte++] = data;

    if (raw->nbyte == CH_HDR_SIZE)
    {
        if ((raw->len = U2(raw->buf + 2)) > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE))
        {
            NL_LOG_W("ch length error: len=%d\n",raw->len);
            raw->nbyte = 0;
            return -1;
        }
    }

    if (raw->nbyte < CH_HDR_SIZE || raw->nbyte < (raw->len + CH_HDR_SIZE))
    {
        return 0;
    }

    raw->nbyte = 0;

    return decode_hipnuc(raw);
}



/**
 * @brief    calcuate hipnuc_crc16
 *
 * @param    inital is intial value
 * @param    buf    is input buffer pointer
 * @param    len    is length of the buffer
 *
 */
static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len)
{
    uint32_t crc = *inital;
    uint32_t j;
    for (j=0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = buf[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *inital = crc;
}
