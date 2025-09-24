#include "nl_wt.h"

#define WT_SYNC1                (0x55)
#define WT_HEADER_GYR           (0x52)
#define WT_HEADER_ANGLE         (0x53)


static int wt_checksum(uint8_t *buff, int len)
{
    uint8_t chksum = 0;
    int i;

    len = 10;
    for (i = 0; i < len; i++)
    {
        chksum += buff[i];
    }

    return chksum == buff[len];
}

static int sync_wt(uint8_t *buff, uint8_t data)
{
    buff[0] = buff[1];
    buff[1] = data;
    return buff[0] == WT_SYNC1 && (buff[1] == WT_HEADER_GYR || buff[1] == WT_HEADER_ANGLE);
}


static int decode_wt(nl_wt_raw_t *raw)
{
    if (!wt_checksum(raw->buf, raw->len))
        return 0;

    if(raw->type == WT_HEADER_GYR)
    {
         raw->gyr_y = (double)((int16_t)(raw->buf[4] | (raw->buf[5]<<8))) * 2000 / 32768;
         raw->gyr_z = (double)((int16_t)(raw->buf[6] | (raw->buf[7]<<8))) * 2000 / 32768;
         return 1;
    }

    if(raw->type == WT_HEADER_ANGLE)
    {
         raw->yaw = (double)((int16_t)(raw->buf[6] | (raw->buf[7]<<8))) *180 / 32768;
         return 1;
    }

    return 0;
}

int nl_input_wt(nl_wt_raw_t *raw, uint8_t ch)
{

    /* synchronize frame */
    if (raw->nbyte == 0)
    {
        if (sync_wt(raw->buf, ch))
        {
            raw->nbyte = 2;
            raw->len = 10;
            raw->type = raw->buf[1];
        }
        return 0;
    }

    raw->buf[raw->nbyte++] = ch;

    if (raw->nbyte < 11)
        return 0;

    raw->nbyte = 0;

    return decode_wt(raw);
}
