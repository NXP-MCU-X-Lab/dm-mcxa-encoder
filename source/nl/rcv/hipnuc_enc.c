#include "hipnuc.h"
#include "nl.h"



static void hipnuc_crc16(uint16_t *inital, const uint8_t *buf, uint32_t len);

typedef struct __attribute__((packed))
{
    uint8_t sync1;
    uint8_t sync2;
    uint16_t payload_len;
    uint16_t crc;
    uint8_t payload[HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE];
}hipnuc_frame;


int bin_hi91data(uint8_t *buf, uint16_t main_status, uint32_t sys_time, nl_t *acc, nl_t *gyr, nl_t *quat, nl_t *mag, nl_t pitch, nl_t roll, nl_t yaw, nl_t air_pressure, nl_t temperature)
{
    uint16_t crc = 0;
    
    hipnuc_frame *hframe = (hipnuc_frame*)buf;
    hframe->sync1 = CHSYNC1;
    hframe->sync2 = CHSYNC2;
    hframe->payload_len = sizeof(hi91_t);
    
    /* load data */
    hi91_t *hi91 = (hi91_t*)&hframe->payload;
    
    hi91->tag = 0x91;
    hi91->main_status = main_status;
    hi91->air_pressure = air_pressure;
    hi91->acc[0] = acc[0] / GRAVITY;
    hi91->acc[1] = acc[1] / GRAVITY;
    hi91->acc[2] = acc[2] / GRAVITY;
    hi91->gyr[0] = gyr[0] * R2D;
    hi91->gyr[1] = gyr[1] * R2D;
    hi91->gyr[2] = gyr[2] * R2D;
    hi91->mag[0] = mag[0];
    hi91->mag[1] = mag[1];
    hi91->mag[2] = mag[2];

    hi91->roll = roll;
    hi91->pitch = pitch;
    hi91->yaw = yaw;

    hi91->system_time = sys_time;
    hi91->temp = temperature;

    hi91->quat[0] = quat[0];
    hi91->quat[1] = quat[1];
    hi91->quat[2] = quat[2];
    hi91->quat[3] = quat[3];

    /* CRC */
    hipnuc_crc16(&crc, (const uint8_t *)hframe, 4);
    hipnuc_crc16(&crc, (const uint8_t *)&hframe->payload, hframe->payload_len);
    hframe->crc = crc;

    return hframe->payload_len + CH_HDR_SIZE;
}




/*  generate CRC16
    @param  currectCrc:     previous buffer pointer, if is a new start, pointer should refer a zero uint16_t
    @param  src:            current buffer pointer
    @param  lengthInBytes:  length of current buf
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

