#include "sanchi.h"
#include "app_def.h"

#define CHSYNC1         (0xA5)
#define CHSYNC2         (0x5A)

void hsi3_start_manual_cal(void);

typedef struct __attribute__((__packed__))
{
    uint8_t header[2];
    uint8_t len;
    int16_t eul[3];
    int16_t acc[3];
    int16_t gyr[3];
    int16_t mag[3];
    int32_t pr;
    int16_t temperature;
    uint32_t time;
    uint8_t mag_flag;
    uint8_t checknum;
    uint8_t tail;
}sanchi_ptl_100s;

typedef struct __attribute__((__packed__))
{
    uint8_t header[2];
    uint8_t len;
    int16_t acc[3];
    int16_t gyr[3];
    int16_t temperature;
    float eul[3];
    int16_t velocity[3];
    double longitude;
    double latitude;
    float  altitude;
    uint8_t rev[4];
    int16_t GNSS_velocity;
    int16_t GNSS_yaw;
    int16_t PDOP;
    int16_t VDOP;
    int16_t HDOP;
    char status;
    int16_t mag[3];
    int16_t mag_yaw;
    float par_temperature;
    float par;
    float GNSSaltitude;
    uint8_t crc;
    uint8_t tail;
}sanchi_ptl_200s;

static uint8_t cksum(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0;
    for(int i = 0; i < len-3; i++)
    {
        crc += buf[i+3];
    }
    crc += len;

    return crc;
}

static uint8_t count_sanchibuf_crc(uint8_t *databuf, uint8_t len)
{
    uint8_t crc = 0;
    for(int i = 0; i < len; i++)
    {
        crc += databuf[i + 3];
    }
    crc += databuf[2];
    
    return crc;
}

static uint8_t get_sanchibuf_len_200s(sanchi_ptl_200s *databuf)
{
    return (sizeof(*databuf) - 5);
}

static uint8_t get_sanchibuf_len_100s(sanchi_ptl_100s *databuf)
{
    return (sizeof(*databuf) - 3);
}

void exchange_data(uint8_t *data, uint16_t len)
{
    uint8_t temp;
    for (int i = 0; i < len; i++)
    {
        temp = data[i];
        data[i] = data[i + 1];
        data[i + 1] = temp;
        i++;
    }
}

int sanchi_setup_frame_ch104_100s(uint8_t *buf, float *acc, float *gyr, float *quat, float *mag, float pitch, float roll, float yaw, float prs, float temperature)
{
    sanchi_ptl_100s *databuf = (sanchi_ptl_100s *)buf;

     /* acc */
    databuf->acc[0] = -(int16_t)(acc[0] / 9.8 * 16384 );
    databuf->acc[1] = -(int16_t)(acc[1] / 9.8 * 16384 );
    databuf->acc[2] =  (int16_t)(acc[2] / 9.8 * 16383 );
    exchange_data((uint8_t *)databuf->acc, 6);
     /* gyr */
    databuf->gyr[0] = -(int16_t)(gyr[0] * 180 / PI * 32.768);
    databuf->gyr[1] = -(int16_t)(gyr[1] * 180 / PI * 32.768);
    databuf->gyr[2] =  (int16_t)(gyr[2] * 180 / PI * 32.768);
    exchange_data((uint8_t *)databuf->gyr, sizeof(databuf->gyr));
    /* mag */
    databuf->mag[0] = (int16_t)(mag[0] * 1);
    databuf->mag[1] = (int16_t)(mag[1] * 1);
    databuf->mag[2] = (int16_t)(mag[2] * 1);
    exchange_data((uint8_t *)databuf->mag, sizeof(databuf->mag));
    /* eul */
    databuf->eul[0] = -(yaw - 180) * 10;
    databuf->eul[1] = -pitch * 10;
    databuf->eul[2] = -roll * 10;
    exchange_data((uint8_t *)databuf->eul, sizeof(databuf->eul));

    /* temperature */
    databuf->temperature = (uint16_t)(temperature * 100);
    exchange_data((uint8_t *)&databuf->temperature, sizeof(databuf->temperature));

    databuf->header[0] = CHSYNC1;
    databuf->header[1] = CHSYNC2;
    databuf->tail = 0xaa;
    databuf->len = get_sanchibuf_len_100s(databuf);
    databuf->checknum = cksum((uint8_t *)databuf, databuf->len);

    return sizeof(sanchi_ptl_100s);
}

int sanchi_setup_frame_ch104_200s(uint8_t *buf, float *acc, float *gyr, float *quat, float *mag, float pitch, float roll, float yaw, float prs, float temperature)
{
    sanchi_ptl_200s *databuf = (sanchi_ptl_200s *)buf;

    /* acc */
    databuf->acc[0] = -(int16_t)(acc[0] * 2000 / 9.8);
    databuf->acc[1] = -(int16_t)(acc[1] * 2000 / 9.8);
    databuf->acc[2] =  (int16_t)(acc[2]  * 2000 / 9.8);
    /* gyr */
    databuf->gyr[0] = -(int16_t)(gyr[0] * 180 / PI * 50);
    databuf->gyr[1] = -(int16_t)(gyr[1] * 180 / PI * 50);
    databuf->gyr[2] =  (int16_t)(gyr[2] * 180 / PI * 50);
    /* mag */
    if(eskfsvr.ucfg.magcal_succ == 1)
    {
        mag[0] = 999;
        mag[1] = 999;
        mag[2] = 999;
    }
    databuf->mag[0] = (int16_t)(mag[0] * 1);
    databuf->mag[1] = (int16_t)(mag[1] * 1);
    databuf->mag[2] = (int16_t)(mag[2] * 1);
    /* eul */
    databuf->eul[0] = -pitch;
    databuf->eul[1] = -roll;
    databuf->eul[2] = -(yaw - 180);
    /* temperature */
    databuf->temperature = (uint16_t)(temperature * 100);

    databuf->header[0] = 0x55;
    databuf->header[1] = 0xaa;
    databuf->tail = 0xBB;
    databuf->len = get_sanchibuf_len_200s(databuf);
    databuf->crc = count_sanchibuf_crc((uint8_t *)databuf, databuf->len);

    return sizeof(sanchi_ptl_200s);
}

int sanchi_serial_input(uint8_t *buf, uint32_t len)
{
    if(len >= 6 && buf[0] == CHSYNC1 && buf[1] == CHSYNC2)
    {
        uint8_t payload_len = buf[2];
        uint8_t chksum = buf[payload_len];
        uint8_t cmd = buf[3];
        uint8_t val = buf[payload_len-1];
        uint8_t chksum_cal = cksum(buf, payload_len);
        uint8_t send_buf[12] = {0xa5, 0x5a, 0x0, 0x64, 0x44, 0x66, 0x50, 0xae, 0x86, 0x6d, 0xFF, 0xAA};
        
        //rt_kprintf("cmd:0x%X, val:%d, CRCCAL:0x%X, CRC:0x%X\r\n", cmd, val, chksum_cal, chksum);
        
        if(chksum_cal != chksum) return RT_ERROR;

        switch(cmd)
        {
            case 0xA8:
            {
                if(val > 0 && val <= 200)
                {
                    eskfsvr.ucfg.log_type[VCOM0][OUTPUT_MSG_SANCHI_100S] = LOG_TYPE_ONTIME;
                    eskfsvr.ucfg.log_ontime_ms[VCOM0][OUTPUT_MSG_SANCHI_100S] = 1000 / val;

                    eskfsvr.ucfg.log_type[VCOM0][OUTPUT_MSG_SANCHI_200S] = LOG_TYPE_ONTIME;
                    eskfsvr.ucfg.log_ontime_ms[VCOM0][OUTPUT_MSG_SANCHI_200S] = 1000 / val;
                    nvdm_write(&eskfsvr.ucfg);
                }

                break;
            }
            case 0: /* ID */
                for(int i = 0; i < 12; i++)
                    rt_kprintf("%c", send_buf[i]);
                break;
            case 1: /* ENBALE */
                eskfsvr.en_output[VCOM0] = 1;
                break;
            case 2: /* DISABLE */
                eskfsvr.en_output[VCOM0] = 0;
                break;
            case 3:
                break;
            case 0xE1: /* save mag result */
                break;
            case 0xE2: /* open 9 xias */
                eskfsvr.ucfg.en_kf_meas_mag = 1;
                eskfsvr.ucfg.en_kf_meas_gravity = 1;
                nvdm_write(&eskfsvr.ucfg);
                break;
            case 0xE3: /* start mag cal */
                hsi3_start_manual_cal();
                break;
            case 0xE4: /* 6 xias */
                eskfsvr.ucfg.en_kf_meas_mag = 0;
                eskfsvr.ucfg.en_kf_meas_gravity = 1;
                nvdm_write(&eskfsvr.ucfg);
                break;
        }
    }
}

