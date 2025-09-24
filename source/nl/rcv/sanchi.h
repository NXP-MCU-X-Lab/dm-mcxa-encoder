#ifndef __SANCHI_H__
#define __SANCHI_H__

#include "nl.h"

int sanchi_setup_frame(uint8_t *buf, nl_t gyr_x, nl_t gyr_y, nl_t gyr_z, nl_t temperature);
int sanchi_setup_frame_ch104_100s(uint8_t *buf, float *acc, float *gyr, float *quat, float *mag, float pitch, float roll, float yaw, float prs, float temperature);
int sanchi_setup_frame_ch104_200s(uint8_t *buf, float *acc, float *gyr, float *quat, float *mag, float pitch, float roll, float yaw, float prs, float temperature);

#endif


