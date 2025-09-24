#ifndef __NL_WT_H__
#define __NL_WT_H__

#ifdef __cplusplus
extern "C"{
#endif


#include "nl.h"

typedef struct
{
    int         nbyte;
    int         len;
    uint8_t     type;
    uint8_t     buf[MAXRAWLEN];
    double      gyr_y;
    double      gyr_z;
    double      yaw;
}nl_wt_raw_t;



int nl_input_wt(nl_wt_raw_t *raw, uint8_t ch);

#ifdef __cplusplus
}
#endif


#endif




