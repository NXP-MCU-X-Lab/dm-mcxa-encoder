#ifndef __RTCM_SAMPLE_H__
#define __RTCM_SAMPLE_H__

#include <stdint.h>

uint8_t base_rtcm_sample[];
uint8_t rover_rtcm_sample[];
uint32_t base_rtcm_sample_len(void);
uint32_t rover_rtcm_sample_len(void);

#endif
