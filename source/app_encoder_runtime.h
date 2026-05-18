#ifndef APP_ENCODER_RUNTIME_H_
#define APP_ENCODER_RUNTIME_H_

#include <stdint.h>

#include "app_adc.h"
#include "app_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

extern adc_sample_result_t adc_result;
extern encoder_result_t encoder_result;
extern encoder_diag_t encoder_diag;
extern float encoder_cal_flat[14];
extern volatile uint32_t adc_sample_count;
extern volatile uint32_t adc_overrun_count;
extern volatile uint32_t encoder_sample_rate_hz;

void EncoderApp_Init(void);
void EncoderApp_Service(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ENCODER_RUNTIME_H_ */
