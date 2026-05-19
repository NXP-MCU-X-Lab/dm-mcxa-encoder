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
extern float encoder_cal_flat[12];
extern float encoder_runtime_trim_delta[4];
extern volatile uint32_t adc_sample_count;
extern volatile uint32_t adc_overrun_count;
extern volatile uint32_t encoder_sample_rate_hz;
extern volatile uint32_t encoder_calibration_source;
extern volatile uint32_t encoder_storage_crc_ok;
extern volatile uint8_t encoder_runtime_trim_enabled;
extern volatile uint8_t encoder_runtime_trim_active;

#define ENCODER_CAL_SOURCE_DEFAULT (0U)
#define ENCODER_CAL_SOURCE_NVM (1U)
#define ENCODER_CAL_SOURCE_FACTORY_PENDING (2U)
#define ENCODER_CAL_SOURCE_INVALID (3U)

#define ENCODER_FACTORY_CAL_STATE_IDLE (0U)
#define ENCODER_FACTORY_CAL_STATE_RUNNING (1U)
#define ENCODER_FACTORY_CAL_STATE_DONE (2U)
#define ENCODER_FACTORY_CAL_STATE_FAILED (3U)

void EncoderApp_Init(void);
void EncoderApp_Service(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ENCODER_RUNTIME_H_ */
