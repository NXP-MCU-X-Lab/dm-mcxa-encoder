#ifndef APP_ENCODER_RUNTIME_H_
#define APP_ENCODER_RUNTIME_H_

#include <stdint.h>

#include "app_adc.h"
#include "app_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_CAL_SOURCE_DEFAULT         (0U)
#define ENCODER_CAL_SOURCE_NVM             (1U)
#define ENCODER_CAL_SOURCE_FACTORY_PENDING (2U)
#define ENCODER_CAL_SOURCE_INVALID         (3U)

#define FM_COMMAND_NONE          (0U)
#define FM_COMMAND_ZERO_SAVE     (1U)
#define FM_COMMAND_FACTORY_CAL   (2U)
#define FM_COMMAND_ERASE_CONFIG  (3U)

#define FM_COMMAND_STATE_IDLE  (0U)
#define FM_COMMAND_STATE_BUSY  (1U)
#define FM_COMMAND_STATE_DONE  (2U)
#define FM_COMMAND_STATE_ERROR (3U)

#define FM_COMMAND_STATUS_OK          (0U)
#define FM_COMMAND_STATUS_MOVING      (1U)
#define FM_COMMAND_STATUS_BUSY        (2U)
#define FM_COMMAND_STATUS_STORAGE     (3U)
#define FM_COMMAND_STATUS_CALIBRATION (4U)
#define FM_COMMAND_STATUS_NOT_READY   (5U)

extern encoder_result_t encoder_result;
extern adc_sample_result_t adc_result;
extern volatile uint32_t adc_sample_count;
extern volatile uint32_t adc_overrun_count;
extern volatile uint32_t encoder_calibration_source;
extern volatile uint32_t encoder_perf_process_max;
extern volatile uint32_t encoder_perf_isr_max;
extern volatile uint32_t encoder_perf_core_clock_hz;

void EncoderApp_Init(void);
void EncoderApp_Service(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_ENCODER_RUNTIME_H_ */
