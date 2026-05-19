#ifndef APP_ENCODER_RUNTIME_H_
#define APP_ENCODER_RUNTIME_H_

#include <stdint.h>

#include "app_adc.h"
#include "app_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

extern encoder_result_t encoder_result;
/* Raw 4-channel ADC sample, refreshed every ADC ISR. Exposed for the
 * FreeMASTER oscilloscope so the host can plot the four OPAMP outputs
 * directly. Read/write tearing is possible (8 bytes, non-atomic on M33)
 * but harmless for scope visualisation. */
extern adc_sample_result_t adc_result;
extern volatile uint32_t adc_sample_count;
extern volatile uint32_t adc_overrun_count;
extern volatile uint32_t encoder_calibration_source;

/* Compute-time profiler outputs sampled with the Cortex-M33 DWT cycle counter.
 *   process_cycles : last encoder_process() call duration.
 *   isr_cycles     : last full ADC ISR callback (process + AGC + snapshot copy).
 *   *_max          : peak observed since boot.
 *   core_clock_hz  : CPU frequency, lets the UI convert cycles to microseconds
 *                    and to a % CPU utilisation at the 10 kHz sample rate. */
extern volatile uint32_t encoder_perf_process_cycles;
extern volatile uint32_t encoder_perf_process_max;
extern volatile uint32_t encoder_perf_isr_cycles;
extern volatile uint32_t encoder_perf_isr_max;
extern volatile uint32_t encoder_perf_core_clock_hz;

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
