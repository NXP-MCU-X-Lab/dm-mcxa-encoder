/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_TFORMAT_H_
#define APP_TFORMAT_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void TFormat_Init(void);
void TFormat_Publish(uint32_t angle_counts, uint32_t encoder_status, bool calibrated);
void TFormat_PublishDiagnostics(uint32_t encoder_status,
                                bool calibrated,
                                uint32_t calibration_source,
                                float mag16_raw,
                                float mag15_raw,
                                uint32_t adc_sample_count,
                                uint32_t adc_overrun_count);

extern volatile uint32_t tformat_id0_request_count;
extern volatile uint32_t tformat_id3_request_count;
extern volatile uint32_t tformat_diag_request_count;
extern volatile uint32_t tformat_response_count;
extern volatile uint32_t tformat_busy_count;
extern volatile uint32_t tformat_unsupported_count;
extern volatile uint32_t tformat_uart_error_count;

#ifdef __cplusplus
}
#endif

#endif /* APP_TFORMAT_H_ */
