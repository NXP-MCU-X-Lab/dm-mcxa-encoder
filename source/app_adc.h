/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#ifndef _APP_ADC_H_
#define _APP_ADC_H_

/* ADC interface for dual-track inductive encoder hardware.
 * Provides the realtime sampling rate, result structure, and APIs for reading
 * both encoder lanes as raw ADC values.
 */

#include <stdint.h>

/* Encoder Pin Mapping:
 * A1_SIN: MCU OPAMP0_OUT -> P2_15 (ADC0_A2)
 * A1_COS: MCU OPAMP1_OUT -> P2_19 (ADC1_A2)
 * A2_SIN: TLV9062 A2_OPA0_OUT -> P2_6 (ADC1_A3)
 * A2_COS: TLV9062 A2_OPA1_OUT -> P2_7 (ADC0_A7)
 */

#define ADC_SAMPLE_RATE_HZ      10000U

/* ADC Sample Result Structure */
typedef struct {
    uint16_t a1_sin_raw;
    uint16_t a1_cos_raw;
    uint16_t a2_sin_raw;
    uint16_t a2_cos_raw;
} adc_sample_result_t;

typedef void (*adc_sample_callback_t)(const adc_sample_result_t *sample);

/* Function Prototypes */

/**
 * @brief Initialize ADC for encoder raw lane sampling.
 */
void adc_init(void);

/**
 * @brief Set the callback invoked from ADC interrupt context when a full four-channel sample is ready.
 *
 * The callback must be short and non-blocking.
 */
void adc_set_sample_callback(adc_sample_callback_t callback);

/**
 * @brief Start CTIMER0-triggered realtime ADC sampling.
 */
void adc_realtime_start(void);

/**
 * @brief Stop CTIMER0-triggered realtime ADC sampling.
 */
void adc_realtime_stop(void);

/**
 * @brief Get the number of complete four-channel samples captured by the realtime sampler.
 */
uint32_t adc_get_sample_count(void);

/**
 * @brief Get ADC FIFO overflow or incomplete-pair overrun count.
 */
uint32_t adc_get_overrun_count(void);


#endif /* _APP_ADC_H_ */
