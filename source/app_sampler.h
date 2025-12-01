/* app_sampler.h - CTIMER1-based 1kHz sampling ISR */
#ifndef APP_SAMPLER_H
#define APP_SAMPLER_H

/* Simple CTIMER1-based sampler interface.
 * Starts/stops periodic ISR that reads ADC and updates encoder results,
 * and provides thread-safe copy helpers for latest data.
 */

#include <stdint.h>
#include "app_encoder.h"
#include "app_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize CTIMER1 periodic interrupt for sampling
 * @param freq_hz Sampling frequency (e.g., 1000 for 1kHz)
 */
void sampler_init(uint32_t freq_hz);

/**
 * @brief Start periodic sampling interrupt
 */
void sampler_start(void);

/**
 * @brief Stop periodic sampling interrupt
 */
void sampler_stop(void);

/**
 * @brief Safely copy latest encoder result from ISR context
 * @param out Pointer to destination struct
 */
void sampler_copy_latest(encoder_result_t *out);

/**
 * @brief Safely copy latest raw ADC result from ISR context
 * @param out Pointer to destination struct
 */
void sampler_copy_latest_raw(adc_sample_result_t *out);

/**
 * @brief Get ISR performance statistics
 * @param isr_count Total ISR execution count
 * @param max_time_us Maximum ISR execution time in microseconds
 * @param last_time_us Last ISR execution time in microseconds
 */
void sampler_get_stats(uint32_t *isr_count, uint32_t *max_time_us, uint32_t *last_time_us);

uint32_t sampler_get_timer_clock_hz(void);
uint32_t sampler_get_timer_count(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SAMPLER_H */
