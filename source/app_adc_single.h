/*
 * Copyright (c) 2023
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _APP_ADC_H_
#define _APP_ADC_H_

#include "fsl_lpadc.h"
#include <stdbool.h>

/* Pin Mapping:
 * OPAMP0_INP  -> P2_12 (ADC0_A5)
 * OPAMP0_INN  -> P2_13 (ADC1_A5)
 * OPAMP0_OUT  -> P2_15 (ADC0_A2)
 * OPAMP1_INP  -> P2_16 (ADC0_A6)
 * OPAMP1_INN  -> P2_17 (ADC1_A6)
 * OPAMP1_OUT  -> P2_19 (ADC1_A2)
 */

/* ADC0 Channels */
#define ADC0_CHANNEL_A5     5
#define ADC0_CHANNEL_A6     6
#define ADC0_CHANNEL_A2     2

/* ADC1 Channels */
#define ADC1_CHANNEL_A5     5
#define ADC1_CHANNEL_A6     6
#define ADC1_CHANNEL_A2     2

/* Command IDs for chaining */
#define ADC_CMD_ID_CH0      4U  // First channel
#define ADC_CMD_ID_CH1      5U  // Second channel
#define ADC_CMD_ID_CH2      6U  // Third channel

/* Trigger ID */
#define ADC_TRIGGER_ID      0U

/* ADC sampling modes */
typedef enum {
    ADC_MODE_OUTPUT_ONLY = 1,   // Sample OPAMP0/1 OUT only (2 channels)
    ADC_MODE_FULL_DEBUG = 2,    // Sample all 6 channels (INP, INN, OUT)
    ADC_MODE_CALIBRATION = 3    // Calibration mode (same as OUTPUT_ONLY)
} adc_sampling_mode_t;

/* Magnetic encoder ADC result structure */
typedef struct {
    /* OPAMP0 differential pair (Sin signal) */
    uint16_t opamp0_inp;    // OPAMP0 positive input (ADC0_A5)
    uint16_t opamp0_inn;    // OPAMP0 negative input (ADC1_A5)
    uint16_t opamp0_out;    // OPAMP0 output (ADC0_A2)
    
    /* OPAMP1 differential pair (Cos signal) */
    uint16_t opamp1_inp;    // OPAMP1 positive input (ADC0_A6)
    uint16_t opamp1_inn;    // OPAMP1 negative input (ADC1_A6)
    uint16_t opamp1_out;    // OPAMP1 output (ADC1_A2)
} adc_encoder_result_t;

/* Function prototypes */

/**
 * @brief Initialize ADC with specified sampling mode
 * 
 * @param mode Sampling mode (OUTPUT_ONLY, FULL_DEBUG, or CALIBRATION)
 */
void ADC_Single_Init(adc_sampling_mode_t mode);

/**
 * @brief Start conversion and get all encoder signals
 * 
 * @return adc_encoder_result_t Structure containing OPAMP0/1 INP, INN, OUT values
 *         (INP/INN will be 0 if not sampled in current mode)
 */
adc_encoder_result_t ADC_StartAndGetResults(void);

/**
 * @brief Get current ADC sampling mode
 * 
 * @return adc_sampling_mode_t Current mode
 */
adc_sampling_mode_t ADC_GetSamplingMode(void);

#endif /* _APP_ADC_H_ */
