/*
 * Copyright (c) 2023
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _APP_ADC_H_
#define _APP_ADC_H_

/* ADC interface for inductive encoder application.
 * Provides channel definitions, sampling modes, result structure, and APIs
 * for initialization and reading OPAMP outputs (and temperature).
 */

#include "fsl_lpadc.h"
#include <stdbool.h>

/* Pin Mapping:
 * OPAMP0_OUT  -> P2_15 (ADC0_A2)
 * OPAMP1_OUT  -> P2_19 (ADC1_A2)
 * Temperature -> ADC0_A26
 */

/* ADC Channel Definitions */
#define ADC_CH_OPAMP0_OUT       2U   /* ADC0 */
#define ADC_CH_OPAMP1_OUT       2U   /* ADC1 */
#define ADC_CH_TEMPERATURE      26U  /* ADC0 */

/* Command IDs */
#define ADC_CMD_NORMAL_SEQ      4U   /* Normal sequence: outputs only */
#define ADC_CMD_TEMP_SEQ_CH0    5U   /* Temperature sequence: CH0 */
#define ADC_CMD_TEMP_SEQ_CH1    6U   /* Temperature sequence: CH1 (chained) */

/* Trigger Configuration */
#define ADC_TRIGGER_ID          0U

/* Hardware Averaging for Signal Channels */
#define ADC_HW_AVG_SIGNAL       kLPADC_HardwareAverageCount8
#define ADC_SAMPLE_TIME_SIGNAL  kLPADC_SampleTimeADCK5
#define ADC_CLK_DIV             3

/* Temperature Sensor Configuration */
#define TEMP_SAMPLE_INTERVAL    (10*1000)   /* Sample temperature every 10000 normal reads */
#define TEMP_LOOP_COUNT         3U          /* 4 samples total (first 2 discarded) */



/* ADC Sample Result Structure */
typedef struct {
    /* OPAMP outputs */
    uint16_t opamp0_out;
    uint16_t opamp1_out;

    /* Voltage conversions */
    float opamp0_out_voltage;
    float opamp1_out_voltage;

    /* Temperature */
    float temperature;          /* Celsius */
} adc_sample_result_t;

/* Function Prototypes */

/**
 * @brief Initialize ADC for OPAMP outputs and temperature sampling
 */
void adc_init(void);

/**
 * @brief Read ADC samples (automatically handles temperature sampling)
 * @return adc_sample_result_t Structure containing all sampled values
 * @note Temperature is sampled every TEMP_SAMPLE_INTERVAL calls
 */
adc_sample_result_t adc_read(void);


#endif /* _APP_ADC_H_ */
