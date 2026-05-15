/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#ifndef _APP_ADC_H_
#define _APP_ADC_H_

/* ADC interface for V2 inductive encoder hardware.
 * Provides channel definitions, sampling modes, result structure, and APIs
 * for reading both encoder lanes as raw ADC values.
 */

#include "fsl_lpadc.h"
#include <stdbool.h>

/* V2 Pin Mapping:
 * A1_SIN: MCU OPAMP0_OUT -> P2_15 (ADC0_A2)
 * A1_COS: MCU OPAMP1_OUT -> P2_19 (ADC1_A2)
 * A2_SIN: TLV9062 A2_OPA0_OUT -> P2_6 (ADC1_A3)
 * A2_COS: TLV9062 A2_OPA1_OUT -> P2_7 (ADC0_A7)
 * Temperature -> ADC0_A26
 */

/* ADC Channel Definitions */
#define ADC_CH_A1_SIN           2U   /* ADC0_A2 */
#define ADC_CH_A1_COS           2U   /* ADC1_A2 */
#define ADC_CH_A2_SIN           3U   /* ADC1_A3 */
#define ADC_CH_A2_COS           7U   /* ADC0_A7 */
#define ADC_CH_TEMPERATURE      26U  /* ADC0 */

/* Command IDs */
#define ADC_CMD_NORMAL_A        1U
#define ADC_CMD_NORMAL_B        2U
#define ADC_CMD_TEMP_A          3U
#define ADC_CMD_TEMP_SENSOR     4U
#define ADC_CMD_TEMP_B          5U

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
    uint16_t opamp0_out;        /* V1 alias: A1_SIN */
    uint16_t opamp1_out;        /* V1 alias: A1_COS */

    uint16_t a1_sin_raw;
    uint16_t a1_cos_raw;
    uint16_t a2_sin_raw;
    uint16_t a2_cos_raw;

    float opamp0_out_voltage;   /* V1 alias: A1_SIN voltage */
    float opamp1_out_voltage;   /* V1 alias: A1_COS voltage */

    float a1_sin_voltage;
    float a1_cos_voltage;
    float a2_sin_voltage;
    float a2_cos_voltage;

    float temperature;          /* Celsius */
} adc_sample_result_t;

/* Function Prototypes */

/**
 * @brief Initialize ADC for V2 raw lane sampling and temperature sampling.
 */
void adc_init(void);

/**
 * @brief Read V2 raw ADC samples (automatically handles temperature sampling).
 * @return adc_sample_result_t Structure containing all sampled values
 * @note Temperature is sampled every TEMP_SAMPLE_INTERVAL calls
 */
adc_sample_result_t adc_read(void);


#endif /* _APP_ADC_H_ */
