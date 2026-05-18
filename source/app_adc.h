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
 */

/* ADC Channel Definitions */
#define ADC_CH_A1_SIN           2U   /* ADC0_A2 */
#define ADC_CH_A1_COS           2U   /* ADC1_A2 */
#define ADC_CH_A2_SIN           3U   /* ADC1_A3 */
#define ADC_CH_A2_COS           7U   /* ADC0_A7 */

/* Command IDs */
#define ADC_CMD_NORMAL_A        1U
#define ADC_CMD_NORMAL_B        2U

/* Trigger Configuration */
#define ADC_TRIGGER_ID          0U

/* Hardware Averaging for Signal Channels */
#define ADC_HW_AVG_SIGNAL       kLPADC_HardwareAverageCount2
#define ADC_SAMPLE_TIME_SIGNAL  kLPADC_SampleTimeADCK5
#define ADC_CLK_DIV             3



/* ADC Sample Result Structure */
typedef struct {
    uint16_t a1_sin_raw;
    uint16_t a1_cos_raw;
    uint16_t a2_sin_raw;
    uint16_t a2_cos_raw;
} adc_sample_result_t;

/* Function Prototypes */

/**
 * @brief Initialize ADC for V2 raw lane sampling.
 */
void adc_init(void);

/**
 * @brief Read V2 raw ADC samples for both lanes.
 * @return adc_sample_result_t Structure containing the four raw channel values
 */
adc_sample_result_t adc_read(void);


#endif /* _APP_ADC_H_ */
