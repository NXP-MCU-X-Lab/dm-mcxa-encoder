/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#include "fsl_debug_console.h"
#include "fsl_lpadc.h"
#include "fsl_inputmux.h"
#include "fsl_clock.h"

#include "app_adc.h"

/* Simple ADC driver for V2 inductive encoder hardware:
 * - Configures ADC0/ADC1 and trigger routing
 * - Sets up chained conversion sequences for A1/A2 channels
 * - Provides a single read function returning the four raw lane values
 */

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/**
 * @brief Initialize single ADC peripheral
 */
static void adc_init_peripheral(ADC_Type *base)
{
    lpadc_config_t config;

    LPADC_GetDefaultConfig(&config);
    config.enableAnalogPreliminary = true;
    config.powerLevelMode = kLPADC_PowerLevelAlt4;
    config.referenceVoltageSource = kLPADC_ReferenceVoltageAlt3; /* VDDA */
    config.conversionAverageMode = kLPADC_ConversionAverage1024;

    LPADC_Init(base, &config);
    LPADC_DoAutoCalibration(base);
}

/**
 * @brief Configure command for signal channels
 */
static void adc_config_signal_cmd(ADC_Type *base, uint32_t cmdId, uint32_t channel, uint32_t chainNext)
{
    lpadc_conv_command_config_t cmdConfig;

    LPADC_GetDefaultConvCommandConfig(&cmdConfig);
    cmdConfig.channelNumber = channel;
    cmdConfig.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    cmdConfig.hardwareAverageMode = ADC_HW_AVG_SIGNAL;
    cmdConfig.sampleTimeMode = ADC_SAMPLE_TIME_SIGNAL;
    cmdConfig.chainedNextCommandNumber = chainNext;

    LPADC_SetConvCommandConfig(base, cmdId, &cmdConfig);
}

/**
 * @brief Configure ADC conversion sequences
 *
 * ADC0: A1_SIN -> A2_COS
 * ADC1: A1_COS -> A2_SIN
 */
static void adc_configure_sequences(void)
{
    lpadc_conv_trigger_config_t triggerConfig;

    adc_config_signal_cmd(ADC0, ADC_CMD_NORMAL_A, ADC_CH_A1_SIN, ADC_CMD_NORMAL_B);
    adc_config_signal_cmd(ADC0, ADC_CMD_NORMAL_B, ADC_CH_A2_COS, 0U);
    adc_config_signal_cmd(ADC1, ADC_CMD_NORMAL_A, ADC_CH_A1_COS, ADC_CMD_NORMAL_B);
    adc_config_signal_cmd(ADC1, ADC_CMD_NORMAL_B, ADC_CH_A2_SIN, 0U);

    LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
    triggerConfig.targetCommandId = ADC_CMD_NORMAL_A;
    triggerConfig.enableHardwareTrigger = true;

    LPADC_SetConvTriggerConfig(ADC0, ADC_TRIGGER_ID, &triggerConfig);
    LPADC_SetConvTriggerConfig(ADC1, ADC_TRIGGER_ID, &triggerConfig);

    INPUTMUX_Init(INPUTMUX0);
    INPUTMUX_AttachSignal(INPUTMUX0, ADC_TRIGGER_ID, kINPUTMUX_ArmTxevToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0, ADC_TRIGGER_ID, kINPUTMUX_ArmTxevToAdc1Trigger);
    INPUTMUX_Deinit(INPUTMUX0);
}

/**
 * @brief Wait and read single ADC result from FIFO
 */
static inline uint16_t adc_read_fifo(ADC_Type *base)
{
    uint32_t tmp;
    while (1) {
        tmp = base->RESFIFO;
        if (tmp & ADC_RESFIFO_VALID_MASK) {
            return (uint16_t)(tmp & ADC_RESFIFO_D_MASK);
        }
    }
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void adc_init(void)
{
    RESET_ReleasePeripheralReset(kADC0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kADC1_RST_SHIFT_RSTn);

    CLOCK_SetClockDiv(kCLOCK_DivADC, ADC_CLK_DIV);
    CLOCK_AttachClk(kFRO_HF_to_ADC);

    adc_init_peripheral(ADC0);
    adc_init_peripheral(ADC1);

    adc_configure_sequences();

    PRINTF("=== V2 Raw ADC Init ===\r\n");
    PRINTF("ADC0 clock: %d Hz\r\n", CLOCK_GetAdcClkFreq(0));
    PRINTF("ADC1 clock: %d Hz\r\n", CLOCK_GetAdcClkFreq(1));
    PRINTF("A1 SIN: internal OPAMP0_OUT -> ADC0_A2\r\n");
    PRINTF("A1 COS: internal OPAMP1_OUT -> ADC1_A2\r\n");
    PRINTF("A2 SIN: external TLV9062 A2_OPA0_OUT -> ADC1_A3\r\n");
    PRINTF("A2 COS: external TLV9062 A2_OPA1_OUT -> ADC0_A7\r\n");
}

adc_sample_result_t adc_read(void)
{
    adc_sample_result_t result;

    __SEV();

    result.a1_sin_raw = adc_read_fifo(ADC0);
    result.a2_cos_raw = adc_read_fifo(ADC0);
    result.a1_cos_raw = adc_read_fifo(ADC1);
    result.a2_sin_raw = adc_read_fifo(ADC1);

    return result;
}
