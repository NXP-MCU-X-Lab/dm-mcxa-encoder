/*
 * Copyright (c) 2023
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_lpadc.h"
#include "fsl_inputmux.h"
#include "fsl_clock.h"
#include <string.h>
#include <stdio.h>

#include "app_adc.h"

/* Simple ADC driver for inductive encoder:
 * - Configures ADCs and trigger routing
 * - Sets up conversion sequences for OPAMP outputs and temperature
 * - Provides a single read function returning structured results
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/



/* Hardware Averaging for Temperature Channel (Must be 128x) */
#define ADC_HW_AVG_TEMP         kLPADC_HardwareAverageCount128
#define ADC_SAMPLE_TIME_TEMP    kLPADC_SampleTimeADCK131


/*******************************************************************************
 * Variables
 ******************************************************************************/

static adc_sampling_mode_t g_adc_mode = ADC_MODE_OUTPUT_ONLY;
static uint32_t g_sample_counter = 0;
static float g_last_temperature = 0;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/**
 * @brief Initialize single ADC peripheral
 * @param base ADC peripheral base address
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
 * @brief Configure command for signal channels (OPAMP outputs)
 * @param base ADC peripheral base address
 * @param cmdId Command ID
 * @param channel Channel number
 * @param chainNext Next command ID (0 = end of chain)
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
 * @brief Configure command for temperature channel (dual-VBE measurement)
 * @param base ADC peripheral base address (must be ADC0)
 * @param cmdId Command ID
 * @param chainNext Next command ID (0 = end of chain)
 */
static void adc_config_temp_cmd(ADC_Type *base, uint32_t cmdId, uint32_t chainNext)
{
    lpadc_conv_command_config_t cmdConfig;
    
    LPADC_GetDefaultConvCommandConfig(&cmdConfig);
    cmdConfig.channelNumber = ADC_CH_TEMPERATURE;
    cmdConfig.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    cmdConfig.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    cmdConfig.hardwareAverageMode = ADC_HW_AVG_TEMP;
    cmdConfig.sampleTimeMode = ADC_SAMPLE_TIME_TEMP;
    cmdConfig.loopCount = TEMP_LOOP_COUNT;  /* 4 samples (index 0-3) */
    cmdConfig.chainedNextCommandNumber = chainNext;
    
    LPADC_SetConvCommandConfig(base, cmdId, &cmdConfig);
}


/**
 * @brief Configure ADC conversion sequences
 */
static void adc_configure_sequences(void)
{
    lpadc_conv_trigger_config_t triggerConfig;
    
    if (g_adc_mode == ADC_MODE_FULL_DEBUG) {
        /* Full Debug Mode: 6 Channels (no temperature) */
        adc_config_signal_cmd(ADC0, ADC_CMD_NORMAL_SEQ, ADC_CH_OPAMP0_INP, ADC_CMD_TEMP_SEQ_CH0);
        adc_config_signal_cmd(ADC0, ADC_CMD_TEMP_SEQ_CH0, ADC_CH_OPAMP1_INP, ADC_CMD_TEMP_SEQ_CH1);
        adc_config_signal_cmd(ADC0, ADC_CMD_TEMP_SEQ_CH1, ADC_CH_OPAMP0_OUT, 0U);
        
        adc_config_signal_cmd(ADC1, ADC_CMD_NORMAL_SEQ, ADC_CH_OPAMP0_INN, ADC_CMD_TEMP_SEQ_CH0);
        adc_config_signal_cmd(ADC1, ADC_CMD_TEMP_SEQ_CH0, ADC_CH_OPAMP1_INN, ADC_CMD_TEMP_SEQ_CH1);
        adc_config_signal_cmd(ADC1, ADC_CMD_TEMP_SEQ_CH1, ADC_CH_OPAMP1_OUT, 0U);
        
    } else {
        /* Output Only Mode with Temperature Support */
        
        /* Normal Sequence: OUT0, OUT1 */
        adc_config_signal_cmd(ADC0, ADC_CMD_NORMAL_SEQ, ADC_CH_OPAMP0_OUT, 0U);
        adc_config_signal_cmd(ADC1, ADC_CMD_NORMAL_SEQ, ADC_CH_OPAMP1_OUT, 0U);
        
        /* Temperature Sequence: OUT0 -> TEMP(�4) -> OUT1 */
        adc_config_signal_cmd(ADC0, ADC_CMD_TEMP_SEQ_CH0, ADC_CH_OPAMP0_OUT, ADC_CMD_TEMP_SEQ_CH1);
        adc_config_temp_cmd(ADC0, ADC_CMD_TEMP_SEQ_CH1, 0U);  /* loopCount=3 ? 4 samples */
        adc_config_signal_cmd(ADC1, ADC_CMD_TEMP_SEQ_CH0, ADC_CH_OPAMP1_OUT, 0U);
    }
    
    /* Configure Triggers */
    LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
    triggerConfig.targetCommandId = ADC_CMD_NORMAL_SEQ;
    triggerConfig.enableHardwareTrigger = true;
    
    LPADC_SetConvTriggerConfig(ADC0, ADC_TRIGGER_ID, &triggerConfig);
    LPADC_SetConvTriggerConfig(ADC1, ADC_TRIGGER_ID, &triggerConfig);
    
    /* Configure INPUTMUX */
    INPUTMUX_Init(INPUTMUX0);
    INPUTMUX_AttachSignal(INPUTMUX0, ADC_TRIGGER_ID, kINPUTMUX_ArmTxevToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0, ADC_TRIGGER_ID, kINPUTMUX_ArmTxevToAdc1Trigger);
    INPUTMUX_Deinit(INPUTMUX0);
}


/**
 * @brief Calculate temperature using dual-VBE method
 * @param vbe1 First VBE reading (raw ADC value)
 * @param vbe8 Second VBE reading (raw ADC value)
 * @return Temperature in Celsius
 * 
 * Formula: T = A * [a*(Vbe8-Vbe1)/(Vbe8 + a*(Vbe8-Vbe1))] - B
 * Where:
 *   A     = Temperature sensor slope
 *   B     = Temperature sensor offset
 *   a     = Alpha coefficient
 *   Vbe1  = First VBE measurement
 *   Vbe8  = Second VBE measurement (different bias current)
 */
static float adc_calculate_temperature(uint16_t vbe1, uint16_t vbe8)
{
    float parameterSlope  = FSL_FEATURE_LPADC_TEMP_PARAMETER_A;
    float parameterOffset = FSL_FEATURE_LPADC_TEMP_PARAMETER_B;
    float parameterAlpha  = FSL_FEATURE_LPADC_TEMP_PARAMETER_ALPHA;
    
    float vbe1_f = (float)vbe1;
    float vbe8_f = (float)vbe8;
    
    /* Calculate: a * (Vbe8 - Vbe1) */
    float numerator = parameterAlpha * (vbe8_f - vbe1_f);
    
    /* Calculate: Vbe8 + a * (Vbe8 - Vbe1) */
    float denominator = vbe8_f + numerator;
    
    /* Avoid division by zero */
    if (denominator < 1.0f) {
        return -273.15f;  /* Return absolute zero on error */
    }
    
    /* Final calculation: T = A * [numerator/denominator] - B */
    float temperature = parameterSlope * (numerator / denominator) - parameterOffset;
    
    return temperature;
}


/**
 * @brief Wait and read single ADC result from FIFO
 * @param base ADC peripheral base address
 * @return 16-bit ADC result
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

/**
 * @brief Convert raw ADC value to voltage
 * @param raw_value 16-bit ADC reading
 * @return Voltage in volts
 */
static inline float adc_raw_to_voltage(uint16_t raw_value)
{
    return (raw_value * 3.3f) / 65536.0f;
}

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

void adc_init(adc_sampling_mode_t mode)
{
    g_adc_mode = mode;
    g_sample_counter = 0;
    
    /* Release peripheral reset */
    RESET_ReleasePeripheralReset(kADC0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kADC1_RST_SHIFT_RSTn);
    
    /* Configure clock */
    CLOCK_SetClockDiv(kCLOCK_DivADC, ADC_CLK_DIV);
    CLOCK_AttachClk(kFRO_HF_to_ADC);
    
    /* Initialize both ADCs */
    adc_init_peripheral(ADC0);
    adc_init_peripheral(ADC1);
    
    /* Configure conversion sequences */
    adc_configure_sequences();
    
    /* Print configuration */
    const char *modeStr = (mode == ADC_MODE_OUTPUT_ONLY) ? "Output Only (2ch)" :
                          (mode == ADC_MODE_FULL_DEBUG) ? "Full Debug (6ch)" :
                          "Calibration (2ch)";
    
    PRINTF("=== Magnetic Encoder ADC Init ===\r\n");
    PRINTF("ADC0 clock: %d Hz\r\n", CLOCK_GetAdcClkFreq(0));
    PRINTF("ADC1 clock: %d Hz\r\n", CLOCK_GetAdcClkFreq(1));
    PRINTF("Signal average: 16x, Resolution: 16-bit\r\n");
    PRINTF("Temperature average: 128x (every %d samples)\r\n", TEMP_SAMPLE_INTERVAL);
    PRINTF("Sampling mode: %s\r\n", modeStr);
    PRINTF("OPAMP0: INP(P2_12) INN(P2_13) OUT(P2_15)\r\n");
    PRINTF("OPAMP1: INP(P2_16) INN(P2_17) OUT(P2_19)\r\n");
}

adc_sample_result_t adc_read(void)
{
    adc_sample_result_t result = {0};
    bool sample_temp = false;
    
    /* Determine if temperature should be sampled */
    if (g_adc_mode != ADC_MODE_FULL_DEBUG) {
        g_sample_counter++;
        if (g_sample_counter >= TEMP_SAMPLE_INTERVAL) {
            sample_temp = true;
            g_sample_counter = 0;
            
            /* Switch to temperature sequence */
            lpadc_conv_trigger_config_t triggerConfig;
            LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
            triggerConfig.targetCommandId = ADC_CMD_TEMP_SEQ_CH0;
            triggerConfig.enableHardwareTrigger = true;
            LPADC_SetConvTriggerConfig(ADC0, ADC_TRIGGER_ID, &triggerConfig);
            LPADC_SetConvTriggerConfig(ADC1, ADC_TRIGGER_ID, &triggerConfig);
        }
    }
    
    /* Trigger conversion */
    __SEV();
    
    /* Read results based on mode */
    if (g_adc_mode == ADC_MODE_FULL_DEBUG) {
        /* Full Debug: 6 channels (no temperature) */
        result.opamp0_inp = adc_read_fifo(ADC0);
        result.opamp1_inp = adc_read_fifo(ADC0);
        result.opamp0_out = adc_read_fifo(ADC0);
        result.opamp0_inn = adc_read_fifo(ADC1);
        result.opamp1_inn = adc_read_fifo(ADC1);
        result.opamp1_out = adc_read_fifo(ADC1);
        
        /* Convert to voltage */
        result.opamp0_inp_voltage = adc_raw_to_voltage(result.opamp0_inp);
        result.opamp0_inn_voltage = adc_raw_to_voltage(result.opamp0_inn);
        result.opamp0_out_voltage = adc_raw_to_voltage(result.opamp0_out);
        result.opamp1_inp_voltage = adc_raw_to_voltage(result.opamp1_inp);
        result.opamp1_inn_voltage = adc_raw_to_voltage(result.opamp1_inn);
        result.opamp1_out_voltage = adc_raw_to_voltage(result.opamp1_out);
        
        result.temperature = g_last_temperature;
        
    } else if (sample_temp) {
        /* Temperature sequence: OUT0 -> [TEMP�4] -> OUT1 */
        result.opamp0_out = adc_read_fifo(ADC0);
        
        /* Read 4 temperature samples (discard first 2) */
        uint16_t temp_samples[4];
        for (int i = 0; i < 4; i++) {
            temp_samples[i] = adc_read_fifo(ADC0);
        }
        
        result.opamp1_out = adc_read_fifo(ADC1);
        
        /* Calculate temperature using samples 2 and 3 (Vbe1 and Vbe8) */
        g_last_temperature = adc_calculate_temperature(temp_samples[2], temp_samples[3]);
        result.temperature = g_last_temperature;
        
        /* Convert to voltage */
        result.opamp0_out_voltage = adc_raw_to_voltage(result.opamp0_out);
        result.opamp1_out_voltage = adc_raw_to_voltage(result.opamp1_out);
        
        /* Restore normal sequence */
        lpadc_conv_trigger_config_t triggerConfig;
        LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
        triggerConfig.targetCommandId = ADC_CMD_NORMAL_SEQ;
        triggerConfig.enableHardwareTrigger = true;
        LPADC_SetConvTriggerConfig(ADC0, ADC_TRIGGER_ID, &triggerConfig);
        LPADC_SetConvTriggerConfig(ADC1, ADC_TRIGGER_ID, &triggerConfig);
        
    } else {
        /* Normal sequence: OUT0, OUT1 */
        result.opamp0_out = adc_read_fifo(ADC0);
        result.opamp1_out = adc_read_fifo(ADC1);
        
        /* Convert to voltage */
        result.opamp0_out_voltage = adc_raw_to_voltage(result.opamp0_out);
        result.opamp1_out_voltage = adc_raw_to_voltage(result.opamp1_out);
        
        result.temperature = g_last_temperature;
    }
    
    return result;
}

