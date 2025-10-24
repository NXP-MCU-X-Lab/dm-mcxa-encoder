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
#include "fsl_reset.h"
#include "app_adc_sample.h"
#include <string.h>
#include <stdio.h>

/* ADC Hardware Average Configuration */
#define ADC_HW_AVERAGE_MODE         kLPADC_HardwareAverageCount16
#define ADC_HW_SAMPLE_TIME_MODE     kLPADC_SampleTimeADCK3
#define ADC_CLK_DIV                 (3)

/* Global sampling mode */
static adc_sampling_mode_t g_adc_mode = ADC_MODE_OUTPUT_ONLY;

/* Conversion command configuration */
static lpadc_conv_trigger_config_t triggerConfig;

/**
 * @brief Configure ADC conversion commands based on sampling mode
 * 
 * Mode 1 (OUTPUT_ONLY): Sample OPAMP0_OUT, OPAMP1_OUT (2 channels)
 * Mode 2 (FULL_DEBUG):  Sample all 6 channels (INP, INN, OUT for both OPAMPs)
 * Mode 3 (CALIBRATION): Same as Mode 1
 */
/* old ConfigureConversion removed; using adc_configure_conversion */


static void adc_configure_conversion(void)
{
    lpadc_conv_command_config_t cmdConfig;
    
    /* ========== Common Configuration Template ========== */
    LPADC_GetDefaultConvCommandConfig(&cmdConfig);
    cmdConfig.conversionResolutionMode = kLPADC_ConversionResolutionHigh;
    cmdConfig.hardwareAverageMode = ADC_HW_AVERAGE_MODE;
    cmdConfig.sampleTimeMode = ADC_HW_SAMPLE_TIME_MODE;
    
    if (g_adc_mode == ADC_MODE_FULL_DEBUG) {
        /* ========== Mode 2: Full Debug (6 channels) ========== */
        
        /* ADC0 Command Chain: A5 -> A6 -> A2 */
        cmdConfig.channelNumber = ADC0_CHANNEL_A5;  // OPAMP0_INP
        cmdConfig.chainedNextCommandNumber = ADC_CMD_ID_CH1;
        LPADC_SetConvCommandConfig(ADC0, ADC_CMD_ID_CH0, &cmdConfig);
        
        cmdConfig.channelNumber = ADC0_CHANNEL_A6;  // OPAMP1_INP
        cmdConfig.chainedNextCommandNumber = ADC_CMD_ID_CH2;
        LPADC_SetConvCommandConfig(ADC0, ADC_CMD_ID_CH1, &cmdConfig);
        
        cmdConfig.channelNumber = ADC0_CHANNEL_A2;  // OPAMP0_OUT
        cmdConfig.chainedNextCommandNumber = 0U;
        LPADC_SetConvCommandConfig(ADC0, ADC_CMD_ID_CH2, &cmdConfig);
        
        /* ADC1 Command Chain: A5 -> A6 -> A2 */
        cmdConfig.channelNumber = ADC1_CHANNEL_A5;  // OPAMP0_INN
        cmdConfig.chainedNextCommandNumber = ADC_CMD_ID_CH1;
        LPADC_SetConvCommandConfig(ADC1, ADC_CMD_ID_CH0, &cmdConfig);
        
        cmdConfig.channelNumber = ADC1_CHANNEL_A6;  // OPAMP1_INN
        cmdConfig.chainedNextCommandNumber = ADC_CMD_ID_CH2;
        LPADC_SetConvCommandConfig(ADC1, ADC_CMD_ID_CH1, &cmdConfig);
        
        cmdConfig.channelNumber = ADC1_CHANNEL_A2;  // OPAMP1_OUT
        cmdConfig.chainedNextCommandNumber = 0U;
        LPADC_SetConvCommandConfig(ADC1, ADC_CMD_ID_CH2, &cmdConfig);
        
    } else {
        /* ========== Mode 1/3: Output Only (2 channels) ========== */
        
        /* ADC0: Only sample A2 (OPAMP0_OUT) */
        cmdConfig.channelNumber = ADC0_CHANNEL_A2;
        cmdConfig.chainedNextCommandNumber = 0U;
        LPADC_SetConvCommandConfig(ADC0, ADC_CMD_ID_CH0, &cmdConfig);
        
        /* ADC1: Only sample A2 (OPAMP1_OUT) */
        cmdConfig.channelNumber = ADC1_CHANNEL_A2;
        cmdConfig.chainedNextCommandNumber = 0U;
        LPADC_SetConvCommandConfig(ADC1, ADC_CMD_ID_CH0, &cmdConfig);
    }
    
    /* ========== Trigger Configuration ========== */
    LPADC_GetDefaultConvTriggerConfig(&triggerConfig);
    triggerConfig.targetCommandId = ADC_CMD_ID_CH0;
    triggerConfig.enableHardwareTrigger = true;
    LPADC_SetConvTriggerConfig(ADC0, ADC_TRIGGER_ID, &triggerConfig);
    LPADC_SetConvTriggerConfig(ADC1, ADC_TRIGGER_ID, &triggerConfig);
    
    /* Configure INPUTMUX for trigger */
    INPUTMUX_Init(INPUTMUX0);
    INPUTMUX_AttachSignal(INPUTMUX0, ADC_TRIGGER_ID, kINPUTMUX_ArmTxevToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0, ADC_TRIGGER_ID, kINPUTMUX_ArmTxevToAdc1Trigger);
    INPUTMUX_Deinit(INPUTMUX0);
}


void adc_init(adc_sampling_mode_t mode)
{
    lpadc_config_t adcConfigStruct;
    
    /* Save sampling mode */
    g_adc_mode = mode;
    
    /* Release peripheral reset for ADC0 and ADC1 */
    RESET_ReleasePeripheralReset(kADC0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kADC1_RST_SHIFT_RSTn);

    /* Set clock source and divider for ADC0 and ADC1 */
    CLOCK_SetClockDiv(kCLOCK_DivADC, ADC_CLK_DIV);
    CLOCK_AttachClk(kFRO_HF_to_ADC);

    /* Initialize ADC configuration */
    LPADC_GetDefaultConfig(&adcConfigStruct);
    adcConfigStruct.enableAnalogPreliminary = true;
    adcConfigStruct.powerLevelMode = kLPADC_PowerLevelAlt4;
    adcConfigStruct.referenceVoltageSource = kLPADC_ReferenceVoltageAlt3; /* VDDA */
    adcConfigStruct.conversionAverageMode = kLPADC_ConversionAverage1024; 
    
    /* Initialize both ADCs with the same configuration */
    LPADC_Init(ADC0, &adcConfigStruct);
    LPADC_Init(ADC1, &adcConfigStruct);

    /* Perform auto-calibration for both ADCs */
    LPADC_DoAutoCalibration(ADC0);
    LPADC_DoAutoCalibration(ADC1);

    /* Configure conversion commands based on mode */
    adc_configure_conversion();

    /* Get hardware average count as string */
    const char *hwAvgStr;
    switch(ADC_HW_AVERAGE_MODE) {
        case kLPADC_HardwareAverageCount1:  hwAvgStr = "1"; break;
        case kLPADC_HardwareAverageCount2:  hwAvgStr = "2"; break;
        case kLPADC_HardwareAverageCount4:  hwAvgStr = "4"; break;
        case kLPADC_HardwareAverageCount8:  hwAvgStr = "8"; break;
        case kLPADC_HardwareAverageCount16: hwAvgStr = "16"; break;
        case kLPADC_HardwareAverageCount32: hwAvgStr = "32"; break;
        case kLPADC_HardwareAverageCount64: hwAvgStr = "64"; break;
        case kLPADC_HardwareAverageCount128: hwAvgStr = "128"; break;
        default: hwAvgStr = "?"; break;
    }

    const char *modeStr;
    switch(g_adc_mode) {
        case ADC_MODE_OUTPUT_ONLY:   modeStr = "Output Only (2ch)"; break;
        case ADC_MODE_FULL_DEBUG:    modeStr = "Full Debug (6ch)"; break;
        case ADC_MODE_CALIBRATION:   modeStr = "Calibration (2ch)"; break;
        default: modeStr = "Unknown"; break;
    }

    PRINTF("=== Magnetic Encoder ADC Init ===\r\n");
    PRINTF("ADC0 clock: %d Hz\r\n", CLOCK_GetAdcClkFreq(0));
    PRINTF("ADC1 clock: %d Hz\r\n", CLOCK_GetAdcClkFreq(1));
    PRINTF("Hardware average: %sx\r\n", hwAvgStr);
    PRINTF("Resolution: 16-bit\r\n");
    PRINTF("Sampling mode: %s\r\n", modeStr);
    PRINTF("OPAMP0: INP(P2_12) INN(P2_13) OUT(P2_15)\r\n");
    PRINTF("OPAMP1: INP(P2_16) INN(P2_17) OUT(P2_19)\r\n");
}

/**
 * @brief Start conversion and wait for results using direct register access
 * 
 * In OUTPUT_ONLY mode: Only reads OPAMP0_OUT and OPAMP1_OUT (INP/INN = 0)
 * In FULL_DEBUG mode:  Reads all 6 channels
 */

adc_sample_result_t adc_read(void)
{
    __SEV();  // Trigger both ADCs simultaneously
    
    adc_sample_result_t result = {0};
    uint32_t tmp32;
    
    if (g_adc_mode == ADC_MODE_FULL_DEBUG) {
        /* ========== Mode 2: Read all 6 channels ========== */
        
        /* ADC0: A5 (OPAMP0_INP) -> A6 (OPAMP1_INP) -> A2 (OPAMP0_OUT) */
        while (1) {
            tmp32 = ADC0->RESFIFO;
            if ((tmp32 & ADC_RESFIFO_VALID_MASK) == ADC_RESFIFO_VALID_MASK) {
                break;
            }
        }
        result.opamp0_inp = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);
        
        while (1) {
            tmp32 = ADC0->RESFIFO;
            if ((tmp32 & ADC_RESFIFO_VALID_MASK) == ADC_RESFIFO_VALID_MASK) {
                break;
            }
        }
        result.opamp1_inp = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);
        
        while (1) {
            tmp32 = ADC0->RESFIFO;
            if ((tmp32 & ADC_RESFIFO_VALID_MASK) == ADC_RESFIFO_VALID_MASK) {
                break;
            }
        }
        result.opamp0_out = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);
        
        /* ADC1: A5 (OPAMP0_INN) -> A6 (OPAMP1_INN) -> A2 (OPAMP1_OUT) */
        while (1) {
            tmp32 = ADC1->RESFIFO;
            if ((tmp32 & ADC_RESFIFO_VALID_MASK) == ADC_RESFIFO_VALID_MASK) {
                break;
            }
        }
        result.opamp0_inn = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);
        
        while (1) {
            tmp32 = ADC1->RESFIFO;
            if ((tmp32 & ADC_RESFIFO_VALID_MASK) == ADC_RESFIFO_VALID_MASK) {
                break;
            }
        }
        result.opamp1_inn = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);
        
        while (1) {
            tmp32 = ADC1->RESFIFO;
            if ((tmp32 & ADC_RESFIFO_VALID_MASK) == ADC_RESFIFO_VALID_MASK) {
                break;
            }
        }
        result.opamp1_out = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);
        
    } else {
        /* ========== Mode 1/3: Read only outputs (2 channels) ========== */
        
        /* ADC0: A2 (OPAMP0_OUT) */
        while (1) {
            tmp32 = ADC0->RESFIFO;
            if ((tmp32 & ADC_RESFIFO_VALID_MASK) == ADC_RESFIFO_VALID_MASK) {
                break;
            }
        }
        result.opamp0_out = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);
        
        /* ADC1: A2 (OPAMP1_OUT) */
        while (1) {
            tmp32 = ADC1->RESFIFO;
            if ((tmp32 & ADC_RESFIFO_VALID_MASK) == ADC_RESFIFO_VALID_MASK) {
                break;
            }
        }
        result.opamp1_out = (uint16_t)(tmp32 & ADC_RESFIFO_D_MASK);
        
        /* INP/INN remain 0 */
    }
    
    return result;
}


adc_sampling_mode_t adc_get_mode(void)
{
    return g_adc_mode;
}
