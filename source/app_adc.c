/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#include "fsl_debug_console.h"
#include "fsl_lpadc.h"
#include "fsl_inputmux.h"
#include "fsl_clock.h"
#include "fsl_ctimer.h"
#include "fsl_reset.h"

#include "app_adc.h"
#include "hardware_init.h"

/* Simple ADC driver for dual-track inductive encoder hardware:
 * - Configures ADC0/ADC1 and trigger routing
 * - Sets up chained conversion sequences for A1/A2 channels
 * - Uses CTIMER0 to trigger deterministic realtime sampling
 */

#define ADC_TIMER               CTIMER0
#define ADC_TIMER_ID            0U
#define ADC_TIMER_MATCH         kCTIMER_Match_0
#define ADC_IRQ_PRIORITY        1U
#define ADC_TRIGGER_ID          0U
#define ADC_CMD_NORMAL_A        1U
#define ADC_CMD_NORMAL_B        2U
#define ADC_CH_A1_SIN           2U
#define ADC_CH_A1_COS           2U
#define ADC_CH_A2_SIN           3U
#define ADC_CH_A2_COS           7U
#define ADC_HW_AVG_SIGNAL       kLPADC_HardwareAverageCount8
#define ADC_SAMPLE_TIME_SIGNAL  kLPADC_SampleTimeADCK5
#define ADC_CLK_DIV             3

typedef struct {
    bool valid;
    uint16_t first;
    uint16_t second;
} adc_pair_result_t;

static adc_sample_callback_t s_sample_callback;
static volatile uint32_t s_sample_count;
static volatile uint32_t s_overrun_count;
static adc_pair_result_t s_adc0_pair;
static adc_pair_result_t s_adc1_pair;

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static uint32_t adc_irq_mask(void)
{
    return (uint32_t)kLPADC_FIFO0WatermarkInterruptEnable |
           (uint32_t)kLPADC_ResultFIFO0OverflowInterruptEnable;
}

static uint32_t adc_result_count(ADC_Type *base)
{
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
    return LPADC_GetConvResultCount(base, 0U);
#else
    return LPADC_GetConvResultCount(base);
#endif
}

static bool adc_get_result(ADC_Type *base, lpadc_conv_result_t *result)
{
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
    return LPADC_GetConvResult(base, result, 0U);
#else
    return LPADC_GetConvResult(base, result);
#endif
}

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
#if (defined(FSL_FEATURE_LPADC_FIFO_COUNT) && (FSL_FEATURE_LPADC_FIFO_COUNT == 2))
    config.FIFO0Watermark = 1U;
    config.FIFO1Watermark = 1U;
#else
    config.FIFOWatermark = 1U;
#endif

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
    INPUTMUX_AttachSignal(INPUTMUX0, ADC_TRIGGER_ID, kINPUTMUX_Ctimer0M0ToAdc0Trigger);
    INPUTMUX_AttachSignal(INPUTMUX0, ADC_TRIGGER_ID, kINPUTMUX_Ctimer0M0ToAdc1Trigger);
    INPUTMUX_Deinit(INPUTMUX0);
}

static void adc_configure_timer(void)
{
    ctimer_config_t timerConfig;
    ctimer_match_config_t matchConfig;
    uint32_t timerClockHz;
    uint32_t matchValue;

    CLOCK_SetClockDiv(kCLOCK_DivCTIMER0, 1U);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);

    CTIMER_GetDefaultConfig(&timerConfig);
    CTIMER_Init(ADC_TIMER, &timerConfig);

    timerClockHz = CLOCK_GetCTimerClkFreq(ADC_TIMER_ID);
    matchValue = (timerClockHz + (ADC_SAMPLE_RATE_HZ / 2U)) / ADC_SAMPLE_RATE_HZ;
    if (matchValue > 0U)
    {
        matchValue -= 1U;
    }

    matchConfig.matchValue = matchValue;
    matchConfig.enableCounterReset = true;
    matchConfig.enableCounterStop = false;
    matchConfig.outControl = kCTIMER_Output_Toggle;
    matchConfig.outPinInitState = false;
    matchConfig.enableInterrupt = false;
    CTIMER_SetupMatch(ADC_TIMER, ADC_TIMER_MATCH, &matchConfig);
}

static void adc_reset_realtime_state(void)
{
    s_adc0_pair.valid = false;
    s_adc1_pair.valid = false;
    s_sample_count = 0U;
    s_overrun_count = 0U;
    LPADC_DoResetFIFO(ADC0);
    LPADC_DoResetFIFO(ADC1);
}

static void adc_publish_if_complete(void)
{
    adc_sample_result_t sample;

    if (!s_adc0_pair.valid || !s_adc1_pair.valid)
    {
        return;
    }

    sample.a1_sin_raw = s_adc0_pair.first;
    sample.a2_cos_raw = s_adc0_pair.second;
    sample.a1_cos_raw = s_adc1_pair.first;
    sample.a2_sin_raw = s_adc1_pair.second;

    s_adc0_pair.valid = false;
    s_adc1_pair.valid = false;
    s_sample_count++;

    if (s_sample_callback != NULL)
    {
        TestPin_Set();
        s_sample_callback(&sample);
        TestPin_Clear();
    }
}

static void adc_store_pair(ADC_Type *base, adc_pair_result_t *pair)
{
    lpadc_conv_result_t first;
    lpadc_conv_result_t second;

    if (adc_result_count(base) < 2U)
    {
        return;
    }

    if (!adc_get_result(base, &first) || !adc_get_result(base, &second))
    {
        s_overrun_count++;
        return;
    }

    if (pair->valid)
    {
        s_overrun_count++;
    }

    pair->first = first.convValue;
    pair->second = second.convValue;
    pair->valid = true;
    adc_publish_if_complete();
}

static void adc_handle_irq(ADC_Type *base, adc_pair_result_t *pair)
{
    const uint32_t flags = LPADC_GetStatusFlags(base);

    if ((flags & (uint32_t)kLPADC_ResultFIFO0OverflowFlag) != 0U)
    {
        s_overrun_count++;
        LPADC_ClearStatusFlags(base, (uint32_t)kLPADC_ResultFIFO0OverflowFlag);
        LPADC_DoResetFIFO(base);
        pair->valid = false;
        return;
    }

    adc_store_pair(base, pair);
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

    /* Let VDDA and analog frontend settle before auto-calibration so the
     * captured offset trim is not contaminated by power-on transients. */
    delay_ms(100U);

    adc_init_peripheral(ADC0);
    adc_init_peripheral(ADC1);

    adc_configure_sequences();
    adc_configure_timer();
    adc_reset_realtime_state();

    PRINTF("=== Encoder Raw ADC Init ===\r\n");
    PRINTF("ADC0 clock: %d Hz\r\n", CLOCK_GetAdcClkFreq(0));
    PRINTF("ADC1 clock: %d Hz\r\n", CLOCK_GetAdcClkFreq(1));
    PRINTF("ADC realtime sample rate: %d Hz\r\n", ADC_SAMPLE_RATE_HZ);
    PRINTF("A1 SIN: internal OPAMP0_OUT -> ADC0_A2\r\n");
    PRINTF("A1 COS: internal OPAMP1_OUT -> ADC1_A2\r\n");
    PRINTF("A2 SIN: external TLV9062 A2_OPA0_OUT -> ADC1_A3\r\n");
    PRINTF("A2 COS: external TLV9062 A2_OPA1_OUT -> ADC0_A7\r\n");
}

void adc_set_sample_callback(adc_sample_callback_t callback)
{
    s_sample_callback = callback;
}

void adc_realtime_start(void)
{
    adc_reset_realtime_state();

    NVIC_SetPriority(ADC0_IRQn, ADC_IRQ_PRIORITY);
    NVIC_SetPriority(ADC1_IRQn, ADC_IRQ_PRIORITY);
    EnableIRQ(ADC0_IRQn);
    EnableIRQ(ADC1_IRQn);

    LPADC_EnableInterrupts(ADC0, adc_irq_mask());
    LPADC_EnableInterrupts(ADC1, adc_irq_mask());
    CTIMER_StartTimer(ADC_TIMER);
}

void adc_realtime_stop(void)
{
    CTIMER_StopTimer(ADC_TIMER);
    LPADC_DisableInterrupts(ADC0, adc_irq_mask());
    LPADC_DisableInterrupts(ADC1, adc_irq_mask());
    DisableIRQ(ADC0_IRQn);
    DisableIRQ(ADC1_IRQn);
}

uint32_t adc_get_sample_count(void)
{
    return s_sample_count;
}

uint32_t adc_get_overrun_count(void)
{
    return s_overrun_count;
}

void ADC0_IRQHandler(void)
{
    adc_handle_irq(ADC0, &s_adc0_pair);
}

void ADC1_IRQHandler(void)
{
    adc_handle_irq(ADC1, &s_adc1_pair);
}
