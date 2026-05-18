/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#include "fsl_lpuart.h"
#include "freemaster.h"
#include "freemaster_serial_lpuart.h"

#include "app_encoder.h"
#include "app_encoder_runtime.h"
#include "app_freemaster.h"

volatile uint8_t fm_cal_enable;
volatile uint8_t fm_cal_done;
volatile uint16_t fm_cal_progress;
volatile uint8_t fm_cal_state;
volatile uint32_t fm_cal_status;
volatile uint8_t fm_reset_ctrl;
volatile uint8_t fm_zero_ctrl;
volatile uint8_t fm_encoder_valid;
volatile uint32_t fm_encoder_status;



// clang-format off

/****************************************************************************
 *
 * With TSA enabled, the user describes the global and static variables using
 * so-called TSA tables. There can be any number of tables defined in
 * the project files. Each table does have the identifier which should be
 * unique across the project.
 *
 * Note that you can declare variables as Read-Only or Read-Write.
 * The FreeMASTER driver denies any write access to the Read-Only variables
 * when TSA_SAFETY is enabled.
 */

FMSTR_TSA_TABLE_BEGIN(first_table)
    // ========== Simple Variables ==========
    FMSTR_TSA_RW_VAR(fm_cal_enable, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_cal_done, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_cal_progress, FMSTR_TSA_UINT16)
    FMSTR_TSA_RO_VAR(fm_cal_state, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_cal_status, FMSTR_TSA_UINT32)
    FMSTR_TSA_RW_VAR(fm_reset_ctrl, FMSTR_TSA_UINT8)
    FMSTR_TSA_RW_VAR(fm_zero_ctrl, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_encoder_valid, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_encoder_status, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(adc_sample_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(adc_overrun_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_sample_rate_hz, FMSTR_TSA_UINT32)

    // ========== ADC Result Structure ==========
    FMSTR_TSA_STRUCT(adc_sample_result_t)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a1_sin_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a1_cos_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a2_sin_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a2_cos_raw, FMSTR_TSA_UINT16)

    // ========== Global Variables ==========
    FMSTR_TSA_RO_VAR(adc_result, FMSTR_TSA_USERTYPE(adc_sample_result_t))

    // ========== Encoder Debug Result ==========
    FMSTR_TSA_STRUCT(encoder_result_t)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg_raw, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg_filtered, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_counts, FMSTR_TSA_UINT32)
        FMSTR_TSA_MEMBER(encoder_result_t, phase16_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, phase15_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, coarse_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, mag16, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, mag15, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, status, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_result, FMSTR_TSA_USERTYPE(encoder_result_t))

    // ========== Encoder Diagnostic Result ==========
    FMSTR_TSA_STRUCT(encoder_diag_t)
        FMSTR_TSA_MEMBER(encoder_diag_t, mapping, FMSTR_TSA_UINT8)
        FMSTR_TSA_MEMBER(encoder_diag_t, dir16, FMSTR_TSA_SINT8)
        FMSTR_TSA_MEMBER(encoder_diag_t, dir15, FMSTR_TSA_SINT8)
        FMSTR_TSA_MEMBER(encoder_diag_t, angle_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_diag_t, coarse_angle_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_diag_t, phase16_error_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_diag_t, phase15_error_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_diag_t, phase_a1_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_diag_t, phase_a2_deg, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(encoder_diag, FMSTR_TSA_USERTYPE(encoder_diag_t))

    // ========== Calibration Flat View ==========
    FMSTR_TSA_RO_VAR(encoder_cal_flat, FMSTR_TSA_FLOAT)

FMSTR_TSA_TABLE_END()







FMSTR_TSA_TABLE_LIST_BEGIN()
FMSTR_TSA_TABLE(first_table)
FMSTR_TSA_TABLE_LIST_END()


/****************************************************************************
 * FreeMASTER application initialization */

void AppFreemaster_Init(void)
{
    FMSTR_SerialSetBaseAddress((LPUART_Type *)LPUART0);
    FMSTR_Init();
    fm_cal_enable = 0;
    fm_cal_done = 0;
    fm_cal_progress = 0;
    fm_cal_state = 0;
    fm_cal_status = 0;
    fm_reset_ctrl = 0;
    fm_zero_ctrl = 0;
    fm_encoder_valid = 0;
    fm_encoder_status = 0;
    adc_sample_count = 0;
    adc_overrun_count = 0;
    encoder_sample_rate_hz = ADC_SAMPLE_RATE_HZ;

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    NVIC_SetPriority(LPUART0_IRQn, 3U);
    EnableIRQ(LPUART0_IRQn);
#endif
}


