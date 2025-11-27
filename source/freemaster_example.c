/*
 * Copyright (c) 2007-2015 Freescale Semiconductor, Inc.
 * Copyright 2018-2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FreeMASTER Communication Driver - Example Application Code
 */

#include <string.h>
#include <stdio.h>

#include "freemaster.h"
#include "freemaster_example.h"
#include "app_adc.h"
#include "app_encoder.h"

extern adc_sample_result_t adc_result;
extern encoder_result_t encoder_result;





#define ARR_SIZE_FLT 10


volatile uint8_t fm_cal_enable;
volatile uint8_t fm_cal_done;
volatile uint16_t fm_cal_progress;
volatile uint8_t fm_reset_ctrl;
volatile uint8_t fm_zero_ctrl;
volatile uint8_t fm_direction;



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
    FMSTR_TSA_RW_VAR(fm_reset_ctrl, FMSTR_TSA_UINT8)
    FMSTR_TSA_RW_VAR(fm_zero_ctrl, FMSTR_TSA_UINT8)
    FMSTR_TSA_RW_VAR(fm_direction, FMSTR_TSA_UINT8)

    // ========== ADC Result Structure ==========
    FMSTR_TSA_STRUCT(adc_sample_result_t)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp0_out, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp1_out, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp0_out_voltage, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp1_out_voltage, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(adc_sample_result_t, temperature, FMSTR_TSA_FLOAT)
    
    // ========== Encoder Result Structure ==========
    FMSTR_TSA_STRUCT(encoder_result_t)
        FMSTR_TSA_MEMBER(encoder_result_t, sin_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(encoder_result_t, cos_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(encoder_result_t, sin_norm, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, cos_norm, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, magnitude, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, elec_angle_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, turns, FMSTR_TSA_SINT32)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_counts, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(encoder_result_t, speed_dps, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, speed_rpm, FMSTR_TSA_FLOAT)

    // ========== Global Variables ==========
    FMSTR_TSA_RO_VAR(adc_result, FMSTR_TSA_USERTYPE(adc_sample_result_t))
    FMSTR_TSA_RO_VAR(encoder_result, FMSTR_TSA_USERTYPE(encoder_result_t))

    // ========== Calibration View ==========
    FMSTR_TSA_STRUCT(encoder_calibration_t)
        FMSTR_TSA_MEMBER(encoder_calibration_t, sin_center, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_calibration_t, cos_center, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_calibration_t, transform, FMSTR_TSA_FLOAT)
    FMSTR_TSA_RO_VAR(s_calibration, FMSTR_TSA_USERTYPE(encoder_calibration_t))

FMSTR_TSA_TABLE_END()







FMSTR_TSA_TABLE_LIST_BEGIN()
FMSTR_TSA_TABLE(first_table)
FMSTR_TSA_TABLE_LIST_END()


/****************************************************************************
 * General initialization of FreeMASTER example */

void FMSTR_Example_Init(void)
{
    int i;

    FMSTR_Init();
    fm_cal_enable = 0;
    fm_cal_done = 0;
    fm_cal_progress = 0;
    fm_reset_ctrl = 0;
    fm_zero_ctrl = 0;
    fm_direction = 0;
}


