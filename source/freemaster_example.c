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

volatile unsigned long var32;



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
    FMSTR_TSA_RO_VAR(var32, FMSTR_TSA_UINT32)

    // ========== ADC Result Structure ==========
    FMSTR_TSA_STRUCT(adc_sample_result_t)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp0_inp, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp0_inn, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp0_out, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp1_inp, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp1_inn, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp1_out, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp0_inp_voltage, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp0_inn_voltage, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp0_out_voltage, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp1_inp_voltage, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(adc_sample_result_t, opamp1_inn_voltage, FMSTR_TSA_FLOAT)
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

    // ========== Global Variables ==========
    FMSTR_TSA_RO_VAR(adc_result, FMSTR_TSA_USERTYPE(adc_sample_result_t))
    FMSTR_TSA_RO_VAR(encoder_result, FMSTR_TSA_USERTYPE(encoder_result_t))

FMSTR_TSA_TABLE_END()







FMSTR_TSA_TABLE_LIST_BEGIN()
FMSTR_TSA_TABLE(first_table)
FMSTR_TSA_TABLE_LIST_END()


/****************************************************************************
 * General initialization of FreeMASTER example */

void FMSTR_Example_Init(void)
{
    int i;

    var32    = 55;
    FMSTR_Init();
}


