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

volatile uint8_t fm_reset_ctrl;
volatile uint8_t fm_zero_ctrl;
volatile uint8_t fm_turn_reset_ctrl;
volatile uint8_t fm_factory_cal_ctrl;
volatile uint8_t fm_factory_cal_state;
volatile uint8_t fm_factory_cal_progress;
volatile uint32_t fm_factory_cal_status;
volatile uint8_t fm_encoder_valid;

// clang-format off

/* TSA tables: only the variables the host UI actually reads.
 * Earlier diagnostic / debug fields (raw ADC dump, ellipse coefficients,
 * runtime trim view, separate diag struct) have been dropped to keep the
 * exposed surface small and the demo focused. */

FMSTR_TSA_TABLE_BEGIN(first_table)
    FMSTR_TSA_RW_VAR(fm_reset_ctrl, FMSTR_TSA_UINT8)
    FMSTR_TSA_RW_VAR(fm_zero_ctrl, FMSTR_TSA_UINT8)
    FMSTR_TSA_RW_VAR(fm_turn_reset_ctrl, FMSTR_TSA_UINT8)
    FMSTR_TSA_RW_VAR(fm_factory_cal_ctrl, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_factory_cal_state, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_factory_cal_progress, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_factory_cal_status, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(fm_encoder_valid, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(adc_sample_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(adc_overrun_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_calibration_source, FMSTR_TSA_UINT32)

    /* Compute-time profiler — DWT cycle counts, sampled inside ADC ISR. */
    FMSTR_TSA_RO_VAR(encoder_perf_process_cycles, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_perf_process_max, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_perf_isr_cycles, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_perf_isr_max, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_perf_core_clock_hz, FMSTR_TSA_UINT32)

    /* Raw 4-channel ADC sample — exposed for the FreeMASTER oscilloscope. */
    FMSTR_TSA_STRUCT(adc_sample_result_t)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a1_sin_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a1_cos_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a2_sin_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a2_cos_raw, FMSTR_TSA_UINT16)
    FMSTR_TSA_RO_VAR(adc_result, FMSTR_TSA_USERTYPE(adc_sample_result_t))

    FMSTR_TSA_STRUCT(encoder_result_t)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg_raw, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg_filtered, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angular_velocity_dps, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_counts, FMSTR_TSA_UINT32)
        FMSTR_TSA_MEMBER(encoder_result_t, phase16_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, phase15_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, coarse_deg, FMSTR_TSA_FLOAT)
        /* Branch margin gauge: approaches +/-180 as the coarse estimate nears a
         * Vernier branch boundary, where the published angle is one bad sample
         * away from jumping 22.5 deg. The most useful single number on the board. */
        FMSTR_TSA_MEMBER(encoder_result_t, fine_delta_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, mag16, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, mag15, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, mag16_raw, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, mag15_raw, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, turn_count, FMSTR_TSA_SINT32)
        FMSTR_TSA_MEMBER(encoder_result_t, multi_turn_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, status, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_result, FMSTR_TSA_USERTYPE(encoder_result_t))

FMSTR_TSA_TABLE_END()



FMSTR_TSA_TABLE_LIST_BEGIN()
FMSTR_TSA_TABLE(first_table)
FMSTR_TSA_TABLE_LIST_END()


void AppFreemaster_Init(void)
{
    FMSTR_SerialSetBaseAddress((LPUART_Type *)LPUART0);
    FMSTR_Init();
    fm_reset_ctrl = 0;
    fm_zero_ctrl = 0;
    fm_turn_reset_ctrl = 0;
    fm_factory_cal_ctrl = 0;
    fm_factory_cal_state = 0;
    fm_factory_cal_progress = 0;
    fm_factory_cal_status = 0;
    fm_encoder_valid = 0;
    adc_sample_count = 0;
    adc_overrun_count = 0;
    encoder_calibration_source = ENCODER_CAL_SOURCE_INVALID;

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    NVIC_SetPriority(LPUART0_IRQn, 3U);
    EnableIRQ(LPUART0_IRQn);
#endif
}
