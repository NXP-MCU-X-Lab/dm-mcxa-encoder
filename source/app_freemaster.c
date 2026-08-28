/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_freemaster.h"

#include "fsl_clock.h"
#include "fsl_lpuart.h"
#include "fsl_reset.h"
#include "freemaster.h"
#include "freemaster_serial_lpuart.h"

#include "app_encoder_runtime.h"
#include "app_tformat.h"

#define FREEMASTER_BAUD_RATE (115200U)

volatile uint8_t fm_command;
volatile uint8_t fm_command_state;
volatile uint32_t fm_command_status;
volatile uint8_t fm_factory_cal_progress;
volatile uint8_t fm_encoder_ready;
volatile uint8_t fm_encoder_stationary;

// clang-format off

FMSTR_TSA_TABLE_BEGIN(first_table)
    FMSTR_TSA_RW_VAR(fm_command, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_command_state, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_command_status, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(fm_factory_cal_progress, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_encoder_ready, FMSTR_TSA_UINT8)
    FMSTR_TSA_RO_VAR(fm_encoder_stationary, FMSTR_TSA_UINT8)

    FMSTR_TSA_RO_VAR(encoder_calibration_source, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(adc_sample_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(adc_overrun_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_perf_process_max, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_perf_isr_max, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_perf_core_clock_hz, FMSTR_TSA_UINT32)

    FMSTR_TSA_RO_VAR(tformat_id0_request_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_id3_request_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_response_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_busy_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_unsupported_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_uart_error_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_stale_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_crc_error_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_desync_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_eeprom_write_count, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(tformat_eeprom_error_count, FMSTR_TSA_UINT32)

    FMSTR_TSA_STRUCT(adc_sample_result_t)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a1_sin_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a1_cos_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a2_sin_raw, FMSTR_TSA_UINT16)
        FMSTR_TSA_MEMBER(adc_sample_result_t, a2_cos_raw, FMSTR_TSA_UINT16)
    FMSTR_TSA_RO_VAR(adc_result, FMSTR_TSA_USERTYPE(adc_sample_result_t))

    FMSTR_TSA_STRUCT(encoder_result_t)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_deg_raw, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angular_velocity_dps, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, angle_counts, FMSTR_TSA_UINT32)
        FMSTR_TSA_MEMBER(encoder_result_t, fine_delta_deg, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, mag16_raw, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, mag15_raw, FMSTR_TSA_FLOAT)
        FMSTR_TSA_MEMBER(encoder_result_t, status, FMSTR_TSA_UINT32)
    FMSTR_TSA_RO_VAR(encoder_result, FMSTR_TSA_USERTYPE(encoder_result_t))
FMSTR_TSA_TABLE_END()

FMSTR_TSA_TABLE_LIST_BEGIN()
    FMSTR_TSA_TABLE(first_table)
FMSTR_TSA_TABLE_LIST_END()

// clang-format on

void AppFreemaster_Init(void)
{
    lpuart_config_t config;

    CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1U);
    CLOCK_AttachClk(kFRO_HF_DIV_to_LPUART0);
    RESET_PeripheralReset(kLPUART0_RST_SHIFT_RSTn);

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = FREEMASTER_BAUD_RATE;
    config.enableTx = true;
    config.enableRx = true;
    (void)LPUART_Init(LPUART0, &config, CLOCK_GetFreq(kCLOCK_FroHfDiv));

    FMSTR_SerialSetBaseAddress((LPUART_Type *)LPUART0);
    FMSTR_Init();

    fm_command = FM_COMMAND_NONE;
    fm_command_state = FM_COMMAND_STATE_IDLE;
    fm_command_status = FM_COMMAND_STATUS_OK;
    fm_factory_cal_progress = 0U;
    fm_encoder_ready = 0U;
    fm_encoder_stationary = 0U;
    encoder_calibration_source = ENCODER_CAL_SOURCE_INVALID;

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    NVIC_SetPriority(LPUART0_IRQn, 3U);
    EnableIRQ(LPUART0_IRQn);
#endif
}
