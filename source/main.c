/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#include <stdbool.h>
#include <stdint.h>

#include "fsl_common.h"
#include "fsl_device_registers.h"
#include "fsl_lpuart.h"

#include "app_adc.h"
#include "app_encoder_v2.h"
#include "freemaster.h"
#include "freemaster_example.h"
#include "freemaster_serial_lpuart.h"
#include "hardware_init.h"

#define FMSTR_LOOP_DELAY_MS     1U
#define CAL_SAMPLE_DELAY_MS     1U
#define ZERO_AVERAGE_SAMPLES    64U

#define FM_CAL_STATE_IDLE       0U
#define FM_CAL_STATE_RUNNING    1U
#define FM_CAL_STATE_DONE       2U
#define FM_CAL_STATE_FAILED     3U

adc_sample_result_t adc_result;
v2_encoder_result_t v2_result;
v2_encoder_diag_t v2_diag;
v2_encoder_calibration_t s_v2_calibration;
/* FreeMASTER JSON-RPC reads this flat view instead of nested struct members. */
float s_v2_cal_flat[14];

static v2_encoder_state_t s_v2_state;

static void refresh_v2_cal_flat(void)
{
    s_v2_cal_flat[0] = s_v2_calibration.a1.center_sin;
    s_v2_cal_flat[1] = s_v2_calibration.a1.center_cos;
    s_v2_cal_flat[2] = s_v2_calibration.a1.transform[0][0];
    s_v2_cal_flat[3] = s_v2_calibration.a1.transform[0][1];
    s_v2_cal_flat[4] = s_v2_calibration.a1.transform[1][0];
    s_v2_cal_flat[5] = s_v2_calibration.a1.transform[1][1];
    s_v2_cal_flat[6] = s_v2_calibration.a2.center_sin;
    s_v2_cal_flat[7] = s_v2_calibration.a2.center_cos;
    s_v2_cal_flat[8] = s_v2_calibration.a2.transform[0][0];
    s_v2_cal_flat[9] = s_v2_calibration.a2.transform[0][1];
    s_v2_cal_flat[10] = s_v2_calibration.a2.transform[1][0];
    s_v2_cal_flat[11] = s_v2_calibration.a2.transform[1][1];
    s_v2_cal_flat[12] = s_v2_calibration.phase_a1_zero_deg;
    s_v2_cal_flat[13] = s_v2_calibration.phase_a2_zero_deg;
}

static v2_encoder_raw_sample_t adc_to_encoder_sample(const adc_sample_result_t *sample)
{
    v2_encoder_raw_sample_t encoder_sample;

    encoder_sample.a1_sin_raw = sample->a1_sin_raw;
    encoder_sample.a1_cos_raw = sample->a1_cos_raw;
    encoder_sample.a2_sin_raw = sample->a2_sin_raw;
    encoder_sample.a2_cos_raw = sample->a2_cos_raw;

    return encoder_sample;
}

static v2_encoder_raw_sample_t average_encoder_sample(uint32_t sample_count, uint32_t per_sample_ms)
{
    uint32_t a1_sin_sum = 0U;
    uint32_t a1_cos_sum = 0U;
    uint32_t a2_sin_sum = 0U;
    uint32_t a2_cos_sum = 0U;
    uint32_t i;
    v2_encoder_raw_sample_t sample;

    for (i = 0U; i < sample_count; i++)
    {
        FMSTR_Poll();
        adc_result = adc_read();
        a1_sin_sum += adc_result.a1_sin_raw;
        a1_cos_sum += adc_result.a1_cos_raw;
        a2_sin_sum += adc_result.a2_sin_raw;
        a2_cos_sum += adc_result.a2_cos_raw;
        delay_ms(per_sample_ms);
    }

    sample.a1_sin_raw = (uint16_t)((a1_sin_sum + (sample_count / 2U)) / sample_count);
    sample.a1_cos_raw = (uint16_t)((a1_cos_sum + (sample_count / 2U)) / sample_count);
    sample.a2_sin_raw = (uint16_t)((a2_sin_sum + (sample_count / 2U)) / sample_count);
    sample.a2_cos_raw = (uint16_t)((a2_cos_sum + (sample_count / 2U)) / sample_count);

    return sample;
}

static void perform_v2_calibration_fm(void)
{
    v2_encoder_cal_stats_t stats;
    v2_encoder_raw_sample_t encoder_sample;
    uint32_t status = V2_ENCODER_STATUS_OK;
    uint32_t i;

    fm_cal_enable = 0U;
    fm_cal_done = 0U;
    fm_cal_progress = 1U;
    fm_cal_state = FM_CAL_STATE_RUNNING;
    fm_cal_status = V2_ENCODER_STATUS_OK;
    fm_encoder_status = V2_ENCODER_STATUS_OK;

    v2_encoder_cal_stats_init(&stats);

    for (i = 0U; i < V2_ENCODER_CAL_SAMPLE_COUNT; i++)
    {
        FMSTR_Poll();
        adc_result = adc_read();
        encoder_sample = adc_to_encoder_sample(&adc_result);
        v2_encoder_cal_stats_accumulate(&stats, &encoder_sample);

        fm_cal_progress = (uint16_t)(((i + 1U) * 100U) / V2_ENCODER_CAL_SAMPLE_COUNT);
        if (fm_cal_progress == 0U)
        {
            fm_cal_progress = 1U;
        }
        delay_ms(CAL_SAMPLE_DELAY_MS);
    }

    if (v2_encoder_cal_stats_build(&stats, &s_v2_calibration, &status))
    {
        encoder_sample = average_encoder_sample(ZERO_AVERAGE_SAMPLES, CAL_SAMPLE_DELAY_MS);
        if (v2_encoder_capture_zero(&s_v2_calibration, &encoder_sample, &status))
        {
            v2_encoder_state_init(&s_v2_state);
            fm_cal_done = 1U;
            fm_cal_progress = 100U;
            fm_cal_state = FM_CAL_STATE_DONE;
            fm_cal_status = V2_ENCODER_STATUS_OK;
            fm_encoder_valid = 1U;
            fm_encoder_status = V2_ENCODER_STATUS_OK;
        }
        else
        {
            fm_cal_done = 0U;
            fm_cal_progress = 100U;
            fm_cal_state = FM_CAL_STATE_FAILED;
            fm_cal_status = status;
            fm_encoder_valid = 0U;
            fm_encoder_status = status;
        }
    }
    else
    {
        fm_cal_done = 0U;
        fm_cal_progress = 100U;
        fm_cal_state = FM_CAL_STATE_FAILED;
        fm_cal_status = status;
        fm_encoder_valid = 0U;
        fm_encoder_status = status;
    }

    refresh_v2_cal_flat();
}

static void capture_current_zero_fm(void)
{
    v2_encoder_raw_sample_t zero_sample;
    uint32_t status = V2_ENCODER_STATUS_OK;

    if (!s_v2_calibration.valid)
    {
        return;
    }

    zero_sample = average_encoder_sample(ZERO_AVERAGE_SAMPLES, CAL_SAMPLE_DELAY_MS);
    if (v2_encoder_capture_zero(&s_v2_calibration, &zero_sample, &status))
    {
        v2_encoder_state_init(&s_v2_state);
        refresh_v2_cal_flat();
        fm_encoder_valid = 1U;
        fm_encoder_status = V2_ENCODER_STATUS_OK;
    }
    else
    {
        fm_encoder_status = status;
    }
}

static void update_encoder_result(void)
{
    adc_sample_result_t next_adc_result;
    v2_encoder_raw_sample_t encoder_sample;
    v2_encoder_result_t next_v2_result;
    v2_encoder_diag_t next_v2_diag;
    uint32_t irq_mask;

    next_adc_result = adc_read();
    encoder_sample = adc_to_encoder_sample(&next_adc_result);
    v2_encoder_process_with_diag(&s_v2_state,
                                 &s_v2_calibration,
                                 &encoder_sample,
                                 &next_v2_result,
                                 &next_v2_diag);

    irq_mask = DisableGlobalIRQ();
    adc_result = next_adc_result;
    v2_result = next_v2_result;
    v2_diag = next_v2_diag;
    fm_encoder_valid = s_v2_calibration.valid ? 1U : 0U;
    fm_encoder_status = next_v2_result.status;
    EnableGlobalIRQ(irq_mask);

    refresh_v2_cal_flat();
}

static void init_freemaster(void)
{
    FMSTR_SerialSetBaseAddress((LPUART_Type *)LPUART0);
    FMSTR_Example_Init();

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    NVIC_SetPriority(LPUART0_IRQn, 0);
    EnableIRQ(LPUART0_IRQn);
#endif
}

static void run_freemaster_loop(void)
{
    while (1)
    {
        FMSTR_Poll();

        if (fm_cal_enable != 0U)
        {
            perform_v2_calibration_fm();
        }

        if (fm_reset_ctrl != 0U)
        {
            const uint8_t cmd = fm_reset_ctrl;

            fm_reset_ctrl = 0U;
            if (cmd == 1U)
            {
                NVIC_SystemReset();
            }
        }

        if (fm_zero_ctrl != 0U)
        {
            const uint8_t cmd = fm_zero_ctrl;

            fm_zero_ctrl = 0U;
            if (cmd == 1U)
            {
                capture_current_zero_fm();
            }
        }

        update_encoder_result();
        delay_ms(FMSTR_LOOP_DELAY_MS);
    }
}

int main(void)
{
    Hardware_Init();
    adc_init();

    v2_encoder_calibration_set_board_defaults(&s_v2_calibration);
    v2_encoder_state_init(&s_v2_state);
    refresh_v2_cal_flat();

    init_freemaster();
    fm_encoder_valid = s_v2_calibration.valid ? 1U : 0U;
    fm_encoder_status = V2_ENCODER_STATUS_OK;
    run_freemaster_loop();
}

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR

void LPUART0_IRQHandler(void)
{
    FMSTR_SerialIsr();
}

#endif

void HardFault_Handler(void)
{
    while (1)
    {
    }
}
