#include "app_encoder_runtime.h"

#include <stdbool.h>
#include <stdint.h>

#include "fsl_common.h"
#include "fsl_device_registers.h"

#include "app_freemaster.h"

#define CAL_EFFECTIVE_SAMPLE_RATE_HZ    1000U
#define CAL_SAMPLE_DECIMATION           (ADC_SAMPLE_RATE_HZ / CAL_EFFECTIVE_SAMPLE_RATE_HZ)
#define ZERO_AVERAGE_SAMPLES            64U

#if ((ADC_SAMPLE_RATE_HZ % CAL_EFFECTIVE_SAMPLE_RATE_HZ) != 0U)
#error "Calibration decimation requires an integer realtime-to-calibration sample-rate ratio."
#endif

#define FM_CAL_STATE_IDLE               0U
#define FM_CAL_STATE_RUNNING            1U
#define FM_CAL_STATE_DONE               2U
#define FM_CAL_STATE_FAILED             3U

#define CAPTURE_MODE_NONE               0U
#define CAPTURE_MODE_CALIBRATION        1U
#define CAPTURE_MODE_ZERO_AFTER_CAL     2U
#define CAPTURE_MODE_ZERO_USER          3U

typedef struct {
    uint32_t a1_sin_sum;
    uint32_t a1_cos_sum;
    uint32_t a2_sin_sum;
    uint32_t a2_cos_sum;
} zero_accumulator_t;

adc_sample_result_t adc_result;
encoder_result_t encoder_result;
encoder_diag_t encoder_diag;
float encoder_cal_flat[14];
volatile uint32_t adc_sample_count;
volatile uint32_t adc_overrun_count;
volatile uint32_t encoder_sample_rate_hz = ADC_SAMPLE_RATE_HZ;

static encoder_calibration_t s_encoder_calibration;
static encoder_state_t s_encoder_state;
static volatile adc_sample_result_t s_realtime_adc_result;
static volatile encoder_result_t s_realtime_encoder_result;
static volatile encoder_diag_t s_realtime_encoder_diag;
static volatile uint8_t s_realtime_encoder_valid;
static volatile uint32_t s_realtime_encoder_status;
static volatile uint32_t s_realtime_sequence;
static uint32_t s_published_sequence;

static volatile uint8_t s_capture_mode;
static volatile uint8_t s_capture_done;
static volatile uint8_t s_capture_pending;
static volatile uint32_t s_capture_count;
static uint32_t s_capture_decimator;
static volatile encoder_raw_sample_t s_capture_sample;
static encoder_cal_stats_t s_capture_stats;
static zero_accumulator_t s_zero_accum;
static encoder_calibration_t s_pending_calibration;

static void refresh_encoder_cal_flat(void)
{
    encoder_cal_flat[0] = s_encoder_calibration.a1.center_sin;
    encoder_cal_flat[1] = s_encoder_calibration.a1.center_cos;
    encoder_cal_flat[2] = s_encoder_calibration.a1.transform[0][0];
    encoder_cal_flat[3] = s_encoder_calibration.a1.transform[0][1];
    encoder_cal_flat[4] = s_encoder_calibration.a1.transform[1][0];
    encoder_cal_flat[5] = s_encoder_calibration.a1.transform[1][1];
    encoder_cal_flat[6] = s_encoder_calibration.a2.center_sin;
    encoder_cal_flat[7] = s_encoder_calibration.a2.center_cos;
    encoder_cal_flat[8] = s_encoder_calibration.a2.transform[0][0];
    encoder_cal_flat[9] = s_encoder_calibration.a2.transform[0][1];
    encoder_cal_flat[10] = s_encoder_calibration.a2.transform[1][0];
    encoder_cal_flat[11] = s_encoder_calibration.a2.transform[1][1];
    encoder_cal_flat[12] = s_encoder_calibration.phase_a1_zero_deg;
    encoder_cal_flat[13] = s_encoder_calibration.phase_a2_zero_deg;
}

static encoder_raw_sample_t adc_to_encoder_sample(const adc_sample_result_t *sample)
{
    encoder_raw_sample_t encoder_sample;

    encoder_sample.a1_sin_raw = sample->a1_sin_raw;
    encoder_sample.a1_cos_raw = sample->a1_cos_raw;
    encoder_sample.a2_sin_raw = sample->a2_sin_raw;
    encoder_sample.a2_cos_raw = sample->a2_cos_raw;

    return encoder_sample;
}

static void capture_sample_from_isr(const encoder_raw_sample_t *sample)
{
    if ((s_capture_mode == CAPTURE_MODE_NONE) || (s_capture_done != 0U))
    {
        return;
    }

    s_capture_decimator++;
    if (s_capture_decimator < CAL_SAMPLE_DECIMATION)
    {
        return;
    }
    s_capture_decimator = 0U;

    if (s_capture_pending != 0U)
    {
        return;
    }

    s_capture_sample = *sample;
    s_capture_pending = 1U;
}

static void encoder_sample_callback(const adc_sample_result_t *sample)
{
    encoder_raw_sample_t encoder_sample = adc_to_encoder_sample(sample);
    encoder_result_t next_encoder_result;
    encoder_diag_t next_encoder_diag;

    encoder_process_with_diag(&s_encoder_state,
                              &s_encoder_calibration,
                              &encoder_sample,
                              &next_encoder_result,
                              &next_encoder_diag);

    s_realtime_adc_result = *sample;
    s_realtime_encoder_result = next_encoder_result;
    s_realtime_encoder_diag = next_encoder_diag;
    s_realtime_encoder_valid = s_encoder_calibration.valid ? 1U : 0U;
    s_realtime_encoder_status = next_encoder_result.status;
    s_realtime_sequence++;

    capture_sample_from_isr(&encoder_sample);
}

static void publish_realtime_snapshot(void)
{
    uint32_t irq_mask;

    irq_mask = DisableGlobalIRQ();
    if (s_published_sequence != s_realtime_sequence)
    {
        adc_result = s_realtime_adc_result;
        encoder_result = s_realtime_encoder_result;
        encoder_diag = s_realtime_encoder_diag;
        fm_encoder_valid = s_realtime_encoder_valid;
        fm_encoder_status = s_realtime_encoder_status;
        s_published_sequence = s_realtime_sequence;
    }
    adc_sample_count = adc_get_sample_count();
    adc_overrun_count = adc_get_overrun_count();
    encoder_sample_rate_hz = ADC_SAMPLE_RATE_HZ;
    EnableGlobalIRQ(irq_mask);
}

static void set_calibration_failed(uint32_t status)
{
    fm_cal_done = 0U;
    fm_cal_progress = 100U;
    fm_cal_state = FM_CAL_STATE_FAILED;
    fm_cal_status = status;
    fm_encoder_valid = 0U;
    fm_encoder_status = status;
}

static bool capture_is_idle(void)
{
    return s_capture_mode == CAPTURE_MODE_NONE;
}

static void start_calibration_capture(void)
{
    uint32_t irq_mask;

    if (!capture_is_idle())
    {
        return;
    }

    irq_mask = DisableGlobalIRQ();
    encoder_cal_stats_init(&s_capture_stats);
    s_capture_decimator = 0U;
    s_capture_count = 0U;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    s_capture_mode = CAPTURE_MODE_CALIBRATION;
    EnableGlobalIRQ(irq_mask);

    fm_cal_enable = 0U;
    fm_cal_done = 0U;
    fm_cal_progress = 1U;
    fm_cal_state = FM_CAL_STATE_RUNNING;
    fm_cal_status = ENCODER_STATUS_OK;
    fm_encoder_status = ENCODER_STATUS_OK;
}

static void start_zero_capture(uint8_t mode)
{
    uint32_t irq_mask;

    irq_mask = DisableGlobalIRQ();
    s_zero_accum.a1_sin_sum = 0U;
    s_zero_accum.a1_cos_sum = 0U;
    s_zero_accum.a2_sin_sum = 0U;
    s_zero_accum.a2_cos_sum = 0U;
    s_capture_decimator = 0U;
    s_capture_count = 0U;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    s_capture_mode = mode;
    EnableGlobalIRQ(irq_mask);
}

static void consume_capture_sample(void)
{
    encoder_raw_sample_t sample;
    uint8_t mode;
    uint32_t irq_mask;

    if ((s_capture_pending == 0U) || (s_capture_done != 0U))
    {
        return;
    }

    irq_mask = DisableGlobalIRQ();
    if ((s_capture_pending == 0U) || (s_capture_done != 0U))
    {
        EnableGlobalIRQ(irq_mask);
        return;
    }
    sample = s_capture_sample;
    mode = s_capture_mode;
    s_capture_pending = 0U;
    EnableGlobalIRQ(irq_mask);

    if (mode == CAPTURE_MODE_CALIBRATION)
    {
        encoder_cal_stats_accumulate(&s_capture_stats, &sample);
        s_capture_count++;
        if (s_capture_count >= ENCODER_CAL_SAMPLE_COUNT)
        {
            s_capture_done = 1U;
        }
        return;
    }

    if ((mode == CAPTURE_MODE_ZERO_AFTER_CAL) || (mode == CAPTURE_MODE_ZERO_USER))
    {
        s_zero_accum.a1_sin_sum += sample.a1_sin_raw;
        s_zero_accum.a1_cos_sum += sample.a1_cos_raw;
        s_zero_accum.a2_sin_sum += sample.a2_sin_raw;
        s_zero_accum.a2_cos_sum += sample.a2_cos_raw;
        s_capture_count++;
        if (s_capture_count >= ZERO_AVERAGE_SAMPLES)
        {
            s_capture_done = 1U;
        }
    }
}

static bool take_capture_stats(encoder_cal_stats_t *stats)
{
    uint32_t irq_mask;

    if ((s_capture_mode != CAPTURE_MODE_CALIBRATION) || (s_capture_done == 0U))
    {
        return false;
    }

    irq_mask = DisableGlobalIRQ();
    *stats = s_capture_stats;
    s_capture_mode = CAPTURE_MODE_NONE;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    EnableGlobalIRQ(irq_mask);
    return true;
}

static bool take_zero_sample(uint8_t *mode, encoder_raw_sample_t *sample)
{
    uint32_t irq_mask;
    zero_accumulator_t accum;
    uint32_t count;

    if (((s_capture_mode != CAPTURE_MODE_ZERO_AFTER_CAL) && (s_capture_mode != CAPTURE_MODE_ZERO_USER)) ||
        (s_capture_done == 0U))
    {
        return false;
    }

    irq_mask = DisableGlobalIRQ();
    *mode = s_capture_mode;
    accum = s_zero_accum;
    count = s_capture_count;
    s_capture_mode = CAPTURE_MODE_NONE;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    EnableGlobalIRQ(irq_mask);

    if (count == 0U)
    {
        return false;
    }

    sample->a1_sin_raw = (uint16_t)((accum.a1_sin_sum + (count / 2U)) / count);
    sample->a1_cos_raw = (uint16_t)((accum.a1_cos_sum + (count / 2U)) / count);
    sample->a2_sin_raw = (uint16_t)((accum.a2_sin_sum + (count / 2U)) / count);
    sample->a2_cos_raw = (uint16_t)((accum.a2_cos_sum + (count / 2U)) / count);
    return true;
}

static void apply_calibration(encoder_calibration_t *calibration)
{
    uint32_t irq_mask = DisableGlobalIRQ();

    s_encoder_calibration = *calibration;
    encoder_state_init(&s_encoder_state);
    refresh_encoder_cal_flat();
    EnableGlobalIRQ(irq_mask);
}

static void finish_calibration_if_ready(void)
{
    encoder_cal_stats_t stats;
    encoder_calibration_t calibration;
    uint32_t status = ENCODER_STATUS_OK;

    if (!take_capture_stats(&stats))
    {
        return;
    }

    if (!encoder_cal_stats_build(&stats, &calibration, &status))
    {
        set_calibration_failed(status);
        return;
    }

    s_pending_calibration = calibration;
    start_zero_capture(CAPTURE_MODE_ZERO_AFTER_CAL);
    fm_cal_progress = 100U;
}

static void finish_zero_if_ready(void)
{
    encoder_raw_sample_t zero_sample;
    encoder_calibration_t calibration;
    uint32_t status = ENCODER_STATUS_OK;
    uint8_t mode;

    if (!take_zero_sample(&mode, &zero_sample))
    {
        return;
    }

    if (mode == CAPTURE_MODE_ZERO_AFTER_CAL)
    {
        calibration = s_pending_calibration;
    }
    else
    {
        calibration = s_encoder_calibration;
    }

    if (!calibration.valid)
    {
        if (mode == CAPTURE_MODE_ZERO_AFTER_CAL)
        {
            set_calibration_failed(ENCODER_STATUS_NOT_CALIBRATED);
        }
        return;
    }

    if (encoder_capture_zero(&calibration, &zero_sample, &status))
    {
        apply_calibration(&calibration);
        fm_encoder_valid = 1U;
        fm_encoder_status = ENCODER_STATUS_OK;

        if (mode == CAPTURE_MODE_ZERO_AFTER_CAL)
        {
            fm_cal_done = 1U;
            fm_cal_progress = 100U;
            fm_cal_state = FM_CAL_STATE_DONE;
            fm_cal_status = ENCODER_STATUS_OK;
        }
    }
    else if (mode == CAPTURE_MODE_ZERO_AFTER_CAL)
    {
        set_calibration_failed(status);
    }
    else
    {
        fm_encoder_status = status;
    }
}

static void update_calibration_progress(void)
{
    uint32_t count;

    if (s_capture_mode != CAPTURE_MODE_CALIBRATION)
    {
        return;
    }

    count = s_capture_count;
    fm_cal_progress = (uint16_t)((count * 100U) / ENCODER_CAL_SAMPLE_COUNT);
    if (fm_cal_progress == 0U)
    {
        fm_cal_progress = 1U;
    }
}

static void service_encoder_commands(void)
{
    if (fm_cal_enable != 0U)
    {
        fm_cal_enable = 0U;
        start_calibration_capture();
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
        if ((cmd == 1U) && s_encoder_calibration.valid && capture_is_idle())
        {
            start_zero_capture(CAPTURE_MODE_ZERO_USER);
        }
    }
}

static void service_encoder_capture(void)
{
    consume_capture_sample();
    update_calibration_progress();
    finish_calibration_if_ready();
    finish_zero_if_ready();
}

void EncoderApp_Init(void)
{
    encoder_calibration_set_board_defaults(&s_encoder_calibration);
    encoder_state_init(&s_encoder_state);
    refresh_encoder_cal_flat();

    fm_encoder_valid = s_encoder_calibration.valid ? 1U : 0U;
    fm_encoder_status = ENCODER_STATUS_OK;

    adc_set_sample_callback(encoder_sample_callback);
    adc_realtime_start();
}

void EncoderApp_Service(void)
{
    publish_realtime_snapshot();
    service_encoder_capture();
    service_encoder_commands();
}
