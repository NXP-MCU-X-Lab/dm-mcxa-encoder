#include "app_encoder_runtime.h"

#include <stdbool.h>
#include <stdint.h>

#include "fsl_common.h"
#include "fsl_device_registers.h"

#include "app_encoder_storage.h"
#include "app_freemaster.h"

#define ZERO_AVERAGE_SAMPLES    64U
#define CAL_SAMPLE_RATE_HZ      1000U
#define CAL_SAMPLE_DECIMATION   (ADC_SAMPLE_RATE_HZ / CAL_SAMPLE_RATE_HZ)

#if ((ADC_SAMPLE_RATE_HZ % CAL_SAMPLE_RATE_HZ) != 0U)
#error "ADC_SAMPLE_RATE_HZ must be an integer multiple of CAL_SAMPLE_RATE_HZ"
#endif

#define CAPTURE_MODE_NONE       0U
#define CAPTURE_MODE_ZERO_USER  1U
#define CAPTURE_MODE_FACTORY    2U

#define FACTORY_CAL_CMD_START   1U

typedef struct {
    uint32_t a1_sin_sum;
    uint32_t a1_cos_sum;
    uint32_t a2_sin_sum;
    uint32_t a2_cos_sum;
} zero_accumulator_t;

adc_sample_result_t adc_result;
encoder_result_t encoder_result;
encoder_diag_t encoder_diag;
float encoder_cal_flat[12];
float encoder_runtime_trim_delta[4];
volatile uint32_t adc_sample_count;
volatile uint32_t adc_overrun_count;
volatile uint32_t encoder_sample_rate_hz = ADC_SAMPLE_RATE_HZ;
volatile uint32_t encoder_calibration_source;
volatile uint32_t encoder_storage_crc_ok;
volatile uint8_t encoder_runtime_trim_enabled;
volatile uint8_t encoder_runtime_trim_active;
volatile uint32_t encoder_runtime_trim_freeze_reason;

static encoder_calibration_t s_factory_calibration;
static encoder_state_t s_encoder_state;
static encoder_runtime_trim_t s_runtime_trim;
static uint32_t s_app_status_flags;

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
static volatile uint32_t s_capture_decimation;
static volatile encoder_raw_sample_t s_capture_sample;
static zero_accumulator_t s_zero_accum;
static encoder_cal_stats_t s_factory_stats;
static encoder_raw_sample_t s_factory_zero_sample;

static void refresh_encoder_cal_flat(void)
{
    encoder_cal_flat[0]  = s_factory_calibration.a1.center_sin;
    encoder_cal_flat[1]  = s_factory_calibration.a1.center_cos;
    encoder_cal_flat[2]  = s_factory_calibration.a1.t00;
    encoder_cal_flat[3]  = s_factory_calibration.a1.t10;
    encoder_cal_flat[4]  = s_factory_calibration.a1.t11;
    encoder_cal_flat[5]  = s_factory_calibration.a2.center_sin;
    encoder_cal_flat[6]  = s_factory_calibration.a2.center_cos;
    encoder_cal_flat[7]  = s_factory_calibration.a2.t00;
    encoder_cal_flat[8]  = s_factory_calibration.a2.t10;
    encoder_cal_flat[9]  = s_factory_calibration.a2.t11;
    encoder_cal_flat[10] = s_factory_calibration.phase_a1_zero_deg;
    encoder_cal_flat[11] = s_factory_calibration.phase_a2_zero_deg;
}

static void refresh_runtime_trim_view(void)
{
    encoder_runtime_trim_enabled = s_runtime_trim.enabled ? 1U : 0U;
    encoder_runtime_trim_active = s_runtime_trim.active ? 1U : 0U;
    encoder_runtime_trim_freeze_reason = s_runtime_trim.freeze_reason;
    encoder_runtime_trim_delta[0] = s_runtime_trim.a1_center_sin_delta;
    encoder_runtime_trim_delta[1] = s_runtime_trim.a1_center_cos_delta;
    encoder_runtime_trim_delta[2] = s_runtime_trim.a2_center_sin_delta;
    encoder_runtime_trim_delta[3] = s_runtime_trim.a2_center_cos_delta;
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

static bool capture_decimation_ready(void)
{
    if (s_capture_mode != CAPTURE_MODE_FACTORY)
    {
        return true;
    }

    s_capture_decimation++;
    if (s_capture_decimation < CAL_SAMPLE_DECIMATION)
    {
        return false;
    }

    s_capture_decimation = 0U;
    return true;
}

static void capture_sample_from_isr(const encoder_raw_sample_t *sample)
{
    if ((s_capture_mode == CAPTURE_MODE_NONE) || (s_capture_done != 0U) ||
        (s_capture_pending != 0U) || !capture_decimation_ready())
    {
        return;
    }

    s_capture_sample = *sample;
    s_capture_pending = 1U;
}

static void encoder_sample_callback(const adc_sample_result_t *sample)
{
    encoder_raw_sample_t encoder_sample = adc_to_encoder_sample(sample);
    encoder_calibration_t effective_calibration;
    encoder_result_t next_encoder_result;
    encoder_diag_t next_encoder_diag;

    encoder_runtime_trim_apply(&s_factory_calibration, &s_runtime_trim, &effective_calibration);
    encoder_process(&s_encoder_state,
                    &effective_calibration,
                    &encoder_sample,
                    &next_encoder_result,
                    &next_encoder_diag);

    if (encoder_calibration_source == ENCODER_CAL_SOURCE_NVM)
    {
        encoder_runtime_trim_update(&s_runtime_trim,
                                    &s_factory_calibration,
                                    &encoder_sample,
                                    &next_encoder_result);
    }
    else
    {
        encoder_runtime_trim_update(&s_runtime_trim, NULL, &encoder_sample, &next_encoder_result);
    }

    next_encoder_result.status |= s_app_status_flags;

    s_realtime_adc_result = *sample;
    s_realtime_encoder_result = next_encoder_result;
    s_realtime_encoder_diag = next_encoder_diag;
    s_realtime_encoder_valid = s_factory_calibration.valid ? 1U : 0U;
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
    refresh_runtime_trim_view();
    EnableGlobalIRQ(irq_mask);
}

static bool capture_is_idle(void)
{
    return s_capture_mode == CAPTURE_MODE_NONE;
}

static void start_zero_capture(void)
{
    uint32_t irq_mask = DisableGlobalIRQ();

    s_zero_accum.a1_sin_sum = 0U;
    s_zero_accum.a1_cos_sum = 0U;
    s_zero_accum.a2_sin_sum = 0U;
    s_zero_accum.a2_cos_sum = 0U;
    s_capture_count = 0U;
    s_capture_decimation = 0U;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    s_capture_mode = CAPTURE_MODE_ZERO_USER;
    EnableGlobalIRQ(irq_mask);
}

static void start_factory_calibration(void)
{
    uint32_t irq_mask = DisableGlobalIRQ();

    encoder_cal_stats_init(&s_factory_stats);
    s_factory_zero_sample = (encoder_raw_sample_t){0};
    s_capture_count = 0U;
    s_capture_decimation = 0U;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    s_capture_mode = CAPTURE_MODE_FACTORY;
    encoder_calibration_source = ENCODER_CAL_SOURCE_FACTORY_PENDING;
    s_runtime_trim.enabled = false;
    encoder_runtime_trim_reset(&s_runtime_trim);
    refresh_runtime_trim_view();
    EnableGlobalIRQ(irq_mask);

    fm_factory_cal_state = ENCODER_FACTORY_CAL_STATE_RUNNING;
    fm_factory_cal_progress = 0U;
    fm_factory_cal_status = ENCODER_STATUS_OK;
}

static void consume_zero_sample(const encoder_raw_sample_t *sample)
{
    s_zero_accum.a1_sin_sum += sample->a1_sin_raw;
    s_zero_accum.a1_cos_sum += sample->a1_cos_raw;
    s_zero_accum.a2_sin_sum += sample->a2_sin_raw;
    s_zero_accum.a2_cos_sum += sample->a2_cos_raw;
    s_capture_count++;
    if (s_capture_count >= ZERO_AVERAGE_SAMPLES)
    {
        s_capture_done = 1U;
    }
}

static void consume_factory_sample(const encoder_raw_sample_t *sample)
{
    encoder_cal_stats_accumulate(&s_factory_stats, sample);
    s_factory_zero_sample = *sample;
    s_capture_count++;
    fm_factory_cal_progress =
        (uint8_t)((s_capture_count * 100U) / ENCODER_CAL_SAMPLE_COUNT);

    if (s_capture_count >= ENCODER_CAL_SAMPLE_COUNT)
    {
        fm_factory_cal_progress = 100U;
        s_capture_done = 1U;
    }
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

    if (mode == CAPTURE_MODE_ZERO_USER)
    {
        consume_zero_sample(&sample);
    }
    else if (mode == CAPTURE_MODE_FACTORY)
    {
        consume_factory_sample(&sample);
    }
    else
    {
        /* No active consumer. */
    }
}

static bool take_zero_sample(encoder_raw_sample_t *sample)
{
    uint32_t irq_mask;
    zero_accumulator_t accum;
    uint32_t count;

    if ((s_capture_mode != CAPTURE_MODE_ZERO_USER) || (s_capture_done == 0U))
    {
        return false;
    }

    irq_mask = DisableGlobalIRQ();
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

static bool take_factory_calibration_done(void)
{
    uint32_t irq_mask;

    if ((s_capture_mode != CAPTURE_MODE_FACTORY) || (s_capture_done == 0U))
    {
        return false;
    }

    irq_mask = DisableGlobalIRQ();
    s_capture_mode = CAPTURE_MODE_NONE;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    EnableGlobalIRQ(irq_mask);

    return true;
}

static void apply_calibration(const encoder_calibration_t *calibration,
                              uint32_t source,
                              uint32_t app_status_flags,
                              bool storage_crc_ok)
{
    uint32_t irq_mask = DisableGlobalIRQ();

    s_factory_calibration = *calibration;
    s_app_status_flags = app_status_flags;
    encoder_calibration_source = source;
    encoder_storage_crc_ok = storage_crc_ok ? 1U : 0U;
    s_runtime_trim.enabled = (source == ENCODER_CAL_SOURCE_NVM);
    encoder_runtime_trim_reset(&s_runtime_trim);
    encoder_state_init(&s_encoder_state);
    refresh_encoder_cal_flat();
    refresh_runtime_trim_view();
    EnableGlobalIRQ(irq_mask);
}

static void load_startup_calibration(void)
{
    encoder_calibration_t calibration;
    encoder_cal_quality_t quality;
    uint32_t sequence;

    if (EncoderStorage_Load(&calibration, &quality, &sequence))
    {
        (void)quality;
        (void)sequence;
        apply_calibration(&calibration, ENCODER_CAL_SOURCE_NVM, ENCODER_STATUS_OK, true);
    }
    else
    {
        encoder_calibration_set_board_defaults(&calibration);
        apply_calibration(&calibration,
                          ENCODER_CAL_SOURCE_DEFAULT,
                          ENCODER_STATUS_CAL_STORAGE_INVALID |
                              ENCODER_STATUS_FACTORY_CAL_REQUIRED,
                          false);
    }
}

static void apply_zero(const encoder_calibration_t *calibration)
{
    apply_calibration(calibration,
                      encoder_calibration_source,
                      s_app_status_flags,
                      encoder_storage_crc_ok != 0U);
}

static void finish_zero_if_ready(void)
{
    encoder_raw_sample_t zero_sample;
    encoder_calibration_t calibration;
    uint32_t status = ENCODER_STATUS_OK;

    if (!take_zero_sample(&zero_sample))
    {
        return;
    }

    calibration = s_factory_calibration;
    if (!calibration.valid)
    {
        return;
    }

    if (encoder_capture_zero(&calibration, &zero_sample, &status))
    {
        apply_zero(&calibration);
        fm_encoder_valid = 1U;
        fm_encoder_status = s_app_status_flags;
    }
    else
    {
        fm_encoder_status = status | s_app_status_flags;
    }
}

static void finish_factory_calibration_if_ready(void)
{
    encoder_calibration_t calibration;
    encoder_state_t temp_state;
    encoder_result_t temp_result;
    encoder_diag_t temp_diag;
    encoder_cal_quality_t quality;
    uint32_t status = ENCODER_STATUS_OK;
    bool saved;

    if (!take_factory_calibration_done())
    {
        return;
    }

    if (!encoder_cal_stats_build(&s_factory_stats, &calibration, &status) ||
        !encoder_capture_zero(&calibration, &s_factory_zero_sample, &status))
    {
        fm_factory_cal_state = ENCODER_FACTORY_CAL_STATE_FAILED;
        fm_factory_cal_status = status | ENCODER_STATUS_CAL_FAILED;
        load_startup_calibration();
        return;
    }

    encoder_state_init(&temp_state);
    encoder_process(&temp_state, &calibration, &s_factory_zero_sample, &temp_result, &temp_diag);
    quality.sample_count = ENCODER_CAL_SAMPLE_COUNT;
    quality.status = temp_result.status;
    quality.mag16 = temp_result.mag16;
    quality.mag15 = temp_result.mag15;

    adc_realtime_stop();
    saved = EncoderStorage_SaveFactoryCalibration(&calibration, &quality);
    adc_realtime_start();

    if (saved)
    {
        apply_calibration(&calibration, ENCODER_CAL_SOURCE_NVM, ENCODER_STATUS_OK, true);
        fm_factory_cal_state = ENCODER_FACTORY_CAL_STATE_DONE;
        fm_factory_cal_progress = 100U;
        fm_factory_cal_status = ENCODER_STATUS_OK;
    }
    else
    {
        fm_factory_cal_state = ENCODER_FACTORY_CAL_STATE_FAILED;
        fm_factory_cal_status = ENCODER_STATUS_CAL_STORAGE_INVALID | ENCODER_STATUS_CAL_FAILED;
        load_startup_calibration();
    }
}

static void service_encoder_commands(void)
{
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
        if ((cmd == 1U) && s_factory_calibration.valid && capture_is_idle())
        {
            start_zero_capture();
        }
    }

    if (fm_factory_cal_ctrl != 0U)
    {
        const uint8_t cmd = fm_factory_cal_ctrl;

        fm_factory_cal_ctrl = 0U;
        if ((cmd == FACTORY_CAL_CMD_START) && capture_is_idle())
        {
            start_factory_calibration();
        }
    }
}

static void service_encoder_capture(void)
{
    consume_capture_sample();
    finish_zero_if_ready();
    finish_factory_calibration_if_ready();
}

void EncoderApp_Init(void)
{
    encoder_runtime_trim_init(&s_runtime_trim);
    load_startup_calibration();

    fm_encoder_valid = s_factory_calibration.valid ? 1U : 0U;
    fm_encoder_status = s_app_status_flags;
    fm_factory_cal_state = ENCODER_FACTORY_CAL_STATE_IDLE;
    fm_factory_cal_progress = 0U;
    fm_factory_cal_status = ENCODER_STATUS_OK;

    adc_set_sample_callback(encoder_sample_callback);
    adc_realtime_start();
}

void EncoderApp_Service(void)
{
    publish_realtime_snapshot();
    service_encoder_capture();
    service_encoder_commands();
}
