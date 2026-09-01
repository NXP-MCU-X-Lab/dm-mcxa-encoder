#include "app_encoder_runtime.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "fsl_common.h"
#include "fsl_device_registers.h"

#include "app_encoder_storage.h"
#include "app_freemaster.h"
#include "app_tformat.h"

#define ZERO_AVERAGE_SAMPLES      (64U)
#define CAL_SAMPLE_RATE_HZ        (1000U)
#define CAL_SAMPLE_DECIMATION     (ADC_SAMPLE_RATE_HZ / CAL_SAMPLE_RATE_HZ)
#define STATIONARY_WINDOW_COUNTS  (16U)
#define STATIONARY_SAMPLES        (ADC_SAMPLE_RATE_HZ / 2U)
#define ENCODER_AUTOSAVE_MIN_SOLVES       (8U)
#define ENCODER_AUTOSAVE_CENTER_COUNTS    (16.0f)
#define ENCODER_AUTOSAVE_GAIN_DELTA       (0.01f)

#if ((ADC_SAMPLE_RATE_HZ % CAL_SAMPLE_RATE_HZ) != 0U)
#error "ADC_SAMPLE_RATE_HZ must be an integer multiple of CAL_SAMPLE_RATE_HZ"
#endif

#define CAPTURE_MODE_NONE    (0U)
#define CAPTURE_MODE_ZERO    (1U)
#define CAPTURE_MODE_FACTORY (2U)

#define ZERO_ORIGIN_NONE    (0U)
#define ZERO_ORIGIN_FM      (1U)
#define ZERO_ORIGIN_TFORMAT (2U)

#define SAVE_PENDING_NONE    (0U)
#define SAVE_PENDING_ZERO    (1U)
#define SAVE_PENDING_FACTORY (2U)

typedef enum _storage_result
{
    STORAGE_RESULT_DEFERRED,
    STORAGE_RESULT_SAVED,
    STORAGE_RESULT_FAILED
} storage_result_t;

typedef struct _zero_accumulator
{
    uint32_t a1_sin_sum;
    uint32_t a1_cos_sum;
    uint32_t a2_sin_sum;
    uint32_t a2_cos_sum;
} zero_accumulator_t;

extern uint32_t SystemCoreClock;

encoder_result_t encoder_result;
adc_sample_result_t adc_result;
volatile uint32_t adc_sample_count;
volatile uint32_t adc_overrun_count;
volatile uint32_t encoder_calibration_source;
volatile uint32_t encoder_perf_process_max;
volatile uint32_t encoder_perf_isr_max;
volatile uint32_t encoder_perf_core_clock_hz;

static encoder_calibration_t s_factory_calibration;
static encoder_cal_quality_t s_saved_quality;
static encoder_state_t s_encoder_state;
static encoder_runtime_trim_t s_runtime_trim;
static uint32_t s_app_status_flags;
static bool s_has_persistent_config;
static bool s_autosave_done;

static volatile encoder_result_t s_realtime_encoder_result;
static volatile uint8_t s_realtime_encoder_ready;
static volatile uint32_t s_realtime_sequence;
static uint32_t s_published_sequence;

static volatile uint8_t s_capture_mode;
static volatile uint8_t s_capture_done;
static volatile uint8_t s_capture_pending;
static volatile uint32_t s_capture_count;
static volatile uint32_t s_capture_decimation;
static volatile encoder_raw_sample_t s_capture_sample;
static zero_accumulator_t s_zero_accumulator;
static encoder_cal_stats_t s_factory_stats;
static encoder_raw_sample_t s_factory_zero_sample;
static uint8_t s_zero_origin;

static uint8_t s_save_pending;
static bool s_erase_pending;
static encoder_calibration_t s_pending_calibration;
static encoder_cal_quality_t s_pending_quality;
static encoder_persistent_config_t s_storage_config;

static uint32_t s_stationary_anchor;
static uint32_t s_stationary_count;
static bool s_stationary_tracking;

static void perf_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    encoder_perf_core_clock_hz = SystemCoreClock;
}

static encoder_raw_sample_t adc_to_encoder_sample(const adc_sample_result_t *sample)
{
    encoder_raw_sample_t result;

    result.a1_sin_raw = sample->a1_sin_raw;
    result.a1_cos_raw = sample->a1_cos_raw;
    result.a2_sin_raw = sample->a2_sin_raw;
    result.a2_cos_raw = sample->a2_cos_raw;
    return result;
}

static int32_t wrapped_count_delta(uint32_t value, uint32_t reference)
{
    int32_t delta = (int32_t)value - (int32_t)reference;

    if (delta > ((int32_t)ENCODER_COUNTS_PER_REV / 2))
    {
        delta -= (int32_t)ENCODER_COUNTS_PER_REV;
    }
    else if (delta < -((int32_t)ENCODER_COUNTS_PER_REV / 2))
    {
        delta += (int32_t)ENCODER_COUNTS_PER_REV;
    }
    return delta;
}

static void update_stationary_state(const encoder_result_t *result)
{
    int32_t delta;

    if ((result->status & ENCODER_STATUS_POSITION_INVALID_MASK) != 0U)
    {
        s_stationary_tracking = false;
        s_stationary_count = 0U;
        fm_encoder_stationary = 0U;
        return;
    }

    if (!s_stationary_tracking)
    {
        s_stationary_anchor = result->angle_counts;
        s_stationary_count = 1U;
        s_stationary_tracking = true;
        fm_encoder_stationary = 0U;
        return;
    }

    delta = wrapped_count_delta(result->angle_counts, s_stationary_anchor);
    if ((delta > (int32_t)STATIONARY_WINDOW_COUNTS) ||
        (delta < -(int32_t)STATIONARY_WINDOW_COUNTS))
    {
        s_stationary_anchor = result->angle_counts;
        s_stationary_count = 1U;
        fm_encoder_stationary = 0U;
        return;
    }

    if (s_stationary_count < STATIONARY_SAMPLES)
    {
        s_stationary_count++;
    }
    fm_encoder_stationary = (s_stationary_count >= STATIONARY_SAMPLES) ? 1U : 0U;
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
    const encoder_raw_sample_t encoder_sample = adc_to_encoder_sample(sample);
    encoder_calibration_t effective_calibration;
    encoder_result_t next_result;
    uint32_t isr_start;
    uint32_t process_start;
    uint32_t process_cycles;
    uint32_t isr_cycles;
    bool ready;

    adc_result = *sample;
    isr_start = DWT->CYCCNT;
    encoder_runtime_trim_apply(&s_factory_calibration, &s_runtime_trim, &effective_calibration);

    process_start = DWT->CYCCNT;
    encoder_process(&s_encoder_state, &effective_calibration, &encoder_sample, &next_result, NULL);
    process_cycles = DWT->CYCCNT - process_start;

    encoder_runtime_trim_update(&s_runtime_trim,
                                &s_factory_calibration,
                                &encoder_sample,
                                &next_result);
    next_result.status |= s_app_status_flags;
    ready = s_has_persistent_config || s_runtime_trim.has_locked;
    update_stationary_state(&next_result);

    s_realtime_encoder_result = next_result;
    s_realtime_encoder_ready = ready ? 1U : 0U;
    s_realtime_sequence++;
    TFormat_Publish(&next_result, ready, fm_encoder_stationary != 0U);
    capture_sample_from_isr(&encoder_sample);

    isr_cycles = DWT->CYCCNT - isr_start;
    if (process_cycles > encoder_perf_process_max)
    {
        encoder_perf_process_max = process_cycles;
    }
    if (isr_cycles > encoder_perf_isr_max)
    {
        encoder_perf_isr_max = isr_cycles;
    }
}

static void publish_realtime_snapshot(void)
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    if (s_published_sequence != s_realtime_sequence)
    {
        encoder_result = s_realtime_encoder_result;
        fm_encoder_ready = s_realtime_encoder_ready;
        s_published_sequence = s_realtime_sequence;
    }
    adc_sample_count = adc_get_sample_count();
    adc_overrun_count = adc_get_overrun_count();
    EnableGlobalIRQ(irq_mask);
}

static bool capture_is_idle(void)
{
    return s_capture_mode == CAPTURE_MODE_NONE;
}

static void capture_reset(uint8_t mode)
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    s_capture_count = 0U;
    s_capture_decimation = 0U;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    s_capture_mode = mode;
    EnableGlobalIRQ(irq_mask);
}

static void start_zero_capture(uint8_t origin)
{
    memset(&s_zero_accumulator, 0, sizeof(s_zero_accumulator));
    s_zero_origin = origin;
    capture_reset(CAPTURE_MODE_ZERO);
}

static void start_factory_calibration(void)
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    encoder_cal_stats_init(&s_factory_stats);
    memset(&s_factory_zero_sample, 0, sizeof(s_factory_zero_sample));
    s_runtime_trim.enabled = false;
    encoder_runtime_trim_reset(&s_runtime_trim);
    EnableGlobalIRQ(irq_mask);

    encoder_calibration_source = ENCODER_CAL_SOURCE_FACTORY_PENDING;
    fm_factory_cal_progress = 0U;
    capture_reset(CAPTURE_MODE_FACTORY);
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

    if (mode == CAPTURE_MODE_ZERO)
    {
        if (fm_encoder_stationary == 0U)
        {
            irq_mask = DisableGlobalIRQ();
            s_capture_mode = CAPTURE_MODE_NONE;
            s_capture_pending = 0U;
            s_capture_done = 0U;
            EnableGlobalIRQ(irq_mask);
            if (s_zero_origin == ZERO_ORIGIN_FM)
            {
                fm_command_state = FM_COMMAND_STATE_ERROR;
                fm_command_status = FM_COMMAND_STATUS_MOVING;
            }
            else
            {
                TFormat_ReportCountingError();
            }
            s_zero_origin = ZERO_ORIGIN_NONE;
            return;
        }
        s_zero_accumulator.a1_sin_sum += sample.a1_sin_raw;
        s_zero_accumulator.a1_cos_sum += sample.a1_cos_raw;
        s_zero_accumulator.a2_sin_sum += sample.a2_sin_raw;
        s_zero_accumulator.a2_cos_sum += sample.a2_cos_raw;
        s_capture_count++;
        if (s_capture_count >= ZERO_AVERAGE_SAMPLES)
        {
            s_capture_done = 1U;
        }
    }
    else if (mode == CAPTURE_MODE_FACTORY)
    {
        encoder_cal_stats_accumulate(&s_factory_stats, &sample);
        s_factory_zero_sample = sample;
        s_capture_count++;
        fm_factory_cal_progress = (uint8_t)((s_capture_count * 100U) /
                                            ENCODER_CAL_SAMPLE_COUNT);
        if (s_capture_count >= ENCODER_CAL_SAMPLE_COUNT)
        {
            s_capture_done = 1U;
            fm_factory_cal_progress = 100U;
        }
    }
}

static bool take_zero_sample(encoder_raw_sample_t *sample)
{
    zero_accumulator_t accumulator;
    uint32_t count;
    uint32_t irq_mask;

    if ((s_capture_mode != CAPTURE_MODE_ZERO) || (s_capture_done == 0U))
    {
        return false;
    }

    irq_mask = DisableGlobalIRQ();
    accumulator = s_zero_accumulator;
    count = s_capture_count;
    s_capture_mode = CAPTURE_MODE_NONE;
    s_capture_done = 0U;
    s_capture_pending = 0U;
    EnableGlobalIRQ(irq_mask);

    sample->a1_sin_raw = (uint16_t)((accumulator.a1_sin_sum + count / 2U) / count);
    sample->a1_cos_raw = (uint16_t)((accumulator.a1_cos_sum + count / 2U) / count);
    sample->a2_sin_raw = (uint16_t)((accumulator.a2_sin_sum + count / 2U) / count);
    sample->a2_cos_raw = (uint16_t)((accumulator.a2_cos_sum + count / 2U) / count);
    return true;
}

static bool take_factory_done(void)
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

static void install_calibration(const encoder_calibration_t *calibration,
                                uint32_t source,
                                uint32_t status_flags,
                                bool reset_state)
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    s_factory_calibration = *calibration;
    s_app_status_flags = status_flags;
    encoder_calibration_source = source;
    s_runtime_trim.enabled = true;
    encoder_runtime_trim_reset(&s_runtime_trim);
    if (reset_state)
    {
        encoder_state_init(&s_encoder_state);
    }
    EnableGlobalIRQ(irq_mask);
}

static void load_startup_configuration(void)
{
    uint32_t sequence;

    if (EncoderStorage_Load(&s_storage_config, &sequence))
    {
        (void)sequence;
        s_saved_quality = s_storage_config.quality;
        TFormat_LoadEeprom(s_storage_config.eeprom);
        s_has_persistent_config = true;
        install_calibration(&s_storage_config.calibration, ENCODER_CAL_SOURCE_NVM,
                            ENCODER_STATUS_OK, true);
        return;
    }

    memset(s_storage_config.eeprom, 0xFF, sizeof(s_storage_config.eeprom));
    memset(&s_saved_quality, 0, sizeof(s_saved_quality));
    TFormat_LoadEeprom(s_storage_config.eeprom);
    s_has_persistent_config = false;
    encoder_calibration_set_board_defaults(&s_storage_config.calibration);
    install_calibration(&s_storage_config.calibration,
                        ENCODER_CAL_SOURCE_DEFAULT,
                        ENCODER_STATUS_CAL_STORAGE_INVALID |
                            ENCODER_STATUS_FACTORY_CAL_REQUIRED,
                        true);
}

static void build_persistent_config(const encoder_calibration_t *calibration,
                                    const encoder_cal_quality_t *quality,
                                    encoder_persistent_config_t *config)
{
    config->calibration = *calibration;
    config->quality = *quality;
    TFormat_CopyEeprom(config->eeprom);
}

static storage_result_t save_configuration(const encoder_calibration_t *calibration,
                                           const encoder_cal_quality_t *quality)
{
    bool saved;

    if (!TFormat_StorageBegin())
    {
        return STORAGE_RESULT_DEFERRED;
    }
    build_persistent_config(calibration, quality, &s_storage_config);
    adc_realtime_stop();
    saved = EncoderStorage_Save(&s_storage_config);
    adc_realtime_start();
    TFormat_StorageEnd();
    return saved ? STORAGE_RESULT_SAVED : STORAGE_RESULT_FAILED;
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

    encoder_runtime_trim_apply(&s_factory_calibration, &s_runtime_trim, &calibration);
    if (!encoder_capture_zero(&calibration, &zero_sample, &status))
    {
        if (s_zero_origin == ZERO_ORIGIN_FM)
        {
            fm_command_state = FM_COMMAND_STATE_ERROR;
            fm_command_status = FM_COMMAND_STATUS_CALIBRATION;
        }
        else if (s_zero_origin == ZERO_ORIGIN_TFORMAT)
        {
            TFormat_ReportCountingError();
        }
        s_zero_origin = ZERO_ORIGIN_NONE;
        return;
    }

    install_calibration(&calibration,
                        encoder_calibration_source,
                        s_app_status_flags,
                        true);
    s_pending_calibration = calibration;
    s_pending_quality = s_saved_quality;
    s_save_pending = SAVE_PENDING_ZERO;
}

static void finish_factory_if_ready(void)
{
    encoder_calibration_t calibration;
    encoder_state_t state;
    encoder_result_t result;
    encoder_cal_quality_t quality;
    uint32_t status = ENCODER_STATUS_OK;

    if (!take_factory_done())
    {
        return;
    }

    if (!encoder_cal_stats_build(&s_factory_stats, &calibration, &status) ||
        !encoder_capture_zero(&calibration, &s_factory_zero_sample, &status))
    {
        fm_command_state = FM_COMMAND_STATE_ERROR;
        fm_command_status = FM_COMMAND_STATUS_CALIBRATION;
        load_startup_configuration();
        return;
    }

    encoder_state_init(&state);
    encoder_process(&state, &calibration, &s_factory_zero_sample, &result, NULL);
    quality.sample_count = ENCODER_CAL_SAMPLE_COUNT;
    quality.status = result.status;
    quality.mag16 = result.mag16_raw;
    quality.mag15 = result.mag15_raw;

    install_calibration(&calibration,
                        ENCODER_CAL_SOURCE_FACTORY_PENDING,
                        ENCODER_STATUS_OK,
                        true);
    s_pending_calibration = calibration;
    s_pending_quality = quality;
    s_save_pending = SAVE_PENDING_FACTORY;
}

static void complete_pending_save(void)
{
    storage_result_t result;
    bool saved;
    uint8_t save_kind;

    if ((s_save_pending == SAVE_PENDING_NONE) || (fm_encoder_stationary == 0U))
    {
        return;
    }

    save_kind = s_save_pending;
    result = save_configuration(&s_pending_calibration, &s_pending_quality);
    if (result == STORAGE_RESULT_DEFERRED)
    {
        return;
    }
    saved = result == STORAGE_RESULT_SAVED;
    s_save_pending = SAVE_PENDING_NONE;
    if (TFormat_EepromWritePending())
    {
        TFormat_CompleteEepromWrite(saved);
    }

    if (saved)
    {
        s_saved_quality = s_pending_quality;
        s_has_persistent_config = true;
        s_app_status_flags = ENCODER_STATUS_OK;
        encoder_calibration_source = ENCODER_CAL_SOURCE_NVM;
        s_autosave_done = true;
        if ((save_kind == SAVE_PENDING_ZERO) && (s_zero_origin == ZERO_ORIGIN_FM))
        {
            fm_command_state = FM_COMMAND_STATE_DONE;
            fm_command_status = FM_COMMAND_STATUS_OK;
        }
        else if (save_kind == SAVE_PENDING_FACTORY)
        {
            fm_command_state = FM_COMMAND_STATE_DONE;
            fm_command_status = FM_COMMAND_STATUS_OK;
        }
    }
    else if ((save_kind == SAVE_PENDING_ZERO) &&
             (s_zero_origin == ZERO_ORIGIN_TFORMAT))
    {
        TFormat_ReportCountingError();
    }
    else if ((save_kind == SAVE_PENDING_FACTORY) ||
             ((save_kind == SAVE_PENDING_ZERO) && (s_zero_origin == ZERO_ORIGIN_FM)))
    {
        fm_command_state = FM_COMMAND_STATE_ERROR;
        fm_command_status = FM_COMMAND_STATUS_STORAGE;
    }
    s_zero_origin = ZERO_ORIGIN_NONE;
}

static void reject_command(uint32_t status)
{
    fm_command_state = FM_COMMAND_STATE_ERROR;
    fm_command_status = status;
}

static void service_freemaster_command(void)
{
    uint8_t command;

    if (fm_command == FM_COMMAND_NONE)
    {
        return;
    }
    command = fm_command;
    fm_command = FM_COMMAND_NONE;

    if ((fm_command_state == FM_COMMAND_STATE_BUSY) || !capture_is_idle() ||
        (s_save_pending != SAVE_PENDING_NONE))
    {
        if (fm_command_state == FM_COMMAND_STATE_BUSY)
        {
            fm_command_status = FM_COMMAND_STATUS_BUSY;
        }
        else
        {
            reject_command(FM_COMMAND_STATUS_BUSY);
        }
        return;
    }

    fm_command_state = FM_COMMAND_STATE_BUSY;
    fm_command_status = FM_COMMAND_STATUS_OK;
    if (command == FM_COMMAND_ZERO_SAVE)
    {
        if (fm_encoder_ready == 0U)
        {
            reject_command(FM_COMMAND_STATUS_NOT_READY);
        }
        else if (fm_encoder_stationary == 0U)
        {
            reject_command(FM_COMMAND_STATUS_MOVING);
        }
        else
        {
            start_zero_capture(ZERO_ORIGIN_FM);
        }
    }
    else if (command == FM_COMMAND_FACTORY_CAL)
    {
        start_factory_calibration();
    }
    else if (command == FM_COMMAND_ERASE_CONFIG)
    {
        if (fm_encoder_stationary == 0U)
        {
            reject_command(FM_COMMAND_STATUS_MOVING);
            return;
        }
        s_erase_pending = true;
    }
    else
    {
        reject_command(FM_COMMAND_STATUS_CALIBRATION);
    }
}

static void service_tformat_resets(void)
{
    const uint32_t requests = TFormat_TakeResetRequests();

    if ((requests & TFORMAT_RESET_POSITION) == 0U)
    {
        return;
    }

    if ((fm_encoder_ready != 0U) && (fm_encoder_stationary != 0U) &&
        capture_is_idle() && (s_save_pending == SAVE_PENDING_NONE) && !s_erase_pending)
    {
        start_zero_capture(ZERO_ORIGIN_TFORMAT);
    }
    else
    {
        TFormat_ReportCountingError();
    }
}

static float center_drift(const encoder_calibration_t *a,
                          const encoder_calibration_t *b)
{
    const float drift[4] = {
        fabsf(a->a1.center_sin - b->a1.center_sin),
        fabsf(a->a1.center_cos - b->a1.center_cos),
        fabsf(a->a2.center_sin - b->a2.center_sin),
        fabsf(a->a2.center_cos - b->a2.center_cos),
    };
    float worst = drift[0];
    uint32_t i;

    for (i = 1U; i < 4U; i++)
    {
        if (drift[i] > worst)
        {
            worst = drift[i];
        }
    }
    return worst;
}

static float gain_drift(const encoder_calibration_t *a,
                        const encoder_calibration_t *b)
{
    const float drift[4] = {
        fabsf(a->a1.t00 - b->a1.t00) / fabsf(b->a1.t00),
        fabsf(a->a1.t11 - b->a1.t11) / fabsf(b->a1.t11),
        fabsf(a->a2.t00 - b->a2.t00) / fabsf(b->a2.t00),
        fabsf(a->a2.t11 - b->a2.t11) / fabsf(b->a2.t11),
    };
    float worst = drift[0];
    uint32_t i;

    for (i = 1U; i < 4U; i++)
    {
        if (drift[i] > worst)
        {
            worst = drift[i];
        }
    }
    return worst;
}

static void fold_effective_calibration(const encoder_calibration_t *effective)
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    s_factory_calibration = *effective;
    encoder_runtime_trim_reset(&s_runtime_trim);
    s_runtime_trim.enabled = true;
    EnableGlobalIRQ(irq_mask);
}

static void service_erase(void)
{
    bool erased;

    if (!s_erase_pending || (fm_encoder_stationary == 0U) ||
        !capture_is_idle() || (s_save_pending != SAVE_PENDING_NONE))
    {
        return;
    }
    if (!TFormat_StorageBegin())
    {
        return;
    }

    adc_realtime_stop();
    erased = EncoderStorage_Erase();
    load_startup_configuration();
    adc_realtime_start();
    TFormat_StorageEnd();
    s_autosave_done = false;
    s_erase_pending = false;
    fm_command_state = erased ? FM_COMMAND_STATE_DONE : FM_COMMAND_STATE_ERROR;
    fm_command_status = erased ? FM_COMMAND_STATUS_OK : FM_COMMAND_STATUS_STORAGE;
}

static void service_eeprom_write(void)
{
    encoder_calibration_t effective;
    encoder_cal_quality_t quality;
    storage_result_t result;
    bool saved;
    uint32_t irq_mask;

    if (!TFormat_EepromWritePending() || s_erase_pending ||
        (s_save_pending != SAVE_PENDING_NONE) ||
        !capture_is_idle() || (fm_encoder_stationary == 0U) ||
        (fm_encoder_ready == 0U))
    {
        return;
    }

    irq_mask = DisableGlobalIRQ();
    encoder_runtime_trim_apply(&s_factory_calibration, &s_runtime_trim, &effective);
    quality = s_saved_quality;
    EnableGlobalIRQ(irq_mask);

    result = save_configuration(&effective, &quality);
    if (result == STORAGE_RESULT_DEFERRED)
    {
        return;
    }
    saved = result == STORAGE_RESULT_SAVED;
    TFormat_CompleteEepromWrite(saved);
    if (saved)
    {
        fold_effective_calibration(&effective);
        s_has_persistent_config = true;
        s_app_status_flags = ENCODER_STATUS_OK;
        encoder_calibration_source = ENCODER_CAL_SOURCE_NVM;
    }
}

static void service_auto_save(void)
{
    encoder_calibration_t effective;
    encoder_cal_quality_t quality;
    uint32_t irq_mask;
    bool locked;
    uint32_t solves;

    if (s_autosave_done || s_erase_pending ||
        (s_save_pending != SAVE_PENDING_NONE) ||
        !capture_is_idle() || (fm_encoder_stationary == 0U) ||
        TFormat_EepromWritePending())
    {
        return;
    }

    irq_mask = DisableGlobalIRQ();
    locked = s_runtime_trim.has_locked;
    solves = s_runtime_trim.solve_count;
    encoder_runtime_trim_apply(&s_factory_calibration, &s_runtime_trim, &effective);
    quality.sample_count = solves;
    quality.status = encoder_result.status;
    quality.mag16 = encoder_result.mag16_raw;
    quality.mag15 = encoder_result.mag15_raw;
    EnableGlobalIRQ(irq_mask);

    if (!locked || (solves < ENCODER_AUTOSAVE_MIN_SOLVES))
    {
        return;
    }

    if (s_has_persistent_config &&
        (center_drift(&effective, &s_factory_calibration) <
         ENCODER_AUTOSAVE_CENTER_COUNTS) &&
        (gain_drift(&effective, &s_factory_calibration) <
         ENCODER_AUTOSAVE_GAIN_DELTA))
    {
        s_autosave_done = true;
        return;
    }

    {
        const storage_result_t result = save_configuration(&effective, &quality);

        if (result == STORAGE_RESULT_DEFERRED)
        {
            return;
        }
        if (result == STORAGE_RESULT_SAVED)
        {
            fold_effective_calibration(&effective);
            s_saved_quality = quality;
            s_has_persistent_config = true;
            s_app_status_flags = ENCODER_STATUS_OK;
            encoder_calibration_source = ENCODER_CAL_SOURCE_NVM;
        }
    }
    s_autosave_done = true;
}

void EncoderApp_Init(void)
{
    perf_dwt_init();
    encoder_runtime_trim_init(&s_runtime_trim);
    load_startup_configuration();

    fm_command = FM_COMMAND_NONE;
    fm_command_state = FM_COMMAND_STATE_IDLE;
    fm_command_status = FM_COMMAND_STATUS_OK;
    fm_factory_cal_progress = 0U;
    fm_encoder_ready = 0U;
    fm_encoder_stationary = 0U;

    adc_set_sample_callback(encoder_sample_callback);
    adc_realtime_start();
}

void EncoderApp_Service(void)
{
    publish_realtime_snapshot();
    consume_capture_sample();
    finish_zero_if_ready();
    finish_factory_if_ready();
    service_freemaster_command();
    service_tformat_resets();
    service_erase();
    complete_pending_save();
    service_eeprom_write();
    service_auto_save();
}
