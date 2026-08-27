#include "app_encoder_runtime.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "fsl_common.h"
#include "fsl_device_registers.h"

#include "app_adc.h"
#include "app_encoder_storage.h"
#include "app_freemaster.h"
#include "app_tformat.h"

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
/* Recovery: wipe stored calibration and fall back to the compiled-in defaults,
 * from which the runtime trim re-acquires. Without this the only way out of a
 * bad stored calibration is a reflash. */
#define FACTORY_CAL_CMD_ERASE   2U

typedef struct {
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

volatile uint32_t encoder_perf_process_cycles;
volatile uint32_t encoder_perf_process_max;
volatile uint32_t encoder_perf_isr_cycles;
volatile uint32_t encoder_perf_isr_max;
volatile uint32_t encoder_perf_core_clock_hz;

static encoder_calibration_t s_factory_calibration;
static encoder_state_t s_encoder_state;
static encoder_runtime_trim_t s_runtime_trim;
/* Harmonic correction prototype. Runtime learning and persistence are not wired
 * yet, so the invalid table remains a no-op in encoder_process(). */
static encoder_inl_t s_encoder_inl;
static uint32_t s_app_status_flags;

static volatile encoder_result_t s_realtime_encoder_result;
static volatile uint8_t s_realtime_encoder_valid;
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
/* Latched per boot, and re-armed by a fresh factory calibration. */
static bool s_autosave_done;

/* Cortex-M33 DWT cycle counter — free, 1-cycle precision profiling source.
 * Has to be unlocked once via TRCENA before CYCCNT will increment. */
static void perf_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    encoder_perf_core_clock_hz = SystemCoreClock;
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
    uint32_t cyc_isr_start;
    uint32_t cyc_proc_start;
    uint32_t cyc_proc;
    uint32_t cyc_isr;

    /* Publish raw ADC sample for the FreeMASTER oscilloscope. */
    adc_result = *sample;

    cyc_isr_start = DWT->CYCCNT;
    encoder_runtime_trim_apply(&s_factory_calibration, &s_runtime_trim, &effective_calibration);

    cyc_proc_start = DWT->CYCCNT;
    encoder_process(&s_encoder_state,
                    &effective_calibration,
                    &s_encoder_inl,
                    &encoder_sample,
                    &next_encoder_result,
                    NULL);
    cyc_proc = DWT->CYCCNT - cyc_proc_start;

    /* Runs against whatever calibration is active, board defaults included. Gating
     * this on a stored calibration made the online estimator useless exactly when
     * it was needed most: a board that has never been calibrated is the one whose
     * defaults are furthest from the truth, and it could not converge because it
     * had not been calibrated. */
    encoder_runtime_trim_update(&s_runtime_trim,
                                &s_factory_calibration,
                                &encoder_sample,
                                &next_encoder_result);

    next_encoder_result.status |= s_app_status_flags;

    s_realtime_encoder_result = next_encoder_result;
    s_realtime_encoder_valid = s_factory_calibration.valid ? 1U : 0U;
    s_realtime_sequence++;

    TFormat_Publish(next_encoder_result.angle_counts,
                    next_encoder_result.status,
                    s_factory_calibration.valid);

    capture_sample_from_isr(&encoder_sample);

    cyc_isr = DWT->CYCCNT - cyc_isr_start;
    encoder_perf_process_cycles = cyc_proc;
    encoder_perf_isr_cycles = cyc_isr;
    if (cyc_proc > encoder_perf_process_max)
    {
        encoder_perf_process_max = cyc_proc;
    }
    if (cyc_isr > encoder_perf_isr_max)
    {
        encoder_perf_isr_max = cyc_isr;
    }
}

static void publish_realtime_snapshot(void)
{
    uint32_t irq_mask;

    irq_mask = DisableGlobalIRQ();
    if (s_published_sequence != s_realtime_sequence)
    {
        encoder_result = s_realtime_encoder_result;
        fm_encoder_valid = s_realtime_encoder_valid;
        s_published_sequence = s_realtime_sequence;
    }
    adc_sample_count = adc_get_sample_count();
    adc_overrun_count = adc_get_overrun_count();
    EnableGlobalIRQ(irq_mask);

#if defined(DEBUG)
    TFormat_PublishDiagnostics(encoder_result.status,
                               fm_encoder_valid != 0U,
                               encoder_calibration_source,
                               encoder_result.mag16_raw,
                               encoder_result.mag15_raw,
                               adc_sample_count,
                               adc_overrun_count);
#endif
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
    EnableGlobalIRQ(irq_mask);

    s_autosave_done = false;
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
                              uint32_t app_status_flags)
{
    uint32_t irq_mask = DisableGlobalIRQ();

    s_factory_calibration = *calibration;
    s_app_status_flags = app_status_flags;
    encoder_calibration_source = source;
    /* Always on. start_factory_calibration() still switches it off for the
     * duration of a capture so the two do not fight over the same coefficients. */
    s_runtime_trim.enabled = true;
    encoder_runtime_trim_reset(&s_runtime_trim);
    encoder_state_init(&s_encoder_state);
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
        apply_calibration(&calibration, ENCODER_CAL_SOURCE_NVM, ENCODER_STATUS_OK);
    }
    else
    {
        encoder_calibration_set_board_defaults(&calibration);
        apply_calibration(&calibration,
                          ENCODER_CAL_SOURCE_DEFAULT,
                          ENCODER_STATUS_CAL_STORAGE_INVALID |
                              ENCODER_STATUS_FACTORY_CAL_REQUIRED);
    }
}

static void apply_zero(const encoder_calibration_t *calibration)
{
    apply_calibration(calibration, encoder_calibration_source, s_app_status_flags);
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

    /* Zero against the calibration actually in force, trim included. Using the
     * untrimmed baseline would install it as the new one and reset the trim,
     * discarding a converged acquisition every time the user zeroes -- harmless
     * back when the trim was a small correction, but it is the primary
     * calibration path now. */
    encoder_runtime_trim_apply(&s_factory_calibration, &s_runtime_trim, &calibration);
    if (!calibration.valid)
    {
        return;
    }

    if (encoder_capture_zero(&calibration, &s_encoder_inl, &zero_sample, &status))
    {
        apply_zero(&calibration);
        fm_encoder_valid = 1U;
    }
}

static void finish_factory_calibration_if_ready(void)
{
    encoder_calibration_t calibration;
    encoder_state_t temp_state;
    encoder_result_t temp_result;
    encoder_cal_quality_t quality;
    uint32_t status = ENCODER_STATUS_OK;
    bool saved;

    if (!take_factory_calibration_done())
    {
        return;
    }

    if (!encoder_cal_stats_build(&s_factory_stats, &calibration, &status) ||
        !encoder_capture_zero(&calibration, &s_encoder_inl, &s_factory_zero_sample, &status))
    {
        fm_factory_cal_state = ENCODER_FACTORY_CAL_STATE_FAILED;
        fm_factory_cal_status = status | ENCODER_STATUS_CAL_FAILED;
        load_startup_calibration();
        return;
    }

    encoder_state_init(&temp_state);
    encoder_process(&temp_state, &calibration, &s_encoder_inl, &s_factory_zero_sample, &temp_result, NULL);
    quality.sample_count = ENCODER_CAL_SAMPLE_COUNT;
    quality.status = temp_result.status;
    quality.mag16 = temp_result.mag16;
    quality.mag15 = temp_result.mag15;

    adc_realtime_stop();
    saved = EncoderStorage_SaveFactoryCalibration(&calibration, &quality);
    adc_realtime_start();

    if (saved)
    {
        apply_calibration(&calibration, ENCODER_CAL_SOURCE_NVM, ENCODER_STATUS_OK);
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

    if (fm_turn_reset_ctrl != 0U)
    {
        const uint8_t cmd = fm_turn_reset_ctrl;
        uint32_t irq_mask;

        fm_turn_reset_ctrl = 0U;
        if (cmd == 1U)
        {
            irq_mask = DisableGlobalIRQ();
            encoder_state_reset_turn_count(&s_encoder_state);
            EnableGlobalIRQ(irq_mask);
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
        else if ((cmd == FACTORY_CAL_CMD_ERASE) && capture_is_idle())
        {
            adc_realtime_stop();
            (void)EncoderStorage_EraseFactoryCalibration();
            adc_realtime_start();
            load_startup_calibration();
            s_autosave_done = false;
            fm_factory_cal_state = ENCODER_FACTORY_CAL_STATE_IDLE;
            fm_factory_cal_status = ENCODER_STATUS_OK;
        }
    }
}

/* Persist the converged calibration so the next boot starts converged instead of
 * spending a revolution re-acquiring. That is the difference between "plug it in
 * and it works" and "plug it in, turn it once, then it works".
 *
 * Steady state is zero writes: a board that boots from a good stored calibration
 * produces a small trim delta and never trips the threshold. Only a board running
 * on defaults, or one that has genuinely drifted, writes anything. That matters
 * because the whole 8 KB calibration area is a single erase sector -- filling it
 * forces an erase, and a power loss inside that window drops every stored slot.
 * The consequence is now bounded though: losing the store falls back to defaults,
 * and the runtime trim re-acquires within a revolution. */
#define ENCODER_AUTOSAVE_MIN_SOLVES (8U)
#define ENCODER_AUTOSAVE_CENTRE_COUNTS (16.0f)

static float centre_drift(const encoder_calibration_t *a, const encoder_calibration_t *b)
{
    float worst = fabsf(a->a1.center_sin - b->a1.center_sin);
    const float candidates[3] = {
        fabsf(a->a1.center_cos - b->a1.center_cos),
        fabsf(a->a2.center_sin - b->a2.center_sin),
        fabsf(a->a2.center_cos - b->a2.center_cos),
    };
    uint32_t i;

    for (i = 0U; i < 3U; i++)
    {
        if (candidates[i] > worst)
        {
            worst = candidates[i];
        }
    }

    return worst;
}

static void service_auto_save(void)
{
    encoder_calibration_t effective;
    encoder_cal_quality_t quality;
    uint32_t irq_mask;
    uint32_t solves;
    bool locked;

    if (s_autosave_done || !capture_is_idle())
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

    /* A board already running from NVM only rewrites if it has actually moved. */
    if ((encoder_calibration_source == ENCODER_CAL_SOURCE_NVM) &&
        (centre_drift(&effective, &s_factory_calibration) < ENCODER_AUTOSAVE_CENTRE_COUNTS))
    {
        s_autosave_done = true;
        return;
    }

    adc_realtime_stop();
    if (EncoderStorage_SaveFactoryCalibration(&effective, &quality))
    {
        /* Deliberately does not re-apply: apply_calibration() would reset the
         * encoder state and glitch the live output. The trim keeps its deltas, so
         * the effective calibration is unchanged either way, and the next boot
         * loads the already-trimmed values with a zero delta. */
        encoder_calibration_source = ENCODER_CAL_SOURCE_NVM;
        s_app_status_flags = ENCODER_STATUS_OK;
    }
    adc_realtime_start();
    s_autosave_done = true;
}

static void service_encoder_capture(void)
{
    consume_capture_sample();
    finish_zero_if_ready();
    finish_factory_calibration_if_ready();
}

void EncoderApp_Init(void)
{
    perf_dwt_init();
    encoder_inl_init(&s_encoder_inl);
    encoder_runtime_trim_init(&s_runtime_trim);
    load_startup_calibration();

    fm_encoder_valid = s_factory_calibration.valid ? 1U : 0U;
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
    service_auto_save();
}
