/* app_sampler.c - CTIMER1-based periodic sampling ISR */

#include "fsl_ctimer.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_common.h"

#include "app_sampler.h"
#include "app_timer.h"
#include "hardware_init.h"

/* Use CTIMER1 for periodic sampling to avoid interfering with CTIMER0 timing utility */
#define SAMPLER_CTIMER        CTIMER1
#define SAMPLER_CTIMER_IRQn   CTIMER1_IRQn

/* State */
static volatile uint8_t s_sampler_running = 0;
static uint32_t s_match_value = 0;
static volatile encoder_result_t s_last_encoder;
static volatile adc_sample_result_t s_last_raw;

/* ISR prototype */
void CTIMER1_IRQHandler(void);

void sampler_init(uint32_t freq_hz)
{
    ctimer_config_t config;

    /* Initialize encoder module state */
    encoder_init();

    uint32_t fro_hf_freq = CLOCK_GetFreq(kCLOCK_FroHf);
    uint32_t divider = 4;
    CLOCK_SetClockDiv(kCLOCK_DivCTIMER1, divider);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER1);
    
    /* Actual timer clock after divider */
    uint32_t ctimer_clk = fro_hf_freq / divider;  // 1MHz
    s_match_value = (ctimer_clk / freq_hz) - 1;   // -1 for 0-based counter

    CTIMER_GetDefaultConfig(&config);
    CTIMER_Init(SAMPLER_CTIMER, &config);

    /* Setup match on MR0: reset counter and generate interrupt */
    ctimer_match_config_t match0;
    match0.enableCounterReset = true;
    match0.enableCounterStop = false;
    match0.matchValue = s_match_value;
    match0.outControl = kCTIMER_Output_NoAction;
    match0.outPinInitState = false;
    match0.enableInterrupt = true;
    CTIMER_SetupMatch(SAMPLER_CTIMER, kCTIMER_Match_0, &match0);

    /* Enable IRQ */
    NVIC_SetPriority(SAMPLER_CTIMER_IRQn, 3); /* moderate priority */
    EnableIRQ(SAMPLER_CTIMER_IRQn);

    s_sampler_running = 0;
}

void sampler_start(void)
{
    if (!s_sampler_running) {
        s_sampler_running = 1;
        CTIMER_StartTimer(SAMPLER_CTIMER);
    }
}

void sampler_stop(void)
{
    if (s_sampler_running) {
        s_sampler_running = 0;
        CTIMER_StopTimer(SAMPLER_CTIMER);
    }
}

void sampler_copy_latest(encoder_result_t *out)
{
    if (!out) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = s_last_encoder;
    if (!primask) __enable_irq();
}

void sampler_copy_latest_raw(adc_sample_result_t *out)
{
    if (!out) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = s_last_raw;
    if (!primask) __enable_irq();
}

void CTIMER1_IRQHandler(void)
{
    /* Clear match0 interrupt flag */
    CTIMER_ClearStatusFlags(SAMPLER_CTIMER, kCTIMER_Match0Flag);
    
    TEST_PIN_Set();
    
    adc_sample_result_t raw = adc_read();

    
    s_last_raw = raw;

    encoder_result_t enc;
    encoder_process(raw.opamp0_out, raw.opamp1_out, &enc);
    s_last_encoder = enc;
    
    TEST_PIN_Clear();
}