


#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app_adc.h"
#include "app_timer.h"
#include "app_sampler.h"
#include "app_encoder.h"
#include "hardware_init.h"
#include "mcux_config.h"
#include "mau_atan2_test.h"
#include "app_uart_dma.h"
#include "app_tformat.h"

#include "freemaster.h"
#include "freemaster_serial_lpuart.h"
#include "freemaster_example.h"



#define SAMPLE_FRQ          (10*1000)

/* Application entry point for inductive encoder demo:
 * - Configures clocks, ADC, and periodic sampler
 * - Supports an auto-calibration routine to fit ellipse and set zero
 * - Streams angle/turns via UART in the main loop
 */


adc_sample_result_t adc_result;
encoder_result_t encoder_result;
static void perform_encoder_calibration(void);

#define CAL_TOTAL_ITERATIONS    (300000U)
#define CAL_TARGET_SAMPLES      (2048U)
#define CAL_PROGRESS_INTERVAL   (1000U)
#define CAL_ZERO_AVG_SAMPLES    (128U)



static void print_system_clocks(void)
{
    uint32_t freq;
    printf("=== System Clocks ===\r\n");
    freq = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    printf("Core Clock:    %u Hz (%u MHz)\r\n", freq, freq / 1000000U);
    freq = CLOCK_GetFreq(kCLOCK_FroHf);
    printf("FRO_HF:        %u Hz (%u MHz)\r\n", freq, freq / 1000000U);
    freq = CLOCK_GetFreq(kCLOCK_FroHfDiv);
    printf("FRO_HF_DIV:    %u Hz (%u MHz)\r\n", freq, freq / 1000000U);
}

static void start_sampling_default(void)
{
    sampler_init(SAMPLE_FRQ);
    sampler_start();
    encoder_set_direction(-1);
}

static void stream_encoder_loop(void)
{
    while (1)
    {
        FMSTR_Example_Poll();
        sampler_copy_latest(&encoder_result);
        sampler_copy_latest_raw(&adc_result);
        SDK_DelayAtLeastUs(10000, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    }
}

static void run_uart_dma_demo(void)
{
    uart_dma_demo_init();
    uart_dma_demo_run();
    while (1)
    {
    }
}

static void run_tformat_slave_demo(void)
{
    tformat_init();
    adc_init(ADC_MODE_OUTPUT_ONLY);
    
    BOARD_InitUART485Control(LPUART2, 1);
    
    start_sampling_default();
    tformat_slave_loop();
    while (1)
    {
    }
}

/* ---------- Mode runners and dispatch ---------- */
static void run_ascii_stream_mode(void)
{
    adc_init(ADC_MODE_OUTPUT_ONLY);
    start_sampling_default();
    stream_encoder_loop();
}

static void run_debug_mode(void)
{
    adc_init(ADC_MODE_FULL_DEBUG);
    start_sampling_default();
    stream_encoder_loop();
}

static void run_calibration_then_ascii(void)
{
    adc_init(ADC_MODE_CALIBRATION);
    perform_encoder_calibration();
    adc_init(ADC_MODE_OUTPUT_ONLY);
    start_sampling_default();
    stream_encoder_loop();
}

static void run_freemaster_mode(void)
{
    adc_init(ADC_MODE_OUTPUT_ONLY);
    start_sampling_default();
    
    FMSTR_SerialSetBaseAddress((LPUART_Type*)LPUART2);
    FMSTR_Example_Init();
    
#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR
    NVIC_SetPriority(LPUART2_IRQn, 0);
    EnableIRQ(LPUART2_IRQn);
#endif
    stream_encoder_loop();
}



static void perform_encoder_calibration(void)
{
    printf("\r\n=== Auto Calibration ===\r\n");
    printf("Rotate encoder slowly for one full circle...\r\n");
    printf("Press any key to start...\r\n");
    GETCHAR();

    uint16_t sin_min = 65535, sin_max = 0;
    uint16_t cos_min = 65535, cos_max = 0;

    double sum_sin = 0.0;
    double sum_cos = 0.0;
    double sum_sin2 = 0.0;
    double sum_cos2 = 0.0;
    double sum_sincos = 0.0;
    uint32_t sample_count = 0;

    uint32_t stride = (CAL_TARGET_SAMPLES > 0) ? (CAL_TOTAL_ITERATIONS / CAL_TARGET_SAMPLES) : 1U;
    if (stride == 0U) {
        stride = 1U;
    }

    for (uint32_t i = 0; i < CAL_TOTAL_ITERATIONS; i++) {
        adc_sample_result_t result = adc_read();

        if (result.opamp0_out < sin_min) sin_min = result.opamp0_out;
        if (result.opamp0_out > sin_max) sin_max = result.opamp0_out;
        if (result.opamp1_out < cos_min) cos_min = result.opamp1_out;
        if (result.opamp1_out > cos_max) cos_max = result.opamp1_out;

        if ((i % stride) == 0U && sample_count < CAL_TARGET_SAMPLES) {
            double sin_raw = (double)result.opamp0_out;
            double cos_raw = (double)result.opamp1_out;
            sum_sin += sin_raw;
            sum_cos += cos_raw;
            sum_sin2 += sin_raw * sin_raw;
            sum_cos2 += cos_raw * cos_raw;
            sum_sincos += sin_raw * cos_raw;
            sample_count++;
        }

        if ((i % CAL_PROGRESS_INTERVAL) == 0U) {
            printf("  Progress: %lu/%lu\r\n", (unsigned long)i, (unsigned long)CAL_TOTAL_ITERATIONS);
        }

        SDK_DelayAtLeastUs(100, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    }

    if (sample_count < 16U) {
        printf("Not enough samples (%lu). Falling back to min/max calibration.\r\n", (unsigned long)sample_count);
        encoder_calibrate(sin_min, sin_max, cos_min, cos_max);
        return;
    }

    double invN = 1.0 / (double)sample_count;
    double mean_sin = sum_sin * invN;
    double mean_cos = sum_cos * invN;
    double var_sin = (sum_sin2 * invN) - mean_sin * mean_sin;
    double var_cos = (sum_cos2 * invN) - mean_cos * mean_cos;
    double cov_sc = (sum_sincos * invN) - mean_sin * mean_cos;

    const double min_var = 1.0;
    if (var_sin < min_var) var_sin = min_var;
    if (var_cos < min_var) var_cos = min_var;

    float a = (float)var_sin;
    float b = (float)cov_sc;
    float d = (float)var_cos;

    float trace = a + d;
    float delta = trace * 0.5f * trace * 0.5f - (a * d - b * b);
    if (delta < 0.0f) delta = 0.0f;
    float root = sqrtf(delta);
    float lambda1 = trace * 0.5f + root;
    float lambda2 = trace * 0.5f - root;

    if (lambda1 < 1e-6f) lambda1 = 1e-6f;
    if (lambda2 < 1e-6f) lambda2 = 1e-6f;

    float v1x = 1.0f, v1y = 0.0f;
    float v2x = 0.0f, v2y = 1.0f;

    if (fabsf(b) > 1e-3f) {
        v1x = lambda1 - d;
        v1y = b;
        float norm = sqrtf(v1x * v1x + v1y * v1y);
        if (norm > 1e-6f) {
            v1x /= norm;
            v1y /= norm;
        } else {
            v1x = 1.0f;
            v1y = 0.0f;
        }
        v2x = -v1y;
        v2y = v1x;
    } else {
        if (a < d) {
            v1x = 0.0f; v1y = 1.0f;
            v2x = 1.0f; v2y = 0.0f;
        }
    }

    float scale1 = 1.0f / sqrtf(2.0f * lambda1);
    float scale2 = 1.0f / sqrtf(2.0f * lambda2);

    encoder_calibration_t cal;
    cal.sin_center = (float)mean_sin;
    cal.cos_center = (float)mean_cos;
    cal.transform[0][0] = scale1 * v1x;
    cal.transform[0][1] = scale1 * v1y;
    cal.transform[1][0] = scale2 * v2x;
    cal.transform[1][1] = scale2 * v2y;

    float det_t = cal.transform[0][0] * cal.transform[1][1] - cal.transform[0][1] * cal.transform[1][0];
    if (det_t < 0.0f) {
        cal.transform[1][0] = -cal.transform[1][0];
        cal.transform[1][1] = -cal.transform[1][1];
    }

    if (!isfinite(cal.transform[0][0]) || !isfinite(cal.transform[0][1]) ||
        !isfinite(cal.transform[1][0]) || !isfinite(cal.transform[1][1])) {
        printf("Ellipse fit produced invalid values. Falling back to min/max calibration.\r\n");
        encoder_calibrate(sin_min, sin_max, cos_min, cos_max);
        return;
    }

    printf("Collected %lu samples for ellipse fit.\r\n", (unsigned long)sample_count);
    printf("  Means: sin=%.3f cos=%.3f\r\n", cal.sin_center, cal.cos_center);
    printf("  Variance: sin=%.3f cos=%.3f cov=%.3f\r\n", a, d, b);

    encoder_apply_calibration(&cal);

    printf("Align mechanical zero position and press any key...\r\n");
    GETCHAR();

    uint64_t sin_acc = 0;
    uint64_t cos_acc = 0;
    for (uint32_t i = 0; i < CAL_ZERO_AVG_SAMPLES; i++) {
        adc_sample_result_t result = adc_read();
        sin_acc += result.opamp0_out;
        cos_acc += result.opamp1_out;
        SDK_DelayAtLeastUs(200, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    }

    uint16_t sin_avg = (uint16_t)((sin_acc + CAL_ZERO_AVG_SAMPLES / 2U) / CAL_ZERO_AVG_SAMPLES);
    uint16_t cos_avg = (uint16_t)((cos_acc + CAL_ZERO_AVG_SAMPLES / 2U) / CAL_ZERO_AVG_SAMPLES);

    encoder_result_t zero_result;
    encoder_process(sin_avg, cos_avg, &zero_result);
    float zero_angle = zero_result.angle_deg;
    encoder_init();
    encoder_set_zero_deg(zero_angle);

    printf("=== Calibration Complete ===\r\n");
    printf("OPAMP0_OUT (Sin): [%u, %u]\r\n", sin_min, sin_max);
    printf("OPAMP1_OUT (Cos): [%u, %u]\r\n", cos_min, cos_max);
    printf("Zero offset set to %.2f deg\r\n\r\n", zero_angle);
}




int main(void)
{
    /* Init board hardware */
    BOARD_InitHardware();
    

    CLOCK_SetupExtClocking(8*1000*1000);              // Enable the 8MHz external crystal

    print_system_clocks();
    
    MAU_Atan2PerformanceTest();
    
    switch (APP_START_MODE) {
    case APP_MODE_ASCII:
        run_ascii_stream_mode();
        break;
    case APP_MODE_DEBUG:
        run_debug_mode();
        break;
    case APP_MODE_CALIBRATE_ASCII:
        run_calibration_then_ascii();
        break;
    case APP_MODE_UART_DMA:
        run_uart_dma_demo();
        break;
    case APP_MODE_FREEMASTER:
    default:
        run_freemaster_mode();
        break;
    }
    /* unreachable: mode handlers are blocking */
    while (1) {}
}


#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR

void LPUART2_IRQHandler(void)
{
    FMSTR_SerialIsr();
}
#endif


void HardFault_Handler(void)
{
    printf("HardFault_Handler\r\n");
    while(1);
}


