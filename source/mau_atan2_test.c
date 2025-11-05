/* MAU vs C atan2 性能与精度测试 - 源文件 */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_clock.h"
#include "fsl_mau.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "mau_atan2_test.h"
#include "mau_atan2.h"

extern uint32_t DWT_GetCycles(void);

#define TEST_SAMPLES 1000

void MAU_Atan2PerformanceTest(void)
{
    printf("\r\n=== MAU atan2 Performance Test ===\r\n");

    /* Init MAU */
    mau_config_t mau_config;
    MAU_GetDefaultConfig(&mau_config);
    MAU_Init(MAU0, &mau_config);

    /* 生成测试数据：整圈 */
    static float test_x[TEST_SAMPLES];
    static float test_y[TEST_SAMPLES];
    static float result_c[TEST_SAMPLES];
    static float result_mau[TEST_SAMPLES];

    for (uint32_t i = 0; i < TEST_SAMPLES; i++) {
        float angle = (i * 360.0f / TEST_SAMPLES) * MAU_MATH_PI / 180.0f;
        test_x[i] = cosf(angle);
        test_y[i] = sinf(angle);
    }

    /* 测试1：标准 C atan2f */
    uint32_t cycles_start = DWT_GetCycles();
    for (uint32_t i = 0; i < TEST_SAMPLES; i++) {
        result_c[i] = atan2f(test_y[i], test_x[i]);
    }
    uint32_t cycles_c = DWT_GetCycles() - cycles_start;

    /* 测试2：MAU 加速的 atan2（封装 mau_atan2） */
    cycles_start = DWT_GetCycles();
    for (uint32_t i = 0; i < TEST_SAMPLES; i++) {
        result_mau[i] = mau_atan2f(test_y[i], test_x[i]);
    }
    uint32_t cycles_mau = DWT_GetCycles() - cycles_start;

    /* 计算精度 */
    float max_error = 0.0f;
    float sum_error = 0.0f;
    for (uint32_t i = 0; i < TEST_SAMPLES; i++) {
        float error = fabsf(result_c[i] - result_mau[i]);
        if (error > max_error) max_error = error;
        sum_error += error;
    }
    float avg_error = sum_error / TEST_SAMPLES;

    /* 打印结果 */
    uint32_t core_freq = CLOCK_GetCoreSysClkFreq();
    printf("\r\nResults (%lu samples):\r\n", (unsigned long)TEST_SAMPLES);
    printf("  C atan2f:     %lu cycles (%.2f us/call)\r\n",
           (unsigned long)cycles_c,
           (float)cycles_c / TEST_SAMPLES / core_freq * 1e6f);
    printf("  MAU atan2:    %lu cycles (%.2f us/call)\r\n",
           (unsigned long)cycles_mau,
           (float)cycles_mau / TEST_SAMPLES / core_freq * 1e6f);
    printf("  Speedup:      %.2fx\r\n", (float)cycles_c / cycles_mau);
    printf("\r\nAccuracy:\r\n");
    printf("  Max error:    %.6f rad (%.3f deg)\r\n", max_error, max_error * 180.0f / MAU_MATH_PI);
    printf("  Avg error:    %.6f rad (%.3f deg)\r\n", avg_error, avg_error * 180.0f / MAU_MATH_PI);
}