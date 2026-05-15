#include "app_encoder_v2.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_CENTER 32768.0f
#define TEST_AMPLITUDE 12000.0f

static float wrap_deg(float deg)
{
    while (deg >= 360.0f)
    {
        deg -= 360.0f;
    }

    while (deg < 0.0f)
    {
        deg += 360.0f;
    }

    return deg;
}

static float abs_angle_error(float a, float b)
{
    float diff = wrap_deg(a - b);
    if (diff > 180.0f)
    {
        diff -= 360.0f;
    }

    return fabsf(diff);
}

static uint16_t synth_raw(float value)
{
    if (value < 0.0f)
    {
        value = 0.0f;
    }
    else if (value > 65535.0f)
    {
        value = 65535.0f;
    }

    return (uint16_t)(value + 0.5f);
}

static v2_encoder_raw_sample_t make_sample_from_a1_a2_phases(float phase_a1_deg, float phase_a2_deg)
{
    const float p1 = wrap_deg(phase_a1_deg) * TEST_PI / 180.0f;
    const float p2 = wrap_deg(phase_a2_deg) * TEST_PI / 180.0f;
    v2_encoder_raw_sample_t sample;

    sample.a1_sin_raw = synth_raw(TEST_CENTER + (TEST_AMPLITUDE * sinf(p1)));
    sample.a1_cos_raw = synth_raw(TEST_CENTER + (TEST_AMPLITUDE * cosf(p1)));
    sample.a2_sin_raw = synth_raw(TEST_CENTER + (TEST_AMPLITUDE * sinf(p2)));
    sample.a2_cos_raw = synth_raw(TEST_CENTER + (TEST_AMPLITUDE * cosf(p2)));

    return sample;
}

static v2_encoder_raw_sample_t make_sample_from_centers(float a1_phase_deg,
                                                        float a2_phase_deg,
                                                        float a1_sin_center,
                                                        float a1_cos_center,
                                                        float a2_sin_center,
                                                        float a2_cos_center,
                                                        float amplitude)
{
    const float p1 = wrap_deg(a1_phase_deg) * TEST_PI / 180.0f;
    const float p2 = wrap_deg(a2_phase_deg) * TEST_PI / 180.0f;
    v2_encoder_raw_sample_t sample;

    sample.a1_sin_raw = synth_raw(a1_sin_center + (amplitude * sinf(p1)));
    sample.a1_cos_raw = synth_raw(a1_cos_center + (amplitude * cosf(p1)));
    sample.a2_sin_raw = synth_raw(a2_sin_center + (amplitude * sinf(p2)));
    sample.a2_cos_raw = synth_raw(a2_cos_center + (amplitude * cosf(p2)));

    return sample;
}

static v2_encoder_raw_sample_t make_sample(float mechanical_deg)
{
    return make_sample_from_a1_a2_phases(15.0f * mechanical_deg, 16.0f * mechanical_deg);
}

static v2_encoder_raw_sample_t make_sample_with_common_phase_error(float mechanical_deg, float common_error_deg)
{
    return make_sample_from_a1_a2_phases((15.0f * mechanical_deg) + common_error_deg,
                                         (16.0f * mechanical_deg) + common_error_deg);
}

static v2_encoder_calibration_t make_identity_calibration(void)
{
    v2_encoder_calibration_t calibration;

    v2_encoder_calibration_set_defaults(&calibration);
    calibration.a1.center_sin = TEST_CENTER;
    calibration.a1.center_cos = TEST_CENTER;
    calibration.a1.transform[0][0] = 1.0f / TEST_AMPLITUDE;
    calibration.a1.transform[0][1] = 0.0f;
    calibration.a1.transform[1][0] = 0.0f;
    calibration.a1.transform[1][1] = 1.0f / TEST_AMPLITUDE;
    calibration.a2 = calibration.a1;
    calibration.phase_a1_zero_deg = 0.0f;
    calibration.phase_a2_zero_deg = 0.0f;
    calibration.valid = true;

    return calibration;
}

static void assert_close_deg(const char *name, float actual, float expected, float tolerance)
{
    const float error = abs_angle_error(actual, expected);
    if (error > tolerance)
    {
        fprintf(stderr, "%s: actual %.4f expected %.4f error %.4f\n", name, actual, expected, error);
        exit(1);
    }
}

static float expected_output_angle(float mechanical_deg)
{
    return wrap_deg(mechanical_deg);
}

static float expected_output_angle_with_common_phase_error(float mechanical_deg, float common_error_deg)
{
    (void)common_error_deg;
    return wrap_deg(mechanical_deg);
}

static void test_vernier_candidate_selection(void)
{
    static const float test_angles[] = {0.0f, 3.0f, 22.4f, 123.45f, 270.0f, 359.1f};
    v2_encoder_calibration_t calibration = make_identity_calibration();
    v2_encoder_state_t state;
    v2_encoder_result_t result;
    size_t i;

    v2_encoder_state_init(&state);

    for (i = 0; i < (sizeof(test_angles) / sizeof(test_angles[0])); i++)
    {
        const float angle = test_angles[i];
        const v2_encoder_raw_sample_t sample = make_sample(angle);

        v2_encoder_process(&state, &calibration, &sample, &result);

        if ((result.status & V2_ENCODER_STATUS_TRACK_MISMATCH) != 0U)
        {
            fprintf(stderr, "unexpected mismatch at %.2f deg, status=0x%08lX\n",
                    angle,
                    (unsigned long)result.status);
            exit(1);
        }

        assert_close_deg("output angle", result.angle_deg, expected_output_angle(angle), 0.05f);
    }
}

static void test_zero_capture(void)
{
    v2_encoder_calibration_t calibration = make_identity_calibration();
    v2_encoder_state_t state;
    v2_encoder_result_t result;
    v2_encoder_raw_sample_t zero_sample = make_sample(15.0f);
    v2_encoder_raw_sample_t target_sample = make_sample(40.0f);
    uint32_t status = 0U;

    if (!v2_encoder_capture_zero(&calibration, &zero_sample, &status))
    {
        fprintf(stderr, "zero capture failed, status=0x%08lX\n", (unsigned long)status);
        exit(1);
    }

    v2_encoder_state_init(&state);
    v2_encoder_process(&state, &calibration, &target_sample, &result);
    assert_close_deg("zero-relative angle", result.angle_deg, expected_output_angle(25.0f), 0.05f);
}

static void test_fault_holds_last_angle(void)
{
    v2_encoder_calibration_t calibration = make_identity_calibration();
    v2_encoder_state_t state;
    v2_encoder_result_t result;
    v2_encoder_raw_sample_t good_sample = make_sample(30.0f);
    v2_encoder_raw_sample_t bad_sample;

    v2_encoder_state_init(&state);
    v2_encoder_process(&state, &calibration, &good_sample, &result);
    assert_close_deg("initial angle", result.angle_deg, expected_output_angle(30.0f), 0.05f);

    bad_sample = good_sample;
    bad_sample.a2_sin_raw = 0U;
    v2_encoder_process(&state, &calibration, &bad_sample, &result);

    if ((result.status & V2_ENCODER_STATUS_ADC_RAIL) == 0U)
    {
        fprintf(stderr, "expected ADC rail status, got 0x%08lX\n", (unsigned long)result.status);
        exit(1);
    }

    if ((result.status & V2_ENCODER_STATUS_HOLD_LAST) == 0U)
    {
        fprintf(stderr, "expected hold-last status, got 0x%08lX\n", (unsigned long)result.status);
        exit(1);
    }

    assert_close_deg("held angle", result.angle_deg, expected_output_angle(30.0f), 0.05f);
}

static void test_stats_calibration_build(void)
{
    v2_encoder_cal_stats_t stats;
    v2_encoder_calibration_t calibration;
    v2_encoder_state_t state;
    v2_encoder_result_t result;
    v2_encoder_raw_sample_t zero_sample = make_sample(0.0f);
    v2_encoder_raw_sample_t target_sample = make_sample(137.25f);
    uint32_t status = 0U;
    uint32_t i;

    v2_encoder_cal_stats_init(&stats);
    for (i = 0; i < V2_ENCODER_CAL_SAMPLE_COUNT; i++)
    {
        const float angle = (360.0f * (float)i) / (float)V2_ENCODER_CAL_SAMPLE_COUNT;
        const v2_encoder_raw_sample_t sample = make_sample(angle);

        v2_encoder_cal_stats_accumulate(&stats, &sample);
    }

    if (!v2_encoder_cal_stats_build(&stats, &calibration, &status))
    {
        fprintf(stderr, "calibration build failed, status=0x%08lX\n", (unsigned long)status);
        exit(1);
    }

    if (!v2_encoder_capture_zero(&calibration, &zero_sample, &status))
    {
        fprintf(stderr, "zero capture after build failed, status=0x%08lX\n", (unsigned long)status);
        exit(1);
    }

    v2_encoder_state_init(&state);
    v2_encoder_process(&state, &calibration, &target_sample, &result);
    assert_close_deg("stats-calibrated angle", result.angle_deg, expected_output_angle(137.25f), 0.1f);
}

static void test_calibration_uses_extrema_not_sample_density(void)
{
    v2_encoder_cal_stats_t stats;
    v2_encoder_calibration_t calibration;
    uint32_t status = 0U;
    uint32_t i;

    v2_encoder_cal_stats_init(&stats);
    for (i = 0U; i < V2_ENCODER_CAL_SAMPLE_COUNT; i++)
    {
        const float angle = (360.0f * (float)i) / (float)V2_ENCODER_CAL_SAMPLE_COUNT;
        const v2_encoder_raw_sample_t sample = make_sample_from_centers(15.0f * angle,
                                                                        16.0f * angle,
                                                                        21000.0f,
                                                                        32000.0f,
                                                                        26000.0f,
                                                                        29000.0f,
                                                                        9000.0f);
        v2_encoder_cal_stats_accumulate(&stats, &sample);
    }

    for (i = 0U; i < 2048U; i++)
    {
        const float angle = 92.0f + (0.01f * (float)(i % 11U));
        const v2_encoder_raw_sample_t sample = make_sample_from_centers(15.0f * angle,
                                                                        16.0f * angle,
                                                                        21000.0f,
                                                                        32000.0f,
                                                                        26000.0f,
                                                                        29000.0f,
                                                                        9000.0f);
        v2_encoder_cal_stats_accumulate(&stats, &sample);
    }

    if (!v2_encoder_cal_stats_build(&stats, &calibration, &status))
    {
        fprintf(stderr, "density-biased calibration build failed, status=0x%08lX\n", (unsigned long)status);
        exit(1);
    }

    if ((fabsf(calibration.a1.center_sin - 21000.0f) > 2.0f) ||
        (fabsf(calibration.a1.center_cos - 32000.0f) > 2.0f) ||
        (fabsf(calibration.a2.center_sin - 26000.0f) > 2.0f) ||
        (fabsf(calibration.a2.center_cos - 29000.0f) > 2.0f))
    {
        fprintf(stderr,
                "calibration center biased: a1=(%.2f, %.2f) a2=(%.2f, %.2f)\n",
                calibration.a1.center_sin,
                calibration.a1.center_cos,
                calibration.a2.center_sin,
                calibration.a2.center_cos);
        exit(1);
    }
}

static void test_fixed_a1_15_a2_16_mapping_is_used(void)
{
    v2_encoder_calibration_t calibration = make_identity_calibration();
    v2_encoder_state_t state;
    v2_encoder_result_t result;
    v2_encoder_diag_t diag;
    v2_encoder_raw_sample_t sample = make_sample(91.5f);

    v2_encoder_state_init(&state);
    v2_encoder_process_with_diag(&state, &calibration, &sample, &result, &diag);

    assert_close_deg("fixed mapping output angle", result.angle_deg, expected_output_angle(91.5f), 0.05f);
    assert_close_deg("coarse diagnostic angle", diag.coarse_angle_deg, 91.5f, 0.05f);
    assert_close_deg("phase16 error", diag.phase16_error_deg, 0.0f, 0.05f);
    assert_close_deg("phase15 error", diag.phase15_error_deg, 0.0f, 0.05f);
    if ((diag.mapping != V2_ENCODER_MAPPING_A1_15_A2_16) || (diag.dir16 != 1) || (diag.dir15 != 1))
    {
        fprintf(stderr,
                "expected A1=15/A2=16 dir16=1 dir15=1, got mapping=%u dir16=%d dir15=%d\n",
                (unsigned)diag.mapping,
                (int)diag.dir16,
                (int)diag.dir15);
        exit(1);
    }
}

static void test_diagnostic_coarse_cancels_common_phase_error(void)
{
    v2_encoder_calibration_t calibration = make_identity_calibration();
    v2_encoder_state_t state;
    v2_encoder_result_t result;
    v2_encoder_diag_t diag;
    v2_encoder_raw_sample_t sample = make_sample_with_common_phase_error(142.0f, 11.0f);

    v2_encoder_state_init(&state);
    v2_encoder_process_with_diag(&state, &calibration, &sample, &result, &diag);

    assert_close_deg("diagnostic coarse angle", diag.coarse_angle_deg, 142.0f, 0.05f);
    assert_close_deg("coarse output angle",
                     result.angle_deg,
                     expected_output_angle_with_common_phase_error(142.0f, 11.0f),
                     0.05f);
    assert_close_deg("phase16 common offset", diag.phase16_error_deg, 11.0f, 0.05f);
    assert_close_deg("phase15 common offset", diag.phase15_error_deg, 11.0f, 0.05f);
}

static void test_uncalibrated_mode_holds_main_angle_but_updates_diag(void)
{
    v2_encoder_calibration_t calibration;
    v2_encoder_state_t state;
    v2_encoder_result_t result;
    v2_encoder_diag_t diag;
    uint32_t i;

    v2_encoder_calibration_set_defaults(&calibration);
    if (calibration.valid)
    {
        fprintf(stderr, "default calibration should enter rough mode\n");
        exit(1);
    }

    v2_encoder_state_init(&state);
    for (i = 0U; i < 360U; i++)
    {
        const float angle = (float)i;
        const v2_encoder_raw_sample_t sample = make_sample_from_centers(15.0f * angle,
                                                                        16.0f * angle,
                                                                        21000.0f,
                                                                        32000.0f,
                                                                        26000.0f,
                                                                        29000.0f,
                                                                        9000.0f);
        v2_encoder_process_with_diag(&state, &calibration, &sample, &result, &diag);
    }

    {
        const v2_encoder_raw_sample_t sample = make_sample_from_centers(15.0f * 203.0f,
                                                                        16.0f * 203.0f,
                                                                        21000.0f,
                                                                        32000.0f,
                                                                        26000.0f,
                                                                        29000.0f,
                                                                        9000.0f);
        v2_encoder_process_with_diag(&state, &calibration, &sample, &result, &diag);
    }

    if ((result.status & V2_ENCODER_STATUS_NOT_CALIBRATED) == 0U)
    {
        fprintf(stderr, "expected not-calibrated status, got 0x%08lX\n", (unsigned long)result.status);
        exit(1);
    }

    if ((result.status & V2_ENCODER_STATUS_HOLD_LAST) == 0U)
    {
        fprintf(stderr, "expected hold-last status, got 0x%08lX\n", (unsigned long)result.status);
        exit(1);
    }

    assert_close_deg("held uncalibrated angle", result.angle_deg, 0.0f, 0.05f);
    assert_close_deg("rough diagnostic angle", diag.angle_deg, expected_output_angle(203.0f), 0.75f);
}

int main(void)
{
    test_vernier_candidate_selection();
    test_zero_capture();
    test_fault_holds_last_angle();
    test_stats_calibration_build();
    test_calibration_uses_extrema_not_sample_density();
    test_fixed_a1_15_a2_16_mapping_is_used();
    test_diagnostic_coarse_cancels_common_phase_error();
    test_uncalibrated_mode_holds_main_angle_but_updates_diag();

    puts("app_encoder_v2 tests passed");
    return 0;
}
