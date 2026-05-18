#include "app_encoder.h"

#include <stddef.h>

#define ENCODER_BOARD_A1_CENTER_SIN (11744.0f)
#define ENCODER_BOARD_A1_CENTER_COS (22415.5f)
#define ENCODER_BOARD_A1_T00        (0.000183f)
#define ENCODER_BOARD_A1_T10        (-0.000001f)
#define ENCODER_BOARD_A1_T11        (0.000092f)
#define ENCODER_BOARD_A1_ZERO_DEG   (56.427f)

#define ENCODER_BOARD_A2_CENTER_SIN (21819.0f)
#define ENCODER_BOARD_A2_CENTER_COS (21027.0f)
#define ENCODER_BOARD_A2_T00        (0.000083f)
#define ENCODER_BOARD_A2_T10        (0.0f)
#define ENCODER_BOARD_A2_T11        (0.000082f)
#define ENCODER_BOARD_A2_ZERO_DEG   (227.642f)

static void set_track_cal(encoder_track_calibration_t *track,
                          float center_sin,
                          float center_cos,
                          float t00,
                          float t10,
                          float t11)
{
    if (track == NULL)
    {
        return;
    }

    track->center_sin = center_sin;
    track->center_cos = center_cos;
    track->t00 = t00;
    track->t10 = t10;
    track->t11 = t11;
}

void encoder_calibration_set_board_defaults(encoder_calibration_t *calibration)
{
    if (calibration == NULL)
    {
        return;
    }

    set_track_cal(&calibration->a1,
                  ENCODER_BOARD_A1_CENTER_SIN,
                  ENCODER_BOARD_A1_CENTER_COS,
                  ENCODER_BOARD_A1_T00,
                  ENCODER_BOARD_A1_T10,
                  ENCODER_BOARD_A1_T11);
    set_track_cal(&calibration->a2,
                  ENCODER_BOARD_A2_CENTER_SIN,
                  ENCODER_BOARD_A2_CENTER_COS,
                  ENCODER_BOARD_A2_T00,
                  ENCODER_BOARD_A2_T10,
                  ENCODER_BOARD_A2_T11);
    calibration->phase_a1_zero_deg = ENCODER_BOARD_A1_ZERO_DEG;
    calibration->phase_a2_zero_deg = ENCODER_BOARD_A2_ZERO_DEG;
    calibration->valid = true;
}
