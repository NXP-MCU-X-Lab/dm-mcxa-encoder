/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */
/* MAU wrapper: mau_atan2 */
#include "fsl_mau.h"
#include <math.h>

#include "mau_atan2.h"

float mau_atan2(MAU_Type *base, float y, float x, mau_result_t res)
{
    /* Handle x == 0 cases to avoid division by zero */
    if (x == 0.0f) {
        if (y > 0.0f) return MAU_MATH_PI / 2.0f;
        if (y < 0.0f) return -MAU_MATH_PI / 2.0f;
        /* x == 0 and y == 0: return 0 by convention */
        return 0.0f;
    }

    float ratio = y / x;
    /* MAU_AtanXDivPIFloat returns atan(ratio)/pi; multiply by PI to get radians */
    float a = MAU_AtanXDivPIFloat(base, ratio, res) * MAU_MATH_PI;

    /* Quadrant correction: extend atan result to atan2 */
    if (x > 0.0f) {
        return a;
    }
    if (y >= 0.0f) {
        return a + MAU_MATH_PI;
    }
    return a - MAU_MATH_PI;
}
