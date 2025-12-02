/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */
/* MAU wrapper: mau_atan2 */
#ifndef MAU_ATAN2_H_
#define MAU_ATAN2_H_

#include "fsl_mau.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Use MAU atan to implement atan2: returns range (-pi, pi]
 * base: MAU peripheral base (e.g., MAU0)
 * y, x: input coordinates
 * res: MAU computation resolution (e.g., kMAU_RES0)
 */
float mau_atan2(MAU_Type *base, float y, float x, mau_result_t res);

/* Convenience wrapper: uses MAU0 with default resolution */
static inline float mau_atan2f(float y, float x)
{
    return mau_atan2(MAU0, y, x, kMAU_RES0);
}

#ifdef __cplusplus
}
#endif

#endif /* MAU_ATAN2_H_ */
