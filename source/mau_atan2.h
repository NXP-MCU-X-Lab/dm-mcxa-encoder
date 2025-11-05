/* MAU 封装：mau_atan2 */
#ifndef MAU_ATAN2_H_
#define MAU_ATAN2_H_

#include "fsl_mau.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 使用 MAU 的 atan 功能实现 atan2：返回范围 (-pi, pi]
 * base: MAU 外设基址（如 MAU0）
 * y, x: 输入坐标
 * res: MAU 计算分辨率（如 kMAU_RES0）
 */
float mau_atan2(MAU_Type *base, float y, float x, mau_result_t res);

/* 便捷封装：固定使用 MAU0 与默认分辨率 */
static inline float mau_atan2f(float y, float x)
{
    return mau_atan2(MAU0, y, x, kMAU_RES0);
}

#ifdef __cplusplus
}
#endif

#endif /* MAU_ATAN2_H_ */