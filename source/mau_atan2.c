/* MAU 封装：mau_atan2 */
#include "fsl_mau.h"
#include <math.h>

#include "mau_atan2.h"

float mau_atan2(MAU_Type *base, float y, float x, mau_result_t res)
{
    /* 处理 x 为 0 的情形，避免除零 */
    if (x == 0.0f) {
        if (y > 0.0f) return MAU_MATH_PI / 2.0f;
        if (y < 0.0f) return -MAU_MATH_PI / 2.0f;
        /* x == 0, y == 0：约定返回 0 */
        return 0.0f;
    }

    float ratio = y / x;
    /* MAU_AtanXDivPIFloat 返回 atan(ratio)/pi，乘回 pi 得到弧度 */
    float a = MAU_AtanXDivPIFloat(base, ratio, res) * MAU_MATH_PI;

    /* 象限修正：将 atan 结果扩展为 atan2 */
    if (x > 0.0f) {
        return a;
    }
    if (y >= 0.0f) {
        return a + MAU_MATH_PI;
    }
    return a - MAU_MATH_PI;
}