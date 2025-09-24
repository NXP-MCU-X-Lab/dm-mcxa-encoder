#include "nl.h"

/**
 * @brief Internal quaternion multiplication helper
 * @param[in] q1 First quaternion
 * @param[in] q2 Second quaternion
 * @param[out] r Result quaternion
 */
static void _qmul_core(const nl_t *q1, const nl_t *q2, nl_t *r)
{
    r[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    r[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    r[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
    r[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1];
}

/**
 * @brief Quaternion multiplication: r = q1 x q2
 * @param[in] q1 First quaternion
 * @param[in] q2 Second quaternion
 * @param[out] r Result quaternion
 */
void qmul(const nl_t *q1, const nl_t *q2, nl_t *r)
{
    _qmul_core(q1, q2, r);
}

/**
 * @brief In-place quaternion multiplication: q1 = q1 x q2
 * @param[in,out] q1 First quaternion and result
 * @param[in] q2 Second quaternion
 */
void qmul2(nl_t *q1, const nl_t *q2)
{
    nl_t r[4];
    _qmul_core(q1, q2, r);
    
    q1[0] = r[0];
    q1[1] = r[1];
    q1[2] = r[2];
    q1[3] = r[3];
}

/**
 * @brief In-place quaternion multiplication: q2 = q1 x q2
 * @param[in] q1 First quaternion
 * @param[in,out] q2 Second quaternion and result
 */
// q2 = q1 x q2
void qmul3(const nl_t *q1, nl_t *q2)
{
    nl_t r[4]={0};
    
    r[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    r[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    r[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
    r[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1];
    
    nl_t norm = sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2] + r[3]*r[3]);
    q2[0] = r[0] / norm;
    q2[1] = r[1] / norm;
    q2[2] = r[2] / norm;
    q2[3] = r[3] / norm;
}


/**
 * @brief Normalize quaternion if necessary
 * @param[in,out] q Quaternion to normalize
 */
void qnormlz(nl_t *q)
{
    const nl_t norm2 = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    
    // Skip normalization if already close to unit length
    if (norm2 > 1.002f || norm2 < 0.998f) {
        // Fast inverse square root approximation
        const nl_t inv_norm = 1.0f / sqrtf(norm2);
        
        q[0] *= inv_norm;
        q[1] *= inv_norm;
        q[2] *= inv_norm;
        q[3] *= inv_norm;
    }
}

/**
 * @brief Convert rotation vector to quaternion
 * @details Converts axis-angle representation to quaternion:
 *          q = [cos(?/2), sin(?/2)*axis]
 *          Uses small angle approximation for better numerical stability
 * @param[in] rv Rotation vector (axis ?angle)
 * @param[out] q Output quaternion [w,x,y,z]
 */
void rv2q(nl_t *rv, nl_t *q)
{
    const nl_t rv_nm2 = rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2];

    if (rv_nm2 < 1.0e-4)
    {
        // Small angle approximation - more efficient polynomial
        const nl_t rv_nm4 = rv_nm2 * rv_nm2;
        
        // cos(|rv|/2) ?1 - n?8 + n4/384
        q[0] = 1.0f - rv_nm2 * (0.125f - rv_nm4 * (1.0f/384.0f));
        
        // sin(|rv|/2)/|rv| ?1/2 - n?48 + n4/3840  
        const nl_t s = 0.5f - rv_nm2 * (1.0f/48.0f - rv_nm4 * (1.0f/3840.0f));
        
        q[1] = s * rv[0];
        q[2] = s * rv[1]; 
        q[3] = s * rv[2];
    }
    else
    {
        // Use ARM math library optimized functions
        const nl_t rv_nm = sqrtf(rv_nm2);
        const nl_t half_angle = rv_nm * 0.5f;
        
        // ARM CMSIS-DSP has optimized sin/cos
        q[0] = cos(half_angle);
        const nl_t s = sin(half_angle) / rv_nm;
        
        q[1] = s * rv[0];
        q[2] = s * rv[1];
        q[3] = s * rv[2];
    }
}

// Calculate the conjugate of a quaternion.
void qconj(nl_t *q, nl_t *qj)
{
    qj[0] =  q[0];
    qj[1] = -q[1];
    qj[2] = -q[2];
    qj[3] = -q[3];
}

void qconj2(nl_t *q)
{
    q[1] = -q[1];
    q[2] = -q[2];
    q[3] = -q[3];
}

void qidentity(nl_t *q)
{
    q[0] = 1;
    q[1] = 0;
    q[2] = 0;
    q[3] = 0;
}

/**
 * @brief Rotate a vector by a unit quaternion
 * @param[in] q Unit quaternion [w,x,y,z]
 * @param[in] v Input vector [x,y,z]
 * @param[out] r Output rotated vector [x,y,z]
 * @note Assumes q is normalized. No checks performed for performance.
 */
void qmulv(const nl_t *q, const nl_t *v, nl_t *r)
{
//    // Pre-calculate common terms to reduce multiplications
//    const nl_t q0q0 = q[0] * q[0];
//    const nl_t q1q1 = q[1] * q[1];
//    const nl_t q2q2 = q[2] * q[2];
//    const nl_t q3q3 = q[3] * q[3];
//    
//    // These terms are used twice each
//    const nl_t q0q1 = q[0] * q[1];
//    const nl_t q0q2 = q[0] * q[2];
//    const nl_t q0q3 = q[0] * q[3];
//    const nl_t q1q2 = q[1] * q[2];
//    const nl_t q1q3 = q[1] * q[3];
//    const nl_t q2q3 = q[2] * q[3];

//    // Rotation matrix terms (optimized)
//    // R = [2(q0q0+q1q1)-1  2(q1q2-q0q3)    2(q1q3+q0q2)   ]
//    //     [2(q1q2+q0q3)    2(q0q0+q2q2)-1  2(q2q3-q0q1)   ]
//    //     [2(q1q3-q0q2)    2(q2q3+q0q1)    2(q0q0+q3q3)-1 ]

//    r[0] = ((q0q0 + q1q1 - q2q2 - q3q3) * v[0] +
//            2.0f * (q1q2 - q0q3) * v[1] +
//            2.0f * (q1q3 + q0q2) * v[2]);
//            
//    r[1] = (2.0f * (q1q2 + q0q3) * v[0] +
//            (q0q0 - q1q1 + q2q2 - q3q3) * v[1] +
//            2.0f * (q2q3 - q0q1) * v[2]);
//            
//    r[2] = (2.0f * (q1q3 - q0q2) * v[0] +
//            2.0f * (q2q3 + q0q1) * v[1] +
//            (q0q0 - q1q1 - q2q2 + q3q3) * v[2]);

    const nl_t qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const nl_t vx = v[0], vy = v[1], vz = v[2];
    
    // Compute diagonal terms of rotation matrix
    const nl_t d0 = qw*qw + qx*qx - qy*qy - qz*qz;
    const nl_t d1 = qw*qw - qx*qx + qy*qy - qz*qz;
    const nl_t d2 = qw*qw - qx*qx - qy*qy + qz*qz;
    
    // Compute off-diagonal terms (multiply by 2 inline)
    const nl_t o01 = 2.0f * (qx*qy - qw*qz);
    const nl_t o02 = 2.0f * (qx*qz + qw*qy);
    const nl_t o10 = 2.0f * (qx*qy + qw*qz);
    const nl_t o12 = 2.0f * (qy*qz - qw*qx);
    const nl_t o20 = 2.0f * (qx*qz - qw*qy);
    const nl_t o21 = 2.0f * (qy*qz + qw*qx);
    
    // Matrix-vector multiplication
    r[0] = d0*vx + o01*vy + o02*vz;
    r[1] = o10*vx + d1*vy + o12*vz;
    r[2] = o20*vx + o21*vy + d2*vz;
}

//https://mazzo.li/posts/vectorized-atan2.html
nl_t atan_scalar_approximation(nl_t x)
{
  const nl_t a1  =  0.99997726f;
  const nl_t a3  = -0.33262347f;
  const nl_t a5  =  0.19354346f;
  const nl_t a7  = -0.11643287f;
  const nl_t a9  =  0.05265332f;
  const nl_t a11 = -0.01172120f;

  nl_t x_sq = x*x;
  return
    x * (a1 + x_sq * (a3 + x_sq * (a5 + x_sq * (a7 + x_sq * (a9 + x_sq * a11)))));
}


static nl_t atan2_auto_1(nl_t y, nl_t x)
{
    // Ensure input is in [-1, +1]
    bool swap = fabs(x) < fabs(y);
    nl_t atan_input = (swap ? x : y) / (swap ? y : x);

    // Approximate atan
    nl_t res = atan_scalar_approximation(atan_input);

    // If swapped, adjust atan output
    res = swap ? (atan_input >= 0.0f ? M_PI_2 : -M_PI_2) - res : res;
    // Adjust quadrants
    if      (x >= 0.0f && y >= 0.0f) {}                     // 1st quadrant
    else if (x <  0.0f && y >= 0.0f) { res =  M_PI + res; } // 2nd quadrant
    else if (x <  0.0f && y <  0.0f) { res = -M_PI + res; } // 3rd quadrant
    else if (x >= 0.0f && y <  0.0f) {}                     // 4th quadrant

    // Store result
    return res;
}

static nl_t asin_auto_1(nl_t x)
{
    // Handle exact boundary cases first
    if(x >= 1.0f) return M_PI_2;
    if(x <= -1.0f) return -M_PI_2;
    
    // Normal case
    return atan2_auto_1(x, sqrt(1.0f - x*x));
}

/**
 * @brief Convert quaternion to Euler angles
 * @details Supports two coordinate frames:
 *          1. ENU (East-North-Up):
 *          2. NED (North-East-Down):
 * 
 * @param[in] q Quaternion array [w,x,y,z]
 * @param[out] att Euler angles structure containing:
 *                 - roll (rad)
 *                 - pitch (rad)
 *                 - yaw (rad)
 *                 - coord: coordinate sequence (ENU or NED)
 * 
 */
void q2att(nl_t *q, att_t *att)
{
    nl_t q00 = q[0] * q[0];
    nl_t q01 = q[0] * q[1];
    nl_t q02 = q[0] * q[2];
    nl_t q03 = q[0] * q[3];
    nl_t q11 = q[1] * q[1];
    nl_t q12 = q[1] * q[2];
    nl_t q13 = q[1] * q[3];
    nl_t q22 = q[2] * q[2];
    nl_t q23 = q[2] * q[3];
    nl_t q33 = q[3] * q[3];
    nl_t C11 = q00 + q11 - q22 - q33;
    nl_t C12 = 2 * (q12 - q03);
    nl_t C22 = q00 - q11 + q22 -q33;
    nl_t C13 = 2 * (q13 + q02);
    nl_t C31 = 2 * (q13 - q02);
    nl_t C32 = 2 * (q23 + q01);
    switch(att->coord)
    {
        case NL_COORD_NED:
        case NL_COORD_NWU:
            att->pitch = asin_auto_1(2 * (q02 - q13));
            if(C31 > 0.997)
            {
                att->roll  = atan2_auto_1(-C12, C22) - att->yaw;
                if (att->roll > M_PI) att->roll = att->roll - 2 * M_PI;
                if (att->roll < -M_PI) att->roll = att->roll + 2 * M_PI;
            }
            else if(C31 < -0.997)
            {
                att->roll  = atan2_auto_1(C12, C22) + att->yaw;
                if (att->roll > M_PI) att->roll = att->roll - 2 * M_PI;
                if (att->roll < -M_PI) att->roll = att->roll + 2 * M_PI;
            }
            else
            {
                att->roll  = atan2_auto_1(2 * (q23 + q01), q00 - q11 - q22 + q33);
                att->yaw   = atan2_auto_1(2 * (q12 + q03), q00 + q11 - q22 - q33);
            }
            break;
        case NL_COORD_ENU:
        default:
            att->pitch = asin_auto_1(2 * (q23 + q01));
            if(C32 > 0.997)
            {
                att->roll  = atan2_auto_1(C13, C11) - att->yaw;
                if (att->roll > M_PI) att->roll = att->roll - 2 * M_PI;
                if (att->roll < -M_PI) att->roll = att->roll + 2 * M_PI;
            }
            else if(C32 < -0.997)
            {
                att->roll  = atan2_auto_1(C13, C11) + att->yaw;
                if (att->roll > M_PI) att->roll = att->roll - 2 * M_PI;
                if (att->roll < -M_PI) att->roll = att->roll + 2 * M_PI;
            }
            else{
                att->roll  = -atan2_auto_1(2 * (q13 - q02), q00 - q11 - q22 + q33);
                att->yaw   = -atan2_auto_1(2 * (q12 - q03), q00 - q11 + q22 - q33);
             }
            break;
    }
}


/**
 * @brief Convert Euler angles to quaternion
 * @details Supports two coordinate frames:
 *          1. ENU (East-North-Up):
 *          2. NED (North-East-Down):
 * 
 * @param[in] att Euler angles structure containing:
 *                - roll (rad)
 *                - pitch (rad) 
 *                - yaw (rad)
 *                - coord: coordinate sequence (ENU or NED)
 * @param[out] q Output quaternion array [w,x,y,z]
 */
void att2q(att_t *att, nl_t *q)
{
    nl_t sp = sin(att->pitch/2);
    nl_t cp = cos(att->pitch/2);
    nl_t sr = sin(att->roll/2);
    nl_t cr = cos(att->roll/2);
    nl_t sy = sin(att->yaw/2);
    nl_t cy = cos(att->yaw/2);
    
    switch(att->coord)
    {
        case NL_COORD_NED:
            q[0] = cr*cp*cy + sr*sp*sy;
            q[1] = sr*cp*cy - cr*sp*sy;
            q[2] = cr*sp*cy + sr*cp*sy;
            q[3] = cr*cp*sy - sr*sp*cy;
            break;
        case NL_COORD_NWU:
            q[0] = cr*cp*cy + sr*sp*sy;
            q[1] = sr*cp*cy - cr*sp*sy;
            q[2] = cr*sp*cy + sr*cp*sy;
            q[3] = cr*cp*sy - sr*sp*cy;
            break;
        case NL_COORD_ENU:
        default:
            q[0] = cp*cr*cy - sp*sr*sy;
            q[1] = sp*cr*cy - cp*sr*sy;
            q[2] = cp*sr*cy + sp*cr*sy;
            q[3] = cp*cr*sy + sp*sr*cy;
            break;
    }
}


//Convert attitude quaternion to Direction Cosine Matrix.(bCn)
void q2dcm(nl_t *q, nl_t *p)
{
    nl_t (*dcm)[3] = (nl_t (*)[3])p;
    
    nl_t q00 = q[0] * q[0];
    nl_t q11 = q[1] * q[1];
    nl_t q22 = q[2] * q[2];
    nl_t q33 = q[3] * q[3];
    
    nl_t q01 = q[0] * q[1];
    nl_t q02 = q[0] * q[2];
    nl_t q03 = q[0] * q[3];
    nl_t q12 = q[1] * q[2];
    nl_t q13 = q[1] * q[3];
    nl_t q23 = q[2] * q[3];
    
    // First row
    dcm[0][0] = q00 + q11 - q22 - q33;
    dcm[0][1] = 2.0 * (q12 - q03);
    dcm[0][2] = 2.0 * (q13 + q02);
    
    // Second row
    dcm[1][0] = 2.0 * (q12 + q03);
    dcm[1][1] = q00 - q11 + q22 - q33;
    dcm[1][2] = 2.0 * (q23 - q01);
    
    // Third row
    dcm[2][0] = 2.0 * (q13 - q02);
    dcm[2][1] = 2.0 * (q23 + q01);
    dcm[2][2] = q00 - q11 - q22 + q33;
}
