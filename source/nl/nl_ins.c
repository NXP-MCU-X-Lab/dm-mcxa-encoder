#include "nl.h"


/**
 * @brief Initialize INS attitude using accelerometer and yaw angle
 * @details Performs coarse alignment using gravity vector and given yaw
 * 
 * @param[in,out] ins INS structure
 * @param[in] acc Accelerometer measurements in body frame (m/s^2)
 * @param[in] yaw Yaw angle in radians, range [-p, p]
 * 
 * @note Uses gravity vector to determine roll and pitch angles
 */
void ins_align(ins_t *ins, nl_t *acc, nl_t yaw)
{
    /* gravity in body frame */
    nl_t gb[3]; 

    /* gb = -fb; */
    gb[0] = -acc[0];
    gb[1] = -acc[1];
    gb[2] = -acc[2];
    v3normlz(gb);

    if(ins->coord == NL_COORD_ENU)
    {
        ins->att.pitch = asin(-gb[1]);
        ins->att.roll = atan2(gb[0], -gb[2]);
    }
    else if(ins->coord == NL_COORD_NED)
    {
        ins->att.pitch = asin(-gb[0]);
        ins->att.roll = atan2(gb[1], gb[2]);
    }
    else if(ins->coord == NL_COORD_NWU)
    {
        ins->att.pitch = asin(gb[0]);
        ins->att.roll = atan2(-gb[1], -gb[2]);
    }
    
    ins->att.yaw = yaw;
    att2q(&ins->att, ins->q);
}

/**
 * @brief Set INS yaw angle and update related quaternion
 * @details Sets the yaw angle in the INS structure and updates the corresponding quaternion.
 *          The yaw angle follows the North-to-East convention:
 *          - ENU: Counter-clockwise(CCW) from North, range [-p, p]
 *          - NED: Clockwise(CW) from North, range [-p, p]
 *          
 * @param[in,out] ins Pointer to INS structure
 * @param[in] yaw Yaw angle in radians, must be in range [-p, p]
 * 
 * 
 * @return void
 */
void ins_set_yaw(ins_t *ins, nl_t yaw)
{
    /* Set yaw angle */
    ins->att.yaw = yaw;
    
    /* Update quaternion and normalize angles */
    att2q(&ins->att, ins->q);
    q2att(ins->q, &ins->att);
}

/**
 * @brief Convert INS data to string format
 * @details Formats position, attitude and other INS parameters into human readable string
 * 
 * @param[out] buf Output buffer for formatted string
 * @param[in] ins INS structure
 * @return Length of formatted string
 */
int ins2str(char *buf, ins_t *ins)
{
    uint32_t len = 0;
    q2att(ins->q, &ins->att);
    len += sprintf((char *)buf + len, "\tLLA0:       %.8f,%.8f,%.1f, Rew:%.0fm Rns:%.0fm\r\n", ins->lla0[0] * R2D, ins->lla0[1] * R2D, ins->lla0[2], ins->Rew, ins->Rns);
    len += sprintf((char *)buf + len, "\tENU:        %.1f,%.1f,%.1f\r\n", ins->p[0], ins->p[1], ins->p[2]);
    len += sprintf((char *)buf + len, "\tATT:        P:%.2f R:%.2f Y:%.2f\r\n", ins->att.pitch * R2D, ins->att.roll * R2D, ins->att.yaw * R2D);
    return len;
}

void ins_set_coord(ins_t *ins, uint8_t coord)
{
    ins->coord = coord;
    ins->att.coord = ins->coord;
    ins->att_b2v.coord = ins->coord;
}


/**
 * @brief Initialize INS structure
 * @details Sets default values for all INS parameters:
 *          - Identity matrices for rotation
 *          - Zero vectors for position, velocity, and biases
 *          - Default LLA position (0,0,0)
 *          - Initial status as INS_ALIGNING
 * 
 * @param[out] ins INS structure to initialize
 */
void ins_init(ins_t *ins)
{
    ins_set_coord(ins, NL_COORD_ENU);
    
    ins->Cb2v = mcreate(3, 3);
    meye(ins->Cb2v);
    
    v3fill(ins->v, 0);
    
    ins->p[0] = 0;
    ins->p[1] = 0;
    ins->p[2] = 0;
    
    v3fill(ins->gb, 0);
    v3fill(ins->wb, 0);
    qidentity(ins->q);
    ins->ins_stat = INS_ALIGNING;

    double lla[3];
    lla[0] = 0;
    lla[1] = 0;
    lla[2] = 0;
    ins_set_inital_lla(ins, lla);
    ins->ins_update_flag = INS_UPDATE_ALL;
    qidentity(ins->Qb2v);
    
    q2att(ins->Qb2v, &ins->att_b2v);
    q2att(ins->q, &ins->att);
}



/**
 * @brief Set initial latitude, longitude and altitude
 * @details Updates initial LLA position and computes Earth radius parameters
 * 
 * @param[in,out] ins INS structure
 * @param[in] lla Initial position array [lat, lon, alt] in radians and meters
 */
void ins_set_inital_lla(ins_t *ins, double *lla)
{
    ins->lla0[0] = lla[0];
    ins->lla0[1] = lla[1];
    ins->lla0[2] = lla[2];

    double sin_lat0 = sin(ins->lla0[0]);
    double cos_lat0 = cos(ins->lla0[0]);

    ins->Rew = (RE_WGS84 * (1 + FE_WGS84 * sin_lat0 * sin_lat0) + ins->lla0[2]) * cos_lat0;
    ins->Rns = RE_WGS84 * (1 - 2 * FE_WGS84 + 3 * FE_WGS84 * sin_lat0 * sin_lat0) + ins->lla0[2];
}

/**
 * @brief Update INS states
 * @details Performs strapdown inertial navigation computation:
 *          1. Attitude update using gyro measurements
 *          2. Velocity update using accelerometer measurements
 *          3. Position update using velocity
 * 
 * @param[in,out] ins INS structure
 * @param[in] gyro Gyroscope measurements (rad/s)
 * @param[in] acc Accelerometer measurements (m/s^2)
 * @param[in] dt Time step in seconds
 * 
 * @note Update flags in ins->ins_update_flag control which states are updated
 */
void ins_update(ins_t *ins, nl_t *gyro, nl_t *acc, nl_t dt)
{
    // Rotation Vector - rv
    nl_t rv[3];
    
    v3sub(ins->gyr_comp, gyro, ins->wb);
    v3sub(ins->acc_comp, acc, ins->gb);

    /****************************Attitude Update****************************/
    if (ins->ins_update_flag & INS_UPDATE_A)
    {
        v3scale(rv, ins->gyr_comp, dt); // rv = gyro * dt
        
        // Transformation Quaternion - dq
        nl_t dq[4];
        rv2q(rv, dq);

        // Quaternion Update
        qmul2(ins->q, dq); // q = q x dq
        qnormlz(ins->q);
    }

    /****************************Velocity Update****************************/

    /* acc */
    qmulv(ins->q, ins->acc_comp, ins->f); // f_b -> f_n

    if (ins->ins_update_flag & INS_UPDATE_V)
    {
        ins->a[0] = ins->f[0]; // a_n = f_n - g_n
        ins->a[1] = ins->f[1];
        ins->a[2] = ins->f[2] - GRAVITY;
        
        nl_t dv_n[3];
        v3scale(dv_n, ins->a, dt); // dv_n = a_n * dt;
        v3add2(ins->v, dv_n);      // v = v + dv_n
        if(v3norm(ins->v) > 0.2) ins->TrackAngle = atan2(ins->v[0], ins->v[1]);
        if (ins->TrackAngle < 0) {
            ins->TrackAngle += 2 * M_PI;
        }
    }

    /****************************Position Update****************************/
    if (ins->ins_update_flag & INS_UPDATE_P)
    {
        nl_t dp_n[3];
        v3scale(dp_n, ins->v, dt);      // dp_n = v * dt
        
        ins->p[0] += dp_n[0];           // p = p + dp_n
        ins->p[1] += dp_n[1];
        ins->p[2] += dp_n[2];
        
        /* sync lla */
        enu2lla(ins, ins->p, ins->lla);
    }
    
    q2att(ins->q, &ins->att);
}

