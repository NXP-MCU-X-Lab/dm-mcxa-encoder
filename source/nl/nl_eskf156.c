#include "nl.h"
#include <rtdbg.h>
/*
ESKF156 LC GNSS-INS INTEGRATION:
STATE: 17: PHI(3) - Attitude error   
         VEL(3) - Velocity error
         POS(3) - Position error
         GYRB(3) - Gyroscope bias
         ACCB(3) - Accelerometer bias
         install_py(2) - Installation angle error (pitch/yaw)

MEAS:   6: GNSS_POS(3) - GNSS position measurement
         GNSS_VEL(3) - GNSS velocity measurement
*/


/**
 * @brief Initialize the loosely-coupled ESKF filter
 * 
 * @param lc Pointer to ESKF structure
 * @param opt Pointer to filter options/parameters
 */
void lc_eskf_create(lc_eskf_t *lc, const eskf156_opt_t *opt)
{
    int i = 0;
    kf_state_t *s = &lc->s;
    kf_state_create(s, 18);
    
    // Define state vector indices
    lc->idx_a = 0;            // Attitude error index
    lc->idx_v = 3;            // Velocity error index
    lc->idx_p = 6;            // Position error index
    lc->idx_wb = 9;           // Gyroscope bias index
    lc->idx_gb = 12;          // Accelerometer bias index
    lc->idx_installangle = 15; // Installation angle error index
    lc->idx_timedelay = 17; //timedelay error index
    
    /* Initialize covariance matrix P0 */
    mfill(s->P, 0);
    // Attitude error initial covariance
    MELEMENT(s->P, lc->idx_a+0, lc->idx_a+0) = POW2(opt->P0_att[0]);
    MELEMENT(s->P, lc->idx_a+1, lc->idx_a+1) = POW2(opt->P0_att[1]);
    MELEMENT(s->P, lc->idx_a+2, lc->idx_a+2) = POW2(opt->P0_att[2]);
    

    // Initialize velocity, position, and bias states covariance
    for(i=0; i<3; i++)
    {
        MELEMENT(s->P, lc->idx_v+ i, lc->idx_v+ i) = POW2(opt->P0_vel);
        MELEMENT(s->P, lc->idx_p+ i, lc->idx_p+ i) = POW2(opt->P0_pos * M2DM); //Unit Convert m to dm
        MELEMENT(s->P, lc->idx_wb+i, lc->idx_wb+i) = POW2(opt->P0_gyr_bias);
        MELEMENT(s->P, lc->idx_gb+i, lc->idx_gb+i) = POW2(opt->P0_acc_bias);
    }
    
    // Installation angle error initial covariance
    MELEMENT(s->P, lc->idx_installangle + 0, lc->idx_installangle + 0) = POW2(opt->P0_installangle);
    MELEMENT(s->P, lc->idx_installangle + 1, lc->idx_installangle + 1) = POW2(opt->P0_installangle);
    // Initialize timedelay error initial covariance
    MELEMENT(s->P, lc->idx_timedelay, lc->idx_timedelay) = POW2(opt->P0_timedelay);
    
    /* Set maximum covariance constraints */
    v3fill(&s->Pmax[lc->idx_a], POW2(opt->Pmax_att));
    v3fill(&s->Pmax[lc->idx_v], POW2(opt->Pmax_vel));
    v3fill(&s->Pmax[lc->idx_p], POW2(opt->Pmax_pos));
    v3fill(&s->Pmax[lc->idx_wb], POW2(opt->Pmax_gyr_bias));
    v3fill(&s->Pmax[lc->idx_gb], POW2(opt->Pmax_acc_bias));
    
    /* Set minimum covariance constraints */
    v3fill(&s->Pmin[lc->idx_a], POW2(opt->Pmin_att));
    v3fill(&s->Pmin[lc->idx_v], POW2(opt->Pmin_vel));
    v3fill(&s->Pmin[lc->idx_p], POW2(opt->Pmin_pos));
    v3fill(&s->Pmin[lc->idx_wb], POW2(opt->Pmin_gyr_bias));
    v3fill(&s->Pmin[lc->idx_gb], POW2(opt->Pmin_acc_bias));
    
    /* Initialize process noise matrix Q */
    vfill(s->Q, 0, s->P->r);
    v3fill(&s->Q[lc->idx_a], POW2(opt->Qgyr_wb));   // Attitude process noise
    v3fill(&s->Q[lc->idx_v], POW2(opt->Qacc_wb));   // Velocity process noise
    v3fill(&s->Q[lc->idx_p], 0);                    // Position has no direct process noise

    v3fill(&s->Q[lc->idx_wb], POW2(opt->Qgyr_wbb)); // Gyro bias random walk noise
    v3fill(&s->Q[lc->idx_gb], POW2(opt->Qacc_wbb)); // Accel bias random walk noise
    s->Q[lc->idx_installangle+0] = POW2(opt->Q_installangle);     // Installation pitch angle noise
    s->Q[lc->idx_installangle+1] = POW2(opt->Q_installangle);  // Installation yaw angle noise (typically less stable)
    s->Q[lc->idx_timedelay] = POW2(opt->Q_timedelay);  // timedelay noise

}

/**
 * @brief Create a generic measurement model for ESKF
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param m Number of measurement elements
 * @param default_noise_r Default measurement noise value
 */
static void lc_eskf_create_meas(kf_meas_t *meas, lc_eskf_t *lc, uint8_t m, nl_t default_noise_r)
{
    kf_meas_create(meas, &lc->s, m);
    mfill(meas->H, 0);  // Initialize measurement matrix to zeros
    vfill(meas->R, POW2(default_noise_r), meas->H->r);  // Set measurement noise
}

/**
 * @brief Create position measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_pos(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 3, default_noise_r);
    
    // Set H matrix for position measurement (direct observation of position states)
    for (int i = 0; i < meas->Z->r; i++)
    {
        MELEMENT(meas->H, i, i + lc->idx_p) = 1;
    }
}

/**
 * @brief Create velocity measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_vel(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 3, default_noise_r);
    
    // Set H matrix for velocity measurement (direct observation of velocity states)
    for (int i = 0; i < meas->Z->r; i++)
    {
        MELEMENT(meas->H, i, i + lc->idx_v) = 1;
    }
}

/**
 * @brief Create dual GNSS antenna measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_dual_gnss(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 3, default_noise_r);
    /* H matrix will be set later based on specific dual antenna configuration */
}

/**
 * @brief Create dual GNSS heading measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_dual_gnss_heading(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 1, default_noise_r);
    // Set H matrix for heading measurement (direct observation of yaw angle)
    for (int i = 0; i < meas->Z->r; i++)
    {
        MELEMENT(meas->H, i, i + 2) = 1;  // Observe yaw component of attitude
    }
}

/**
 * @brief Create Zero velocity Update (ZUPT) measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_zupt(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas_vel(meas, lc, default_noise_r);
    // Uses velocity measurement model with zero reference
}

/**
 * @brief Create Zero Angular Rate Update (ZARU) measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_zaru(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 3, default_noise_r);
    
    for (int i = 0; i < meas->Z->r; i++)
    {
        MELEMENT(meas->H, i, i + lc->idx_wb) = 1;
    }
    // Additional implementation details would follow here
}

/**
 * @brief Create Zero Integrated Heading Rate (ZIHR) measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_zihr(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 1, default_noise_r);
    // Set H matrix for heading measurement (direct observation of yaw angle)
    for (int i = 0; i < meas->Z->r; i++)
    {
        MELEMENT(meas->H, i, i + 2) = 1;  // Observe yaw component of attitude
    }
}

/**
 * @brief Create magnetometer measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_mag(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 3, default_noise_r);
    
    // Magnetometer measurement model will be configured during update
    // Typically provides heading information based on Earth's magnetic field
}

void lc_eskf_create_meas_gravity(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 2, default_noise_r);
    
    MELEMENT(meas->H, lc->idx_a+0, 1) = 1;
    MELEMENT(meas->H, lc->idx_a+1, 0) = -1;
}

/**
 * @brief Create Non-Holonomic Constraint (NHC) measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_nhc(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 2, default_noise_r);
    
    // NHC measurement model will be configured during update based on vehicle orientation
    // This typically constrains lateral and vertical velocity in body frame
}

/**
 * @brief Create odometer measurement model
 * 
 * @param meas Pointer to measurement structure
 * @param lc Pointer to ESKF structure
 * @param default_noise_r Default measurement noise value
 */
void lc_eskf_create_meas_odo(kf_meas_t *meas, lc_eskf_t *lc, nl_t default_noise_r)
{
    lc_eskf_create_meas(meas, lc, 1, default_noise_r);
    
    // Odometer measurement model will be configured during update
    // Typically measures forward velocity in body frame
}

void lc_eskf_get_std_att(lc_eskf_t *lc, nl_t *std) 
{
    kf_get_state_std(&lc->s, lc->idx_a, 3, std);
}

void lc_eskf_get_std_wb(lc_eskf_t *lc, nl_t *std) 
{
    kf_get_state_std(&lc->s, lc->idx_wb, 3, std);
}

void lc_eskf_get_std_v(lc_eskf_t *lc, nl_t *std) 
{
    kf_get_state_std(&lc->s, lc->idx_v, 3, std);
}

void lc_eskf_get_std_p(lc_eskf_t *lc, nl_t *std) 
{
    kf_get_state_std(&lc->s, lc->idx_p, 3, std);
}

void lc_eskf_get_std_gb(lc_eskf_t *lc, nl_t *std) 
{
    kf_get_state_std(&lc->s, lc->idx_gb, 3, std);
}

nl_t lc_eskf_get_std_install_angle_yaw(lc_eskf_t *lc) 
{
    nl_t std;
    kf_get_state_std(&lc->s, lc->idx_installangle + 1, 1, &std);
    return std;
}

/* generate F */
void lc_eskf15_state_model(kf_state_t *s, ins_t *ins, nl_t dt)
{
    // f_n
    m_t Fn;
    nl_t Fn_data[9];
    minit(&Fn, 3, 3, Fn_data);
    nl_skew(&Fn, ins->f);
    mscale2(&Fn, dt);

    mbcopy(s->F, &Fn, 3, 0);

    // bCn
    nl_t c[9], nc[9];
    m_t C, NC;
    minit(&C, 3, 3, c);
    minit(&NC, 3, 3, nc);
    q2dcm(ins->q, c);
    mscale2(&C, dt);
    mscale(&C, &NC, -1);

    // Matrix bias->phi, dv
    mbcopy(s->F, &NC, 0, 9);
    mbcopy(s->F, &C, 3, 12);

    // Matrix dv->dp
    for (int i = 6; i < 9; i++)
    {
        MELEMENT(s->F, i, i - 3) = dt * M2DM;// Convert m to dm
    }
}

/**
 * lc_eskf_feedback - Apply ESKF state feedback to update INS states
 * @lc: Pointer to ESKF filter structure
 * @ins: INS system state to be updated
 * @opt: Feedback option mask, specifies states to feedback (see ESKF_FB_XXX definitions)
 * @alpha_base: Bias feedback coefficient(0~1), 1 means full feedback, <1 means partial feedback
 *
 * This function applies the estimated error states from ESKF to correct the INS states.
 * For attitude, velocity and position, full feedback is applied.
 * For gyro and accelerometer biases, partial feedback can be applied based on alpha_base.
 */
void lc_eskf_feedback(lc_eskf_t *lc, ins_t *ins, uint32_t opt, nl_t alpha_base)
{
    int i;
    int an = 0, vn = 0, pn = 0, wn = 0, gn = 0, bcv = 0, time = 0;
    kf_state_t *s = &lc->s;
    nl_t rv[3] = {0};
    nl_t qe[4];
    
    nl_t alpha_bias = alpha_base;
    
    if (opt & ESKF_FB_A_XY)
        an = 2;
    if (opt & ESKF_FB_A_XYZ)
        an = 3;
    if (opt & ESKF_FB_V_XYZ)
        vn = 3;
    if (opt & ESKF_FB_P_XYZ) 
        pn = 3;
    if (opt & ESKF_FB_W_XY)
        wn = 2;
    if (opt & ESKF_FB_W_XYZ)
        wn = 3;
    if (opt & ESKF_FB_G_XYZ)
        gn = 3;
    if (opt & ESKF_FB_INSTALL_CBV)
        bcv = 2; // p,y install esti
    if (opt & ESKF_FB_TIMEDELAY)
        time = 1; // time

    if (an)
    {
        vcopy(rv, M2V(s->X), an);

        /* Error Quaternion - dq */
        rv2q(rv, qe);

        /* Quaternion Update */
        qmul3(qe, ins->q); /* q = qe x q */

        /* State reset */
        for (i = 0; i < an; i++)
        {
            MELEMENT(s->X, i + lc->idx_a, 0) = 0;
        }
    }

    if (vn)
    {
        for (i = 0; i < vn; i++)
        {
            ins->v[i] -= MELEMENT(s->X, i + lc->idx_v, 0);
            MELEMENT(s->X, i + lc->idx_v, 0) = 0;
        }
    }

    if (pn)
    {
        for (i = 0; i < pn; i++)
        {
            ins->p[i] -= 0.01 * (MELEMENT(s->X, i + lc->idx_p, 0) / M2DM); // feedback Unit conversion: decimeter to meter
            MELEMENT(s->X, i + lc->idx_p, 0) = 0.99 * MELEMENT(s->X, i + lc->idx_p, 0);
        }
    }

    if (wn)
    {
        for (i = 0; i < wn; i++)
        {
            ins->wb[i] += MELEMENT(s->X, i + lc->idx_wb, 0) * alpha_bias;
            MELEMENT(s->X, i + lc->idx_wb, 0) = 0;
            //  MELEMENT(s->X, i + lc->idx_wb, 0) *= (1.0 - alpha_bias); /* FIXME: it's correct, but not good */
        }
    }

    if (gn)
    {
        for (i = 0; i < gn; i++)
        {
            ins->gb[i] += MELEMENT(s->X, i + lc->idx_gb, 0) * alpha_bias;
            MELEMENT(s->X, i + lc->idx_gb, 0) = 0;
            // MELEMENT(s->X, i + lc->idx_gb, 0) *= (1.0 - alpha_bias); /* FIXME: it's correct, but not good */
        }
    }
    
    if (bcv)
    {
        nl_t X_temp[3];
        X_temp[0] = MELEMENT(s->X, lc->idx_installangle + 0, 0);
        X_temp[1] = 0;
        X_temp[2] = MELEMENT(s->X, lc->idx_installangle + 1, 0);
			
        vcopy(rv, X_temp, 3);

        // Error Quaternion - dq
        rv2q(rv, qe);

        // Quaternion Update
        qmul3(qe, ins->Qb2v); // q = qe x q

        q2dcm(ins->Qb2v, ins->Cb2v->dat);

        q2att(ins->Qb2v, &ins->att_b2v);

        MELEMENT(s->X, lc->idx_installangle + 0, 0) = 0;
        MELEMENT(s->X, lc->idx_installangle + 1, 0) = 0;
    }
    LOG_D("afterfdb_install:pitch = %f,roll = %f,yaw = %f",ins->att_v2n.pitch * D2R, ins->att_b2v.roll * D2R, ins->att_b2v.yaw * D2R);
    if (time)
    {
        ins->timedelay += MELEMENT(s->X, lc->idx_timedelay + 0, 0);
        MELEMENT(s->X, lc->idx_timedelay + 0, 0) = 0;
    }

}

void lc_eskf_meas_set_pos(kf_meas_t *m, ins_t *ins, nl_t *pos_enu, nl_t gnss_delay, nl_t *lever)
{
    nl_t lever_n[3];
    qmulv(ins->q, lever, lever_n);
    for (int i = 0; i < m->Z->r; i++)
    {
        MELEMENT(m->Z, i, 0) = (ins->p[i] - pos_enu[i] - ins->v[i] * gnss_delay + lever_n[i]) * M2DM; // Unit Convert m to dm
        MELEMENT(m->H, i, 17) = ins->v[i];
    }
}

void lc_eskf_meas_set_vel(kf_meas_t *m, ins_t *ins, nl_t *gyr, nl_t *vel_enu, nl_t gnss_delay, nl_t *lever)
{
    nl_t skew_w_b[9], wb_x_lever[3], Cb2n_wb_x_lever[3]; 
    m_t Skew;
    minit(&Skew, 3, 3, skew_w_b);
    nl_skew(&Skew, gyr);
    mvmul(&Skew, lever, wb_x_lever);
    qmulv(ins->q, wb_x_lever, Cb2n_wb_x_lever);
    for (int i = 0; i < m->Z->r; i++)
    {
        MELEMENT(m->Z, i, 0) = ins->v[i] - vel_enu[i] - ins->a[i] * gnss_delay + Cb2n_wb_x_lever[i];
        MELEMENT(m->H, i, 17) = ins->f[i];
    }
}

/*
  [I] RposENU: std of pos
*/
void lc_eskf_meas_set_pos_std(kf_meas_t *m, nl_t *RposENU)
{
    // Unit Convert m to dm
    m->R[0] = POW2(RposENU[0] * M2DM);
    m->R[1] = POW2(RposENU[1] * M2DM);
    m->R[2] = POW2(RposENU[2] * M2DM);
}

void lc_eskf_meas_set_vel_std(kf_meas_t *m, nl_t *RvelENU)
{
    m->R[0] = POW2(RvelENU[0]);
    m->R[1] = POW2(RvelENU[1]);
    m->R[2] = POW2(RvelENU[2]);
}

void lc_eskf_meas_set_zaru_std(kf_meas_t *m, nl_t R)
{
    m->R[0] = POW2(R);
    m->R[1] = POW2(R);
    m->R[2] = POW2(R);
}

void lc_eskf_meas_set_dualgnss_h(kf_meas_t *m, nl_t *vec_n)
{
    mfill(m->H, 0);

    nl_t bl_n[3];
    v3copy(bl_n, vec_n);

    m_t BL;
    nl_t bl_akew[9];
    minit(&BL, 3, 3, bl_akew);
    nl_skew(&BL, bl_n);

    MELEMENT(m->H, 0, 2) = MELEMENT((&BL), 0, 2);
    MELEMENT(m->H, 1, 2) = MELEMENT((&BL), 1, 2);
    MELEMENT(m->H, 2, 2) = MELEMENT((&BL), 2, 2);
    //mbcopy(m->H, &BL, 0, 0);
}

/**
 * @brief Set up dual GNSS antenna measurement for ESKF update
 *
 * This function prepares the measurement model for a dual GNSS antenna system
 * to be used in an Error-State Kalman Filter (ESKF) update. It calculates the
 * measurement residual between the expected baseline vector (derived from the
 * current navigation solution) and the measured baseline vector from the GNSS.
 *
 * @param m Pointer to the measurement structure (kf_meas_t)
 * @param ins Pointer to the INS (Inertial Navigation System) structure
 * @param bl_vec Pointer to the measured baseline vector in N-frame (from GNSS)
 * @param antA Pointer to the position of antenna A in V-frame
 * @param antB Pointer to the position of antenna B in V-frame
 *
 * @note The function performs the following steps:
 *       1. Calculates the baseline vector in V-frame (Vehicle frame)
 *       2. Converts the baseline vector from V-frame to B-frame (IMU frame)
 *       3. Converts the baseline vector from B-frame to N-frame (Navigation frame)
 *       4. Processes the GNSS measurement (already in N-frame)
 *       5. Calculates the measurement residual
 *
 * @warning This function assumes that antA and antB are provided in the V-frame,
 *          and that bl_vec is provided in the N-frame. Ensure coordinate frames
 *          are consistent when calling this function.
 */
void lc_eskf_meas_set_dualgnss(kf_meas_t *m, ins_t *ins, nl_t *bl_vec, nl_t *antA, nl_t *antB)
{
    nl_t set_bl_v[3], set_bl_n[3], mes_bl_n[3], set_bl_b[3], Qv2b[4];
    
    // Calculate the baseline vector in V-frame (Vehicle frame)
    vsub(set_bl_v, antB, antA, 3);
    vnormlz(set_bl_v, 3);
    
    // Convert the baseline vector from V-frame to B-frame (IMU frame)
    qconj(ins->Qb2v, Qv2b);  // Get the quaternion from V-frame to B-frame
    qmulv(Qv2b, set_bl_v, set_bl_b);

    // Convert the baseline vector from B-frame to N-frame (Navigation frame)
    qmulv(ins->q, set_bl_b, set_bl_n);

    // Process the measurement (GNSS-provided baseline vector, already in N-frame)
    vcopy(mes_bl_n, bl_vec, 3);
    vnormlz(mes_bl_n, 3);

    // Calculate the measurement residual
    vsub(M2V(m->Z), set_bl_n, mes_bl_n, 3);
}

void lc_eskf_meas_set_dualgnss_heading(kf_meas_t *m, ins_t *ins,nl_t heading_dual, nl_t heading_v2dual, nl_t ante_heading, nl_t gnss_delay)
{
    //convert dual heading to b heading ; rad
    nl_t Z_yaw = heading_dual - heading_v2dual - ante_heading - ins->att_b2v.yaw;
    
    MELEMENT(m->Z, 0, 0) = angle_normalize_pi(( - Z_yaw) - ins->att.yaw);
}

void lc_eskf_meas_set_gravity(kf_meas_t *m, ins_t *ins, nl_t *acc)
{
    nl_t f_b[3];
    vcopy(f_b, acc, 3);
    vnormlz(f_b, 3);

    nl_t f_n[3], g_n[3];
    qmulv(ins->q, f_b, f_n);
    
    if(ins->coord == NL_COORD_ENU)
    {
        vscale(g_n, f_n, -1, 3);
    }
    else if(ins->coord == NL_COORD_NED)
    {
        vscale(g_n, f_n, 1, 3);
    }
    else if(ins->coord == NL_COORD_NWU)
    {
        vscale(g_n, f_n, -1, 3);
    }

    MELEMENT(m->Z, 0, 0) = g_n[0];
    MELEMENT(m->Z, 1, 0) = g_n[1];
}

void lc_eskf_meas_set_mag(kf_meas_t *m, ins_t *ins, nl_t *mag)
{
    nl_t normlz[3], h[3], b[3], n;
    
    n = v3norm(mag);
    if(n < 0.01) return;
    
    v3scale(normlz, mag, 1.0 / n);
    
    /* Rotate magnetometer measurement from body frame to navigation frame */
    qmulv(ins->q, normlz, h);
    
    if(ins->coord == NL_COORD_ENU)
    {
        /* In ENU frame:
           Reference magnetic field lies in the north-east horizontal plane
           b[0] = 0;         // East component
           b[1] = sqrt(...); // North component (horizontal magnitude)
           b[2] = h[2];      // Up component (remains unchanged) */
        b[0] = 0;                               
        b[1] = sqrt(POW2(h[0]) + POW2(h[1]));   
        b[2] = h[2];                            
    }
    else if(ins->coord == NL_COORD_NED)
    {
        /* In NED frame:
           Reference magnetic field lies in the north-east horizontal plane
           b[0] = sqrt(...); // North component (horizontal magnitude)
           b[1] = 0;         // East component
           b[2] = h[2];      // Down component (remains unchanged) */
        b[0] = sqrt(POW2(h[0]) + POW2(h[1]));   
        b[1] = 0;                               
        b[2] = h[2];                            
    }
    else if(ins->coord == NL_COORD_NWU)
    {
        /* In NWU frame:
           Reference magnetic field lies in the north-west horizontal plane
           b[0] = sqrt(...); // North component (horizontal magnitude)
           b[1] = 0;         // West component
           b[2] = h[2];      // Up component (remains unchanged) */
        b[0] = sqrt(POW2(h[0]) + POW2(h[1]));   
        b[1] = 0;                               
        b[2] = h[2];                            
    }

    
    v3sub2(h, b);
    
    MELEMENT(m->Z, 0, 0) = h[0];
    MELEMENT(m->Z, 1, 0) = h[1];
    MELEMENT(m->Z, 2, 0) = h[2];
    
    /* Calculate observation matrix using cross product */
    nl_skew(m->H, b);
    
    /* Clear roll and pitch columns, only keep yaw update */
    MELEMENT(m->H, 0, 0) = 0;
    MELEMENT(m->H, 1, 0) = 0;
    MELEMENT(m->H, 2, 0) = 0;
    MELEMENT(m->H, 0, 1) = 0;
    MELEMENT(m->H, 1, 1) = 0;
    MELEMENT(m->H, 2, 1) = 0;
}

void lc_eskf_meas_set_zaru(kf_meas_t *m, ins_t *ins, nl_t *wb)
{
    for (int i = 0; i < m->Z->r; i++)
    {
        MELEMENT(m->Z, i, 0) = wb[i] - ins->wb[i]; 
    }
}

void lc_eskf_meas_set_zihr(kf_meas_t *m, ins_t *ins, nl_t yaw_zihr)
{
    for (int i = 0; i < m->Z->r; i++)
    {
        MELEMENT(m->Z, i, 0) = angle_normalize_pi(yaw_zihr - ins->att.yaw); 
    }
}

void lc_eskf_meas_set_zupt(kf_meas_t *m, ins_t *ins)
{
    for (int i = 0; i < m->Z->r; i++)
    {
        MELEMENT(m->Z, i, 0) = ins->v[i] - 0;
    }
}



// void lc_eskf_meas_set_nhc(kf_meas_t *m, ins_t *ins)
//{
//     nl_t Qbn[4], Vb[3], Vn[3];

//    qconj(ins->q, Qbn);
//    qmulv(Qbn, ins->v, Vb);
//    Vb[0] = 0;
//    Vb[2] = 0;
//    qmulv(ins->q, Vb, Vn);

//    for (int i=0; i<3; i++)
//    {
//        MELEMENT(m->Z, i, 0) = ins->v[i] - Vn[i];
//    }
//}

/* est_install_angle: 1: est install angle(ESKF17), 0: do not est inatall angle */
void lc_eskf_meas_set_nhc(kf_meas_t *m, ins_t *ins, uint8_t est_install_angle)
{
    static nl_t Qn2b[4], Vb[3], Vv[3], M1_dat[9], Cn2v_dat[9], M3_dat[9], m_skew_vel_dat[9], Cn2b_dat[9];
    m_t Cn2v;

    /* Create Qn2b */
    qconj(ins->q, Qn2b);
    
    /* Create Vb */
    qmulv(Qn2b, ins->v, Vb);
    
    /* create Cn2b */
    m_t Cn2b;
    minit(&Cn2b, 3, 3, Cn2b_dat);
    q2dcm(Qn2b, Cn2b_dat);	
    
    /* create Cn2v */
    minit(&Cn2v, 3, 3, Cn2v_dat);
    mmul(ins->Cb2v, &Cn2b, &Cn2v);
    
    mvmul(ins->Cb2v, Vb, Vv);
    
    if(est_install_angle)
    {
        m_t M1, M3, m_skew_vel;

        minit(&m_skew_vel, 3, 3, m_skew_vel_dat);

        minit(&M3, 3, 3, M3_dat);	
        minit(&M1, 3, 3, M1_dat);	
        
        nl_skew(&m_skew_vel, ins->v);
        mmul(&Cn2v, &m_skew_vel, &M1);	 
        mscale2(&M1, -1);
          

        nl_skew(&M3, Vv);
        
        /* set Z */
        MELEMENT(m->Z, 0, 0) = Vv[0];
        MELEMENT(m->Z, 1, 0) = Vv[2];
        
        /* set H */
        MELEMENT(m->H, 0, 0) = MELEMENT((&M1), 0, 0);
        MELEMENT(m->H, 0, 1) = MELEMENT((&M1), 0, 1);
        MELEMENT(m->H, 0, 2) = MELEMENT((&M1), 0, 2);
        MELEMENT(m->H, 0, 3) = MELEMENT((&Cn2v), 0, 0);
        MELEMENT(m->H, 0, 4) = MELEMENT((&Cn2v), 0, 1);
        MELEMENT(m->H, 0, 5) = MELEMENT((&Cn2v), 0, 2);
        MELEMENT(m->H, 0, 16) = MELEMENT((&M3), 0, 2);

        MELEMENT(m->H, 1, 0) = MELEMENT((&M1), 2, 0);
        MELEMENT(m->H, 1, 1) = MELEMENT((&M1), 2, 1);
        MELEMENT(m->H, 1, 2) = MELEMENT((&M1), 2, 2);
        MELEMENT(m->H, 1, 3) = MELEMENT((&Cn2v), 2, 0);
        MELEMENT(m->H, 1, 4) = MELEMENT((&Cn2v), 2, 1);
        MELEMENT(m->H, 1, 5) = MELEMENT((&Cn2v), 2, 2);
        MELEMENT(m->H, 1, 15) = MELEMENT((&M3), 2, 0);
    }
    else
    {
        /* set Z */
        MELEMENT(m->Z, 0, 0) = Vv[0];
        MELEMENT(m->Z, 1, 0) = Vv[2];

        /* set H */
        MELEMENT(m->H, 0, 3) = MELEMENT((&Cn2v), 0, 0);
        MELEMENT(m->H, 0, 4) = MELEMENT((&Cn2v), 0, 1);
        MELEMENT(m->H, 0, 5) = MELEMENT((&Cn2v), 0, 2);

        MELEMENT(m->H, 1, 3) = MELEMENT((&Cn2v), 2, 0);
        MELEMENT(m->H, 1, 4) = MELEMENT((&Cn2v), 2, 1);
        MELEMENT(m->H, 1, 5) = MELEMENT((&Cn2v), 2, 2);
    }
}

void lc_eskf_meas_set_od(kf_meas_t *m, ins_t *ins, nl_t od_vel, nl_t od_scale_factor)
{
    nl_t Qbn[4], Qvn[4], Vb[3], Vv[3];

    qconj(ins->q, Qbn);
    qmulv(Qbn, ins->v, Vb);/* n-frame to b-frame  */
	qmulv(ins->Qb2v, Vb, Vv);/* b-frame to b-frame*/
    /* set Z */
    MELEMENT(m->Z, 0, 0) = Vv[1] - od_vel * od_scale_factor;

    nl_t c[9];
    m_t Cvn;
    minit(&Cvn, 3, 3, c);
	qmul(ins->Qb2v, Qbn, Qvn);
    q2dcm(Qvn, c);
	
    /* set H */
    MELEMENT(m->H, 0, 3) = MELEMENT((&Cvn), 1, 0);
    MELEMENT(m->H, 0, 4) = MELEMENT((&Cvn), 1, 1);
    MELEMENT(m->H, 0, 5) = MELEMENT((&Cvn), 1, 2);
}
