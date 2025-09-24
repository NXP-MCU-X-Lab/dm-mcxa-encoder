#include "nl.h"

/*
ESKF attitude filter：
STATE: 3: PHI(3)
MEAS:  2: GRAVITY, 3 MAG

*/


void lc_eskf_att_create(lc_eskf_t *lc, const eskfatt_opt_t *opt)
{
    int i = 0;
    
    kf_state_t *s = &lc->s;
    kf_state_create(s, 6);
    
    lc->idx_a = 0;
    lc->idx_wb = 3;
    
    /* P0 */
    mfill(s->P, 0);
    MELEMENT(s->P, lc->idx_a+0, lc->idx_a+0) = POW2(opt->P0_att[0]);
    MELEMENT(s->P, lc->idx_a+1, lc->idx_a+1) = POW2(opt->P0_att[1]);
    MELEMENT(s->P, lc->idx_a+2, lc->idx_a+2) = POW2(opt->P0_att[2]);
    
    for(i=0; i<3; i++)
    {
        MELEMENT(s->P, lc->idx_wb+i, lc->idx_wb+i) = POW2(opt->P0_gyr_bias);
    }
    
    /* P0 constract */
    v3fill(&s->Pmax[lc->idx_a], POW2(opt->Pmax_att));
    v3fill(&s->Pmax[lc->idx_wb], POW2(opt->Pmax_gyr_bias));
    
    v3fill(&s->Pmin[lc->idx_a], POW2(opt->Pmin_att));
    v3fill(&s->Pmin[lc->idx_wb], POW2(opt->Pmin_gyr_bias));
    
    /* Q */
    vfill(s->Q, 0, s->P->r);
    v3fill(&s->Q[lc->idx_a], POW2(opt->Qgyr_wb));
    v3fill(&s->Q[lc->idx_wb], POW2(opt->Qgyr_wbb));
}


/* generate F */
void eskfatt_state_model(kf_state_t *s, ins_t *ins, nl_t dt)
{
    // bCn
    nl_t c[9], nc[9];
    m_t C, NC;
    minit(&C, 3, 3, c);
    minit(&NC, 3, 3, nc);
    q2dcm(ins->q, c);
    mscale2(&C, dt);
    mscale(&C, &NC, -1);

    mbcopy(s->F, &NC, 0, 3);
}


///**
// * @brief Direct nonlinear pitch correction for high pitch angles near gimbal lock
// * 
// * This function provides direct pitch correction using accelerometer data when
// * the pitch angle approaches ±90° where traditional ESKF may become unstable.
// * Uses time-gated updates to ensure smooth corrections.
// * 
// * @param ins Pointer to INS state structure
// * @param acc Pointer to accelerometer measurements [m/s²]
// */
//void direct_nonlinear_correction(ins_t *ins, nl_t *acc)
//{
//    static uint32_t last_correction_time = 0;
//    const uint32_t CORRECTION_INTERVAL_MS = 100;    // 10Hz update rate for smooth correction
//    const nl_t PITCH_THRESHOLD = 88.0 * D2R;        // Activation threshold near gimbal lock
//    const nl_t MAX_PITCH_ERROR = 1.0 * D2R;         // Maximum allowed pitch error for correction
//    const nl_t CORRECTION_GAIN = 0.00001;           // Conservative gain for stability
//    const nl_t ACC_THRESHOLD_RATIO = 0.05;
//    
//    uint32_t current_time = nl_get_sys_ms();
//    
//    // Time gating: only update every CORRECTION_INTERVAL_MS milliseconds
//    if(current_time - last_correction_time < CORRECTION_INTERVAL_MS) {
//        return;
//    }
//    
//    // Check accelerometer magnitude to detect acceleration/deceleration
//    nl_t acc_magnitude = v3norm(acc);
//    nl_t gravity_error = fabs(acc_magnitude - GRAVITY) / GRAVITY;
//    
//    // Skip correction if accelerometer magnitude deviates too much from gravity
//    // This indicates significant linear acceleration
//    if(gravity_error > ACC_THRESHOLD_RATIO) {
//        return;  // Not in quasi-static condition
//    }
//    
//    // Normalize accelerometer vector to unit vector
//    nl_t acc_norm[3];
//    vcopy(acc_norm, acc, 3);
//    vnormlz(acc_norm, 3);
//    
//    // Calculate pitch angle from normalized accelerometer Y-component
//    // Note: asin() range is [-π/2, π/2], suitable for most pitch angles
//    // For pitch > 90°, additional quadrant detection would be needed
//    nl_t acc_pitch = asin(acc_norm[1]);
//    
//    // Calculate pitch error between accelerometer and current INS estimate
//    nl_t pitch_error = acc_pitch - ins->att.pitch;
//    
//    // Apply correction only when:
//    // 1. Pitch angle is near gimbal lock (|pitch| > 88°)
//    // 2. Pitch error is reasonable (< 1°) to avoid large corrections from noise
//    if(fabs(ins->att.pitch) > PITCH_THRESHOLD && fabs(pitch_error) < MAX_PITCH_ERROR)
//    {
//        // Apply small correction to pitch angle
//        ins->att.pitch += CORRECTION_GAIN * pitch_error;
//        
//        // Update quaternion representation from corrected Euler angles
//        att2q(&ins->att, ins->q);
//        
//        // Update timestamp for next correction cycle
//        last_correction_time = current_time;
//    }
//}



