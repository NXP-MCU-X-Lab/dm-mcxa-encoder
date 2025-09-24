#include "nl.h"

static nl_t kp_acc = 5.0F;
static nl_t kp_mag = 5.0F;

void mahony_set_kp_gravity(nl_t kp)
{
    kp_acc = kp;
}

void mahony_set_kp_mag(nl_t kp)
{
    kp_mag = kp;
}
    
void mahony_gravity_fb(nl_t *q, nl_t *acc, nl_t *e)
{
    nl_t  acc_nz[3], Qn2b[4];
    nl_t GN[3]; /* GRAVITY in N frame */
    nl_t GB[3]; /* GRAVITY in B frame */
    
    /* gravity in N frame */
    GN[0] = 0.0;
    GN[1] = 0.0;
    GN[2] = 1.0;
    
    /* Normalise accelerometer measurement */
    vcopy(acc_nz, acc, 3);
    vnormlz(acc_nz, 3);
    /* Estimated direction of gravity and vector perpendicular to magnetic flux */
    qconj(q, Qn2b);
    qmulv(Qn2b, GN, GB);
    
    /* C = A x B */
    v3cross(acc_nz, GB, e);
    vscale2(e, kp_acc, 3);
}


void mahony_mag_fb(nl_t *q, nl_t *mag, nl_t *e)
{
    nl_t h[3], b[3], w[3], mag_normlz[3], Qn2b[4];

    vcopy(mag_normlz, mag, 3);
    v3normlz(mag_normlz);
    
    /* rotate mag(B) to H frame: H frame is ?? with N in XOY, only Z direction is different http://blog.csdn.net/qq_21842557/article/details/50993809  */
    qmulv(q, mag_normlz, h);

    /* ENU */
    b[0] = 0;
    b[1] = sqrt(POW2(h[0]) + POW2(h[1]));
    b[2] = h[2];
    
    /* Estimated direction of gravity and magnetic field */
    qconj(q, Qn2b);
    qmulv(Qn2b, b, w);
            
    v3cross(mag_normlz, w, e);
    vscale2(e, kp_mag, 3);
}



