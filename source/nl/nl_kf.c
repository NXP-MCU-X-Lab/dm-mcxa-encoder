#include "nl.h"


void kf_state_create(kf_state_t *s, size_t n)
{
    s->X = mcreate(n, 1);
    s->Xtmp = mcreate(n, 1);
    s->P = mcreate(n, n);
    s->Ptmp = mcreate(n, n);
    s->F = mcreate(n, n);
    s->Q = vcreate(n);
    s->KH = mcreate(n, n);
    s->Pmax = vcreate(n);
    s->Pmin = vcreate(n);

    mfill(s->F, 0);
    vfill(s->Pmax, 999999, n);
    vfill(s->Pmin, 0, n);
    maddeye(s->F);
}

/* reset all P diag value to max, and clear all non-diag val */
void lc_eskf_reset_p(kf_state_t *s)
{
    mfill(s->P, 0);
    madddiag(s->P, s->Pmax);
}

/* m: num of meas */
void kf_meas_create(kf_meas_t *meas, kf_state_t *s, size_t m)
{
    uint8_t n = s->P->r;
    
    meas->Z = mcreate(m, 1);
    meas->H = mcreate(m, n);
    meas->R = vcreate(m);
    meas->S = mcreate(m, m);
    meas->K = mcreate(n, m);
    meas->PHt = mcreate(n, m);
    meas->chi_tmp = mcreate(m, 1);
    meas->innov = mcreate(m, 1);
    meas->lambda = mcreate(1, 1);
    meas->kf_state = s;
}

void kf_set_meas_r(kf_meas_t *meas, nl_t R_std)
{
    vfill(meas->R, POW2(R_std), meas->H->r);
}

void kf_p_constrain(kf_state_t *s)
{
    for (int i = 0; i < s->P->r; i++)
    {
//        // State's standard deviation
//        s->Xstd[i] = sqrt(MELEMENT(s->P, i, i));

        // Constrain matrix P
        if (MELEMENT(s->P, i, i) < s->Pmin[i])
        {
            MELEMENT(s->P, i, i) = s->Pmin[i];
        }
        else if (MELEMENT(s->P, i, i) > s->Pmax[i])
        {
            nl_t p_scale = sqrt(s->Pmax[i] / MELEMENT(s->P, i, i));
            for (int j = 0; j < s->P->r; j++)
            {
                MELEMENT(s->P, i, j) *= p_scale;
                MELEMENT(s->P, j, i) *= p_scale;
            }
        }
    }
}

void kf_get_state_std(kf_state_t *s, int start_idx, int count, nl_t *std)
{
    if (!s->P) return;
    
    for (int i = 0; i < count; i++) {
        std[i] = sqrt(MELEMENT(s->P, start_idx + i, start_idx + i));
    }
}

void kf_state_update(kf_state_t *s, nl_t dt)
{
    /* X = F * X */
    mmul(s->F, s->X, s->Xtmp);
    mcopy(s->X, s->Xtmp);

    /* P = F * P * F' + Q*dt */
    mmul3("NT", 1, s->P, s->F, 0, s->Ptmp); /* P*F' */
    mmul3("NN", 1, s->F, s->Ptmp, 0, s->P); /* F*P*F' */
    madddiag2(s->P, dt, s->Q);              /* F*P*F' + dt*Q */

    /* Constrain the diagonal elements of matrix P */
    kf_p_constrain(s);
}

void kf_meas_update(kf_meas_t *m)
{
    kf_state_t *s = m->kf_state;
    
    // S = H * P * H' + R
    mmul3("NT", 1, s->P, m->H, 0, m->PHt); /* P*H' */
    mmul3("NN", 1, m->H, m->PHt, 0, m->S); /* H*P*H' */
    madddiag(m->S, m->R);                  /* H*P*H' + R */

    // z = z - H * x
   // mmul3("NN", -1, m->H, s->X, 1, m->Z); /* z - H * x */
    mcopy(m->innov, m->Z);      /* replace " mmul3("NN", -1, m->H, s->X, 1, m->Z); " just make sure we dont't change Z */
    mmul3("NN", -1, m->H, s->X, 1, m->innov); /* z - H * x */

    // K = P * H' * S^-1
    minv(m->S);
    
    mmul3("NN", 1, m->PHt, m->S, 0, m->K); /* P * H' * S^-1 */

    // x = x + K * (z - H * x)
    mmul3("NN", 1, m->K, m->innov, 1, s->X); /* x + K * (z - H * x) */

    // P = (I - K * H) * P
    mmul3("NN", -1, m->K, m->H, 0, s->KH);   /* - K * H */
    maddeye(s->KH);                          /* I - K * H */
    mmul3("NN", 1, s->KH, s->P, 0, s->Ptmp); /* (I - K * H) * P */
    // mcopy(s->P, s->Ptmp);

    // Keep matrix symmetry (P + P')/2
    mtrans(s->Ptmp, s->P);
    madd2(s->P, s->Ptmp);
    mscale2(s->P, 0.5);
}

nl_t kf_meas_get_chi_lambda(kf_meas_t *m)
{
    kf_state_t *s = m->kf_state;
    
    // S = H * P * H' + R
    mmul3("NT", 1, s->P, m->H, 0, m->PHt); /* P*H' */
    mmul3("NN", 1, m->H, m->PHt, 0, m->S); /* H*P*H' */
    madddiag(m->S, m->R);                  /* H*P*H' + R */

    // z = z - H * x
    mcopy(m->innov, m->Z);
    mmul3("NN", -1, m->H, s->X, 1, m->innov); /* z - H * x */

    // K = P * H' * S^-1
    minv(m->S);
    
    /* computer chi squre: 一种基于软卡方检测的自适应Ｋａｌｍａｎ滤波方法 */
    mmul3("NN", 1, m->S, m->innov, 0, m->chi_tmp); /* S*R */
    mmul3("TN", 1, m->innov, m->chi_tmp, 0, m->lambda); /* R'*S*R */
    return MELEMENT(m->lambda, 0, 0);
}

void Scalar_KalmanFilte(nl_t* x, nl_t zk, nl_t* P, nl_t Q, nl_t R, nl_t MaxError)
{
	nl_t K = 0.0f;
	nl_t x_ = 0.0f;
	nl_t P_ = 0.0f;
	nl_t err = 0;
	x_ = *x;
	P_ = *P + Q;
	K = P_ / (P_ + R);
	err = zk - x_;
	//误差限幅
	if (err > MaxError)
	{
		err = MaxError;
	}
	else if (err < -MaxError)
	{
		err = -MaxError;
	}	
	* x = x_ + K * (zk - x_);
	//	*x = x_+K*err;
	*P = P_ * (1.0f - K);
}
