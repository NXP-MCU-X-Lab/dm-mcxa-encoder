#ifndef NL_SIN_FRQ_EST_H
#define NL_SIN_FRQ_EST_H

#include "nl.h"


typedef struct {
    /* Parameters */
    nl_t a;        // Low-pass filter coefficient
    nl_t b;        // Observer gain
    nl_t k;        // Estimator gain

    /* States */
    nl_t y;        // Measurement signal
    nl_t x1;       // State estimate
    nl_t theta;    // Frequency parameter estimate
    nl_t sigma;    // Auxiliary variable
    nl_t omega;    // Estimated angular frequency
    nl_t freq;     // Estimated frequency in Hz
} nl_sin_frq_est_t;

void nl_sin_frq_est_init(nl_sin_frq_est_t *est);
void nl_sin_frq_est_config(nl_sin_frq_est_t *est, nl_t a, nl_t b, nl_t k);
void nl_sin_frq_est_update(nl_sin_frq_est_t *est, nl_t y, nl_t dt);

#endif
