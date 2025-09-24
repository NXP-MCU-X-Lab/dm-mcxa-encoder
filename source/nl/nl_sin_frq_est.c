#include "nl_sin_frq_est.h"
#include <math.h>


/* Default parameters */
#define DEFAULT_A 1.0f
#define DEFAULT_B 1.0f
#define DEFAULT_K 1.0f

/**
 * @brief Initialize the frequency estimator
 * @param est Pointer to estimator instance
 */
void nl_sin_frq_est_init(nl_sin_frq_est_t *est)
{
    // Initialize parameters with default values
    est->a = DEFAULT_A;
    est->b = DEFAULT_B;
    est->k = DEFAULT_K;

    // Initialize states
    est->y = 0;
    est->x1 = 0;
    est->theta = -(est->a * est->a / 4.0f);  // Initial frequency estimate
    est->sigma = est->theta;
    est->omega = 0;
    est->freq = 0;
}

/**
 * @brief Configure the frequency estimator parameters
 * @param est Pointer to estimator instance
 * @param a Low-pass filter coefficient
 * @param b Observer gain
 * @param k Estimator gain
 */
void nl_sin_frq_est_config(nl_sin_frq_est_t *est, nl_t a, nl_t b, nl_t k)
{
    est->a = a;
    est->b = b;
    est->k = k;
    
    // Reset initial estimate based on new parameters
    est->theta = -(est->a * est->a / 4.0f);
    est->sigma = est->theta;
}

/**
 * @brief Update frequency estimate with new measurement
 * @param est Pointer to estimator instance
 * @param y Current measurement value
 * @param dt Time step in seconds
 * 
 * Implementation based on:
 * "The New Algorithm of Sinusoidal Signal Frequency Estimation"
 * by Bobtsov et al., IFAC 2013
 */
void nl_sin_frq_est_update(nl_sin_frq_est_t *est, nl_t y, nl_t dt)
{
    nl_t x1_dot, sigma_dot;
    
    // Store measurement
    est->y = y;
    
    // Calculate state derivative
    x1_dot = -est->a * est->x1 + est->b * y;
    
    // Calculate auxiliary variable derivative
    sigma_dot = -est->k * est->x1 * est->x1 * est->theta 
                - est->k * est->a * est->x1 * x1_dot 
                - est->k * est->b * x1_dot * y;
                
    // Update states using Euler integration
    est->x1 = est->x1 + x1_dot * dt;
    est->sigma = est->sigma + sigma_dot * dt;
    
    // Update frequency estimate
    est->theta = est->sigma + est->k * est->b * est->x1 * y;
    est->omega = sqrtf(fabsf(est->theta));
    est->freq = est->omega / (2.0f * M_PI);
}



///**
// * Usage examples for nl_sin_frq_est
// * 
// * Parameter Guide:
// * 1. a (omega_up): 
// *    - Set to highest expected frequency (rad/s)
// *    - Higher: Fast response, noise sensitive
// *    - Lower: Noise robust, slower response
// *    - Range: 2p * [0.5 to 2.0] for waves
// * 
// * 2. b: Observer gain
// *    - Usually equals 'a'
// *    - Controls amplitude tracking speed
// * 
// * 3. k: Estimator gain
// *    - Controls frequency tracking speed
// *    - Higher: Fast but noisy
// *    - Lower: Stable but slow
// *    - Range: 1.0 to 5.0
// *    - Start with 2.0
// */


