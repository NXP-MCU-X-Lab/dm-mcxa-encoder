/**
  ******************************************************************************
  * @file    iir.h
  * @author  YANDLD
  * @version V2.5
  * @date    2015.3.26

  * @note    
  ******************************************************************************
  */

#include <math.h>
#include "nl_filter.h"



//void iir_create(iir_t *s, nl_t *b, int b_size, nl_t *a, int a_size)
//{
//    int i;
//    s->b = b;
//    s->a = a;
//    s->b_size = b_size;
//    s->a_size = a_size;
//    
//    s->xbuf = vcreate(b_size);
//    s->ybuf = vcreate(a_size);
//    
//    for(i=0; i<s->b_size; i++)
//    {
//        s->xbuf[i] = 0.0F;
//    }
//    
//    for(i=0; i<s->a_size; i++)
//    {
//        s->ybuf[i] = 0.0F;
//    }
//}

//void iir_run(iir_t *s, nl_t *in, nl_t *out)
//{
//    /* shifing the buffer */
//    for(s->i=s->b_size-1; s->i>0; s->i--)
//    {
//        s->xbuf[s->i] = s->xbuf[s->i-1];
//    }
//    s->xbuf[0] = (*in);
//    
//    /* sum all buffer value with coeffs */
//    s->ybuf[0] = 0;
//    
//    /* adding all x items */
//    for(s->i=0; s->i<s->b_size; s->i++)
//    {
//        s->ybuf[0] += s->xbuf[s->i] * s->b[s->i];
//    }
//    
//    /* subtract(feedback) all y items */
//    for(s->i=1; s->i<s->a_size; s->i++)
//    {
//        s->ybuf[0] -= s->ybuf[s->i] * s->a[s->i];
//    }
//    
//    /* ybuf update, shift the ybuf */
//    for(s->i=s->a_size-1; s->i>0; s->i--)
//    {
//        s->ybuf[s->i] = s->ybuf[s->i-1];
//    }
//    
//    *out = s->ybuf[0];
//}



/**
 * @brief  Initialize exponential filter instance
 * @note   This is a first-order IIR low-pass filter implementation
 *         Transfer function: H(z) = a/(1 - (1-a)z^(-1))
 *         Difference equation: y[n] = a*x[n] + (1-a)*y[n-1]
 *         where a = dt/(rc + dt), rc = 1/(2p*fc)
 * 
 * @param  filter: Pointer to filter instance
 * @param  cutoff_freq: Cut-off frequency in Hz
 * @param  sample_freq: Sampling frequency in Hz
 */
void exp_filter_init(exp_filter_t *filter, nl_t cutoff_freq, nl_t sample_freq)
{
    // Calculate filter coefficient
    nl_t rc = 1.0f / (2.0f * M_PI * cutoff_freq);
    nl_t dt = 1.0f / sample_freq;
    filter->alpha = dt / (rc + dt);
    
    // Initialize previous output
    filter->y_prev = 0.0f;
}


/**
 * @brief  Apply exponential filter to input value
 * @note   This implements the first-order difference equation:
 *         y[n] = a*x[n] + (1-a)*y[n-1]
 *         Time constant t = 1/(2p*fc) = rc
 *         Settling time (99%) ˜ 5t
 * 
 * @param  filter: Pointer to filter instance
 * @param  in: Input value
 * @return Filtered output value
 */
nl_t exp_filter_update(exp_filter_t *filter, nl_t in)
{
    // Exponential filter core equation
    nl_t out = filter->y_prev + filter->alpha * (in - filter->y_prev);
    filter->y_prev = out;
    return out;
}




/**
 * @brief Initialize a 2nd order Butterworth lowpass filter
 * @param filter Pointer to filter instance
 * @param fc Cutoff frequency in Hz
 * @param fs Sampling frequency in Hz
 * @note Butterworth filter provides maximally flat response in the passband
 */
void biquad_lowpass_init(biquad_filter_t *filter, nl_t fc, nl_t fs)
{
    // Pre-warp the cutoff frequency for bilinear transform
    const nl_t omega = 2.0f * M_PI * fc;
    const nl_t omega_d = 2.0f * fs * tanf(omega / (2.0f * fs));
    
    // Compute analog prototype coefficients
    const nl_t alpha = omega_d / fs;
//    const nl_t beta = cosf(omega / fs);
    
    // Compute digital coefficients
    const nl_t a0_inv = 1.0f / (alpha * alpha + 2.0f * alpha + 4.0f);
    
    // Numerator coefficients
    filter->b0 = alpha * alpha * a0_inv;
    filter->b1 = 2.0f * alpha * alpha * a0_inv;
    filter->b2 = alpha * alpha * a0_inv;
    
    // Denominator coefficients
    filter->a1 = 2.0f * (alpha * alpha - 4.0f) * a0_inv;
    filter->a2 = (alpha * alpha - 2.0f * alpha + 4.0f) * a0_inv;
    
    // Clear filter history
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
}

/**
 * @brief Process one sample through the biquad filter
 * @param filter Pointer to filter instance
 * @param input Input sample
 * @return Filtered output
 * @note Uses Direct Form II structure for better numerical properties
 */
nl_t biquad_update(biquad_filter_t *filter, nl_t input)
{
    // Direct Form II implementation
    nl_t output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                  - filter->a1 * filter->y1 - filter->a2 * filter->y2;
                  
    // Update filter history
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    return output;
}


/**
 * @brief Initialize a 2nd order Butterworth highpass filter
 * @param filter Pointer to filter instance
 * @param fc Cutoff frequency in Hz
 * @param fs Sampling frequency in Hz
 * @note 
 *      - Uses frequency pre-warping to compensate for bilinear transform
 *      - Implements Direct Form II for better numerical stability
 *      - All coefficients are normalized by a0
 */
void biquad_highpass_init(biquad_filter_t *filter, nl_t fc, nl_t fs)
{
    // Calculate normalized frequency
    const nl_t omega = 2.0f * M_PI * fc / fs;  // ??????
    const nl_t cos_omega = cosf(omega);
    const nl_t sin_omega = sinf(omega);
    const nl_t alpha = sin_omega / (2.0f * 0.7071f); // Q=0.7071 for Butterworth

    // Alpha value for Butterworth filter (Q=0.7071)
    const nl_t a0_inv = 1.0f / (1.0f + alpha);
    filter->b0 = (1.0f + cos_omega) * 0.5f * a0_inv;
    filter->b1 = -(1.0f + cos_omega) * a0_inv;
    filter->b2 = (1.0f + cos_omega) * 0.5f * a0_inv;

    // Numerator coefficients (zeros)
    filter->a1 = -2.0f * cos_omega * a0_inv;
    filter->a2 = (1.0f - alpha) * a0_inv;

    // Reset filter states
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
}


/**
 * @brief Initialize first-order allpass filter
 * @param filter Pointer to filter instance
 * @param freq Target frequency in Hz
 * @param fs Sampling frequency in Hz
 * @param phase_deg Desired phase shift in degrees at target frequency
 */
void allpass_filter_init(allpass_filter_t *filter, nl_t freq, nl_t fs, nl_t phase_deg)
{
    // Convert phase from degrees to radians
    nl_t phase_rad = phase_deg * M_PI / 180.0f;
    
    // Calculate normalized frequency
    nl_t omega = 2.0f * M_PI * freq / fs;
    nl_t tan_half = tanf(omega * 0.5f);
    
    // Calculate filter coefficient
    filter->alpha = (tan_half - tanf(phase_rad * 0.5f)) / 
                   (tan_half + tanf(phase_rad * 0.5f));
    
    // Initialize state variables
    filter->x_prev = 0.0f;
    filter->y_prev = 0.0f;
}

/**
 * @brief Process one sample through allpass filter
 * @param filter Pointer to filter instance
 * @param input Input sample
 * @return Filtered output
 */
nl_t allpass_filter_update(allpass_filter_t *filter, nl_t input)
{
    // Allpass filter difference equation:
    // y[n] = alpha * x[n] + x[n-1] - alpha * y[n-1]
    nl_t output = filter->alpha * input + filter->x_prev - 
                 filter->alpha * filter->y_prev;
    
    // Update state variables
    filter->x_prev = input;
    filter->y_prev = output;
    
    return output;
}

/**
 * @brief Reset allpass filter state
 * @param filter Pointer to filter instance
 */
void allpass_filter_reset(allpass_filter_t *filter)
{
    filter->x_prev = 0.0f;
    filter->y_prev = 0.0f;
}

/**
 * @brief Update allpass filter coefficient for new frequency/phase
 * @param filter Pointer to filter instance
 * @param freq Target frequency in Hz
 * @param fs Sampling frequency in Hz
 * @param phase_deg Desired phase shift in degrees
 */
void allpass_filter_update_coeff(allpass_filter_t *filter, nl_t freq, nl_t fs, nl_t phase_deg)
{
    nl_t phase_rad = phase_deg * M_PI / 180.0f;
    nl_t omega = 2.0f * M_PI * freq / fs;
    nl_t tan_half = tanf(omega * 0.5f);
    
    filter->alpha = (tan_half - tanf(phase_rad * 0.5f)) / 
                   (tan_half + tanf(phase_rad * 0.5f));
}




/**
 * @brief Initialize a 2nd order Butterworth bandpass filter
 * @param filter Pointer to filter instance
 * @param f_low Lower cutoff frequency in Hz
 * @param f_high Higher cutoff frequency in Hz
 * @param fs Sampling frequency in Hz
 * @note 
 *      - Implements a bandpass filter using biquad structure
 *      - Optimized for limited computational resources
 *      - Designed for passing signals between f_low and f_high Hz
 */
void biquad_bandpass_init(biquad_filter_t *filter, nl_t f_low, nl_t f_high, nl_t fs)
{
    // Calculate center frequency and bandwidth
    nl_t f0 = sqrtf(f_low * f_high);  // Geometric mean for center frequency
    nl_t bw = f_high - f_low;         // Bandwidth
    
    // Normalize frequencies to sampling rate
    nl_t omega0 = 2.0f * M_PI * f0 / fs;
    nl_t alpha = sinf(omega0) * sinhf(logf(2.0f) / 2.0f * bw / f0 * omega0 / sinf(omega0));
    
    // For low computational resources, we can use a simplified approach
    // with a reasonable Q factor for our bandwidth
    nl_t Q = f0 / bw;
    
    // If computational resources are very limited, use this simpler alpha calculation
    // which is an approximation but works well for moderate Q values
    alpha = sinf(omega0) / (2.0f * Q);
    
    nl_t cos_omega0 = cosf(omega0);
    
    // Calculate filter coefficients
    nl_t a0_inv = 1.0f / (1.0f + alpha);
    
    // Numerator coefficients (zeros)
    filter->b0 = alpha * a0_inv;
    filter->b1 = 0.0f;
    filter->b2 = -alpha * a0_inv;
    
    // Denominator coefficients (poles)
    filter->a1 = -2.0f * cos_omega0 * a0_inv;
    filter->a2 = (1.0f - alpha) * a0_inv;
    
    // Reset filter states
    filter->x1 = filter->x2 = 0.0f;
    filter->y1 = filter->y2 = 0.0f;
}



