/**
  ******************************************************************************
  * @file    fir.h
  * @author  YANDLD
  * @version V2.5
  * @date    2017.10.08

  * @note    
  ******************************************************************************
  */
#ifndef _FTILER_H_
#define _FTILER_H_

#include "nl.h"


typedef struct
{
    uint8_t     tap_size;                   /* tap size is the number of coeffs and should be (order of filter + 1). which shoud be 3,5,7,9.... */
    uint8_t     max_tap_size;
    nl_t        *coeffs;                    /* pCoeffs points to the array of filter coefficients, same as matlab func: fir1, reversed order of arm_fir_create_f32 */
    nl_t        *buf;                       /* filter state */
}fir_t;

//typedef struct
//{
//    nl_t *b;          /* xbuf coeffict b, numerator */
//    nl_t *a;          /* ybuf coeffict a, denominator */
//    int b_size;
//    int a_size;
//    nl_t *ybuf;
//    nl_t *xbuf;
//    int i;
//}iir_t;

typedef struct {
    nl_t y_prev;       // Previous output value
    nl_t alpha;        // Filter coefficient
} exp_filter_t;


/**
 * @brief Biquad IIR filter structure
 * Transfer function: H(z) = (b0 + b1*z^-1 + b2*z^-2)/(1 + a1*z^-1 + a2*z^-2)
 * Direct Form II implementation for better numerical stability
 */
typedef struct {
    nl_t b0, b1, b2;        // Numerator coefficients
    nl_t a1, a2;            // Denominator coefficients
    nl_t x1, x2;            // Input history
    nl_t y1, y2;            // Output history
} biquad_filter_t;

/**
 * @brief First-order allpass filter structure
 */
typedef struct {
    nl_t alpha;     // Filter coefficient
    nl_t x_prev;    // Previous input
    nl_t y_prev;    // Previous output
} allpass_filter_t;


int fir_create(fir_t *s, int tap_size, nl_t *coeffs);
void fir_run(fir_t *s, nl_t *in);
//void iir_create(iir_t *s, nl_t *b, int b_size, nl_t *a, int a_size);
//void iir_run(iir_t *s, nl_t *in, nl_t *out);

void exp_filter_init(exp_filter_t *filter, nl_t cutoff_freq, nl_t sample_freq);
nl_t exp_filter_update(exp_filter_t *filter, nl_t in);

void biquad_highpass_init(biquad_filter_t *filter, nl_t fc, nl_t fs);
void biquad_lowpass_init(biquad_filter_t *filter, nl_t fc, nl_t fs);
void biquad_bandpass_init(biquad_filter_t *filter, nl_t f_low, nl_t f_high, nl_t fs);
nl_t biquad_update(biquad_filter_t *filter, nl_t input);
                     
void allpass_filter_init(allpass_filter_t *filter, nl_t freq, nl_t fs, nl_t phase_deg);
nl_t allpass_filter_update(allpass_filter_t *filter, nl_t input);
void allpass_filter_reset(allpass_filter_t *filter);
void allpass_filter_update_coeff(allpass_filter_t *filter, nl_t freq, nl_t fs, nl_t phase_deg);



nl_t fir_get_result(fir_t *s);

int moving_avg_create(fir_t *s, uint8_t win_size);
void moving_avg_run(fir_t *s, nl_t *in);
nl_t moving_avg_get_result(fir_t *s);
void moving_avg_set_win_size(fir_t *s, uint8_t win_size);

#endif
