/**
  ******************************************************************************
  * @file    moving_average.c
  * @author  YANDLD
  * @version V2.5
  * @date    2015.3.26
  * @brief   Moving average filter implementation based on FIR filter structure
  * @note    This is a simple moving average filter implementation that uses
  *          equal weights for all samples in the window
  ******************************************************************************
  */

#include "nl_filter.h"

/**
 * @brief Set the window size for the moving average filter
 * @param s Pointer to the FIR filter structure
 * @param win_size Desired window size for averaging
 * @note Window size must be between 2 and max_tap_size
 *       All coefficients are set to 1/win_size for equal weighting
 */
void moving_avg_set_win_size(fir_t *s, uint8_t win_size)
{
    int i;
    if(win_size <= s->max_tap_size && win_size > 1) s->tap_size = win_size;
    
    for(i=0; i<win_size; i++)
    {
        s->coeffs[i] = 1 / ((nl_t)win_size);  // Set equal weights for all samples
    }
}

/**
 * @brief Create and initialize a new moving average filter
 * @param s Pointer to the FIR filter structure to be initialized
 * @param win_size Size of the moving average window
 * @return Status of creation (likely returns 0 for success)
 * @note Creates a FIR filter with coefficients all set to 1/win_size
 */
int moving_avg_create(fir_t *s, uint8_t win_size)
{
    int i;
    
    nl_t *coeffs = vcreate(win_size);  // Allocate memory for coefficients
    
    for(i=0; i<win_size; i++)
    {
        coeffs[i] = 1 / ((nl_t)win_size);  // Initialize coefficients for equal weighting
    }
    return fir_create(s, win_size, coeffs);  // Initialize the underlying FIR filter
}

/**
 * @brief Process a new input sample through the moving average filter
 * @param s Pointer to the FIR filter structure
 * @param in Pointer to the input sample
 * @note This is a wrapper function for the FIR filter processing
 */
void moving_avg_run(fir_t *s, nl_t *in)
{
    fir_run(s, in);
}

/**
 * @brief Get the current output value of the moving average filter
 * @param s Pointer to the FIR filter structure
 * @return The current filtered (averaged) value
 * @note This is a wrapper function for getting FIR filter output
 */
nl_t moving_avg_get_result(fir_t *s)
{
    return fir_get_result(s);
}
