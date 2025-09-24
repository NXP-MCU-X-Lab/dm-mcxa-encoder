/**
  ******************************************************************************
  * @file    fir.c
  * @author  YANDLD
  * @version V2.5
  * @date    2017.10.08

  * @note    
  ******************************************************************************
  */
#include <math.h>
#include "nl_filter.h"

/* coeffs can be get from matlab fir1 function, or use fdatool*/
int fir_create(fir_t *s, int tap_size, nl_t *coeffs)
{
    int i;
    s->tap_size = s->max_tap_size = tap_size;
    s->coeffs = coeffs;
    
    s->buf = vcreate(tap_size);
    
    /* init delay buffer */
    for(i=0; i<s->tap_size; i++)
    {
        s->buf[i] = 0.0F;
    }
    return 0;
}


void fir_run(fir_t *s, nl_t *in)
{
    int i;
    
    /* shifing the buffer */
    for(i=s->tap_size-1; i>0; i--)
    {
        s->buf[i] = s->buf[i-1];
    }
    s->buf[0] = (*in);
}

nl_t fir_get_result(fir_t *s)
{
    /* sum all buffer value with coeffs */
    nl_t val = 0;
    for(int i=0; i<s->tap_size; i++)
    {
        val += s->buf[i] * s->coeffs[i];
    }
    return val;
}


 


