/*
 * Copyright (c) 2023
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_timer.h"
#include "fsl_ctimer.h"
#include "fsl_clock.h"
#include "fsl_reset.h"

/* Use CTIMER0 for high precision timing */
#define TIMER_PERIPHERAL CTIMER0

/* Timer clock frequency in Hz - use 96MHz for higher precision */
#define TIMER_CLOCK_FREQ 96000000U  /* 96 MHz */

/* Microseconds per second */
#define US_PER_SEC 1000000U

/* Timer state tracking */
static volatile bool timerRunning = false;
static volatile uint32_t startValue = 0;

int TIMER_Init(void)
{
    ctimer_config_t timerConfig;
    
    /* Release CTIMER0 from reset */
    RESET_ReleasePeripheralReset(kCTIMER0_RST_SHIFT_RSTn);
    
    CLOCK_SetClockDiv(kCLOCK_DivCTIMER0, 1u);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER0);
    
    /* Get default CTIMER configuration */
    CTIMER_GetDefaultConfig(&timerConfig);
    
    /* Initialize CTIMER0 */
    CTIMER_Init(TIMER_PERIPHERAL, &timerConfig);
    
    /* Reset timer counter and prescale counter */
    TIMER_Reset();
    
    /* Start the timer */
    CTIMER_StartTimer(TIMER_PERIPHERAL);
    
    return 0;
}

void TIMER_Start(void)
{
    /* Store current timer value as start point */
    startValue = CTIMER_GetTimerCountValue(TIMER_PERIPHERAL);
    timerRunning = true;
}

float TIMER_Stop(void)
{
    uint32_t currentValue;
    uint32_t elapsedTicks;
    float elapsedUs;
    
    /* Get current timer value */
    currentValue = CTIMER_GetTimerCountValue(TIMER_PERIPHERAL);
    
    /* Calculate elapsed ticks, handling timer overflow */
    if (currentValue >= startValue) {
        elapsedTicks = currentValue - startValue;
    } else {
        /* Timer has wrapped around */
        elapsedTicks = (0xFFFFFFFFU - startValue) + currentValue + 1;
    }
    
    /* Convert ticks to microseconds with 0.1us precision */
    elapsedUs = ((float)elapsedTicks * (float)US_PER_SEC) / (float)TIMER_CLOCK_FREQ;
    
    /* Reset timer state */
    timerRunning = false;
    
    return elapsedUs;
}

float TIMER_GetValue(void)
{
    uint32_t currentValue;
    uint32_t elapsedTicks;
    float elapsedUs;
    
    /* Get current timer value */
    currentValue = CTIMER_GetTimerCountValue(TIMER_PERIPHERAL);
    
    /* If timer is not running, just return current value in microseconds */
    if (!timerRunning) {
        return ((float)currentValue * (float)US_PER_SEC) / (float)TIMER_CLOCK_FREQ;
    }
    
    /* Calculate elapsed ticks, handling timer overflow */
    if (currentValue >= startValue) {
        elapsedTicks = currentValue - startValue;
    } else {
        /* Timer has wrapped around */
        elapsedTicks = (0xFFFFFFFFU - startValue) + currentValue + 1;
    }
    
    /* Convert ticks to microseconds with 0.1us precision */
    elapsedUs = ((float)elapsedTicks * (float)US_PER_SEC) / (float)TIMER_CLOCK_FREQ;
    
    return elapsedUs;
}

void TIMER_Reset(void)
{
    /* Reset timer counter and prescale counter */
    TIMER_PERIPHERAL->TC = 0;
    TIMER_PERIPHERAL->PR = 0;
    
    /* Reset timer state */
    timerRunning = false;
    startValue = 0;
}
