/*
 * Copyright (c) 2023
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _APP_TIMER_H_
#define _APP_TIMER_H_

#include <stdint.h>

/**
 * @brief Initialize high precision timer
 * 
 * @return 0 on success, non-zero on error
 */
int TIMER_Init(void);

/**
 * @brief Start the timer measurement
 */
void TIMER_Start(void);

/**
 * @brief Stop the timer measurement and return elapsed time
 * 
 * @return Elapsed time in microseconds (with 0.1us precision)
 */
float TIMER_Stop(void);

/**
 * @brief Get current timer value without stopping
 * 
 * @return Current timer value in microseconds (with 0.1us precision)
 */
float TIMER_GetValue(void);

/**
 * @brief Reset timer counter to zero
 */
void TIMER_Reset(void);

#endif /* _APP_TIMER_H_ */
