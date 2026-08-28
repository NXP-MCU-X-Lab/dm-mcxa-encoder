/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#ifndef _HARDWARE_INIT_H_
#define _HARDWARE_INIT_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

void Hardware_Init(void);
void TestPin_Init(void);
void TestPin_Set(void);
void TestPin_Clear(void);
void Heartbeat_Init(void);
void Heartbeat_Service(void);

/* SysTick-driven millisecond delay (independent of SDK_DelayAtLeastUs). */
extern volatile uint32_t g_systick_ms;
void delay_ms(uint32_t ms);

#if defined(__cplusplus)
}
#endif

#endif /* _HARDWARE_INIT_H_ */
