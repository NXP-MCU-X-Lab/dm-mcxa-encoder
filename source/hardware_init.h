/*
 * Copyright 2024 NXP
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

void BOARD_InitHardware(void);
void BOARD_InitUART485Control(LPUART_Type *LPUARTx, uint8_t enable);
void TEST_PIN_Init(void);
void TEST_PIN_Set(void);
void TEST_PIN_Clear(void);

#if defined(__cplusplus)
}
#endif

#endif /* _HARDWARE_INIT_H_ */