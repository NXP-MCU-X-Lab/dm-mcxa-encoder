/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#ifndef _HARDWARE_INIT_H_
#define _HARDWARE_INIT_H_

#include "fsl_common.h"
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/* Debug console configuration */
#define HW_DEBUG_UART_INSTANCE   0U
#define HW_DEBUG_UART_BAUDRATE   115200U
#define HW_DEBUG_UART_CLK_ATTACH kFRO_HF_DIV_to_LPUART0
#define HW_DEBUG_UART_RST        kLPUART0_RST_SHIFT_RSTn

void Hardware_Init(void);
void Hardware_DebugConsoleInit(void);
void UART485_SetTxRts(LPUART_Type *LPUARTx, bool enable);
void TestPin_Init(void);
void TestPin_Set(void);
void TestPin_Clear(void);
void Heartbeat_Init(void);

#if defined(__cplusplus)
}
#endif

#endif /* _HARDWARE_INIT_H_ */
