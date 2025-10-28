/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*! @brief The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE     kSerialPort_Uart
#define BOARD_DEBUG_UART_CLK_FREQ 12000000U
#define BOARD_DEBUG_UART_BAUDRATE 115200U


#define BOARD_DEBUG_UART_BASEADDR   (uint32_t) LPUART2
#define BOARD_DEBUG_UART_INSTANCE   2U
#define BOARD_DEBUG_UART_CLK_ATTACH kFRO_LF_DIV_to_LPUART2
#define BOARD_DEBUG_UART_RST        kLPUART2_RST_SHIFT_RSTn
#define BOARD_DEBUG_UART_CLKSRC     kCLOCK_LPUART2
#define BOARD_UART_IRQ_HANDLER      LPUART2_IRQHandler
#define BOARD_UART_IRQ              LPUART2_IRQn



#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
void BOARD_InitDebugConsole(void);


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
