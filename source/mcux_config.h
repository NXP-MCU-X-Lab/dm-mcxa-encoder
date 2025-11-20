/*
 * Copyright 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MCUX_CONFIG_H_
#define _MCUX_CONFIG_H_

#define CONFIG_FLASH_BASE_ADDRESS 0x0


#define APP_MODE_ASCII            1
#define APP_MODE_DEBUG            2
#define APP_MODE_CALIBRATE_ASCII  3
#define APP_MODE_UART_DMA         4
#define APP_MODE_FREEMASTER       5

#ifndef APP_START_MODE
#define APP_START_MODE APP_MODE_FREEMASTER
#endif

#endif /* _MCUX_CONFIG_H_ */
