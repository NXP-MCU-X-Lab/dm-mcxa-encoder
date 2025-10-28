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

/*!
 * @brief Initialize hardware components
 */
void BOARD_InitHardware(void);

/*!
 * @brief Initialize test pin for ADC timing measurement
 */
void TEST_PIN_Init(void);

/*!
 * @brief Set test pin to high level
 */
void TEST_PIN_Set(void);

/*!
 * @brief Set test pin to low level
 */
void TEST_PIN_Clear(void);

/*!
 * @brief Toggle test pin level
 */
void TEST_PIN_Toggle(void);

#if defined(__cplusplus)
}
#endif

#endif /* _HARDWARE_INIT_H_ */