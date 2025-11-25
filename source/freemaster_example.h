/*
 * Copyright (c) 2007-2015 Freescale Semiconductor, Inc.
 * Copyright 2018-2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * FreeMASTER Communication Driver - Example Application Declarations
 */

#ifndef __FMSTR_EXAMPLE_H
#define __FMSTR_EXAMPLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * Functions definitions
 ******************************************************************************/

// Prototypes of example functions
void FMSTR_Example_Init(void);

extern volatile uint8_t fm_cal_enable;
extern volatile uint8_t fm_cal_done;
extern volatile uint16_t fm_cal_progress;

#ifdef __cplusplus
}
#endif

#endif /* __FMSTR_EXAMPLE_H */
