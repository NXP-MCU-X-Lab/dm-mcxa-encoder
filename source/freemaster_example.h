/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
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
extern volatile uint8_t fm_reset_ctrl;
extern volatile uint8_t fm_zero_ctrl;
extern volatile uint8_t fm_direction;

#ifdef __cplusplus
}
#endif

#endif /* __FMSTR_EXAMPLE_H */
