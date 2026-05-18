/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#ifndef APP_FREEMASTER_H_
#define APP_FREEMASTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void AppFreemaster_Init(void);

extern volatile uint8_t fm_reset_ctrl;
extern volatile uint8_t fm_zero_ctrl;
extern volatile uint8_t fm_factory_cal_ctrl;
extern volatile uint8_t fm_factory_cal_state;
extern volatile uint8_t fm_factory_cal_progress;
extern volatile uint32_t fm_factory_cal_status;
extern volatile uint8_t fm_encoder_valid;
extern volatile uint32_t fm_encoder_status;
#ifdef __cplusplus
}
#endif

#endif /* APP_FREEMASTER_H_ */
