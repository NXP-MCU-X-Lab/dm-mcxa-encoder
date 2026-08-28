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

extern volatile uint8_t fm_command;
extern volatile uint8_t fm_command_state;
extern volatile uint32_t fm_command_status;
extern volatile uint8_t fm_factory_cal_progress;
extern volatile uint8_t fm_encoder_ready;
extern volatile uint8_t fm_encoder_stationary;

void AppFreemaster_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FREEMASTER_H_ */
