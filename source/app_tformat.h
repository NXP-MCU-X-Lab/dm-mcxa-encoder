/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef APP_TFORMAT_H_
#define APP_TFORMAT_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TFORMAT_EEPROM_PAGE_SIZE  (127U)
#define TFORMAT_EEPROM_DATA_PAGES (6U)
#define TFORMAT_EEPROM_PAGE_COUNT (8U)
#define TFORMAT_EEPROM_SIZE       (TFORMAT_EEPROM_PAGE_SIZE * TFORMAT_EEPROM_DATA_PAGES)

#define TFORMAT_RESET_ERROR     (1UL << 0)
#define TFORMAT_RESET_POSITION  (1UL << 1)
#define TFORMAT_RESET_MULTITURN (1UL << 2)

void TFormat_Init(void);
void TFormat_Publish(const encoder_result_t *result, bool ready, bool stationary);
uint32_t TFormat_TakeResetRequests(void);
void TFormat_ReportCountingError(void);

void TFormat_LoadEeprom(const uint8_t data[TFORMAT_EEPROM_SIZE]);
void TFormat_CopyEeprom(uint8_t data[TFORMAT_EEPROM_SIZE]);
bool TFormat_EepromWritePending(void);
void TFormat_CompleteEepromWrite(bool saved);
bool TFormat_StorageBegin(void);
void TFormat_StorageEnd(void);

extern volatile uint32_t tformat_id0_request_count;
extern volatile uint32_t tformat_id3_request_count;
extern volatile uint32_t tformat_response_count;
extern volatile uint32_t tformat_busy_count;
extern volatile uint32_t tformat_unsupported_count;
extern volatile uint32_t tformat_uart_error_count;
extern volatile uint32_t tformat_stale_count;
extern volatile uint32_t tformat_crc_error_count;
extern volatile uint32_t tformat_desync_count;
extern volatile uint32_t tformat_eeprom_write_count;
extern volatile uint32_t tformat_eeprom_error_count;

#ifdef __cplusplus
}
#endif

#endif /* APP_TFORMAT_H_ */
