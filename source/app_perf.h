/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#ifndef APP_PERF_H
#define APP_PERF_H

#include <stdint.h>

typedef struct {
    uint32_t adc_us;
    uint32_t atan2_us;
    uint32_t algo_us;
    uint32_t isr_us;
} perf_metrics_t;

void perf_init(void);
void perf_record_atan2_cycles(uint32_t cycles);
uint32_t perf_get_atan2_cycles(void);
void perf_set_cycles(uint32_t adc_cycles, uint32_t atan2_cycles, uint32_t algo_cycles, uint32_t isr_cycles);
extern volatile perf_metrics_t g_perf;

#endif
