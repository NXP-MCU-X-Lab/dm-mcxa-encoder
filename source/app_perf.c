/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#include "app_perf.h"
#include "fsl_clock.h"
#include "app_sampler.h"
#include <stddef.h>

volatile perf_metrics_t g_perf;
static uint32_t s_timer_hz;
static volatile uint32_t s_atan2_cycles;

static uint32_t ticks_to_us(uint32_t ticks)
{
    if (s_timer_hz == 0) {
        s_timer_hz = sampler_get_timer_clock_hz();
        if (s_timer_hz == 0) return 0;
    }
    uint64_t v = (uint64_t)ticks * 1000000ULL;
    return (uint32_t)(v / (uint64_t)s_timer_hz);
}

void perf_init(void)
{
    s_timer_hz = sampler_get_timer_clock_hz();
    g_perf.adc_us = 0;
    g_perf.atan2_us = 0;
    g_perf.algo_us = 0;
    g_perf.isr_us = 0;
    s_atan2_cycles = 0;
}

void perf_record_atan2_cycles(uint32_t cycles)
{
    s_atan2_cycles = cycles;
}

uint32_t perf_get_atan2_cycles(void)
{
    return s_atan2_cycles;
}

void perf_set_cycles(uint32_t adc_cycles, uint32_t atan2_cycles, uint32_t algo_cycles, uint32_t isr_cycles)
{
    g_perf.adc_us = ticks_to_us(adc_cycles);
    g_perf.atan2_us = ticks_to_us(atan2_cycles);
    g_perf.algo_us = ticks_to_us(algo_cycles);
    g_perf.isr_us = ticks_to_us(isr_cycles);
}
