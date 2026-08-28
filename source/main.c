/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */

#include "fsl_common.h"

#include "app_adc.h"
#include "app_encoder_runtime.h"
#include "app_freemaster.h"
#include "app_tformat.h"
#include "freemaster.h"
#include "freemaster_cfg.h"
#include "freemaster_serial_lpuart.h"
#include "hardware_init.h"

static void run_main_loop(void)
{
    while (1)
    {
        EncoderApp_Service();
        FMSTR_Poll();
        Heartbeat_Service();
        __WFI();
    }
}

int main(void)
{
    Hardware_Init();
    adc_init();
    AppFreemaster_Init();
    TFormat_Init();
    EncoderApp_Init();

    run_main_loop();
}

#if FMSTR_SHORT_INTR || FMSTR_LONG_INTR

void LPUART0_IRQHandler(void)
{
    FMSTR_SerialIsr();
}

#endif

void HardFault_Handler(void)
{
    while (1)
    {
    }
}
