/*
  * Copyright 2025 NXP
  *
  * SPDX-License-Identifier: BSD-3-Clause
  */
/*${header:start}*/
#include "pin_mux.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_lpuart.h"
#include "fsl_spc.h"
#include "fsl_opamp.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_mau.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "hardware_init.h"
#include "app_perf.h"
#include <stdbool.h>
/*${header:end}*/





/* Test pin for ADC timing measurement */
#define TEST_PIN_GPIO       GPIO3
#define TEST_PIN_PORT       PORT3
#define TEST_PIN_NUM        10U

#define HEARTBEAT_GPIO      GPIO3
#define HEARTBEAT_PORT      PORT3
#define HEARTBEAT_PIN       11U
#define HEARTBEAT_TOGGLE_INTERVAL_MS 500U

#define DEMO_OPAMP_COMP_CAP       kOPAMP_FitGain2x
#define DEMO_OPAMP_BIAS_CURRENT   kOPAMP_ChangeToQuarter



/*! @brief Initialize test pin P1_8 for ADC timing measurement */
void TestPin_Init(void)
{
    gpio_pin_config_t pin_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0  /* Start with low level */
    };
    
    /* Configure P1_8 as GPIO output */
    PORT_SetPinMux(TEST_PIN_PORT, TEST_PIN_NUM, kPORT_MuxAsGpio);
    GPIO_PinInit(TEST_PIN_GPIO, TEST_PIN_NUM, &pin_config);
    
}

static volatile uint32_t s_heartbeat_ms_accum = 0U;

void Heartbeat_Init(void)
{
    gpio_pin_config_t pin_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0
    };
    PORT_SetPinMux(HEARTBEAT_PORT, HEARTBEAT_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(HEARTBEAT_GPIO, HEARTBEAT_PIN, &pin_config);
    uint32_t core_hz = CLOCK_GetFreq(kCLOCK_CoreSysClk);
    SysTick_Config(core_hz / 1000U);
}

void SysTick_Handler(void)
{
    s_heartbeat_ms_accum++;
    if (s_heartbeat_ms_accum >= HEARTBEAT_TOGGLE_INTERVAL_MS)
    {
        s_heartbeat_ms_accum = 0U;
        GPIO_PortToggle(HEARTBEAT_GPIO, 1U << HEARTBEAT_PIN);
    }
}

/*! @brief Configure OPAMP modules */
void Opamp_Init(void)
{
    opamp_config_t config;

    RESET_ReleasePeripheralReset(kOPAMP0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kOPAMP1_RST_SHIFT_RSTn);

    SPC_EnableActiveModeAnalogModules(SPC0, (kSPC_controlOpamp0 | kSPC_controlOpamp1));
    
    OPAMP_GetDefaultConfig(&config);
    config.compCap     = DEMO_OPAMP_COMP_CAP;
    config.biasCurrent = DEMO_OPAMP_BIAS_CURRENT;
    config.enable = true;
    
    OPAMP_Init(OPAMP0, &config);
    OPAMP_Init(OPAMP1, &config);
    
    OPAMP_Enable(OPAMP0, true);
    OPAMP_Enable(OPAMP1, true);
}

void UART485_SetTxRts(LPUART_Type *LPUARTx, bool enable)
{
    if (enable)
    {
        LPUARTx->MODIR |= LPUART_MODIR_TXRTSE_MASK;
        LPUARTx->MODIR |= LPUART_MODIR_TXRTSPOL_MASK;
    }
    else
    {
        LPUARTx->MODIR &= ~LPUART_MODIR_TXRTSE_MASK;
        LPUARTx->MODIR &= ~LPUART_MODIR_TXRTSPOL_MASK;
    }
}

/*${function:start}*/
void Hardware_Init(void)
{
    Pins_Init();
    BOARD_InitBootClocks();
    Hardware_DebugConsoleInit();
    Opamp_Init();
    TestPin_Init();
    perf_init();
    Heartbeat_Init();
    
    mau_config_t cfg;
    MAU_GetDefaultConfig(&cfg);
    MAU_Init(MAU0, &cfg);
}

/*! @brief Set test pin to high level */
void TestPin_Set(void)
{
    GPIO_PinWrite(TEST_PIN_GPIO, TEST_PIN_NUM, 1U);
}

/*! @brief Set test pin to low level */
void TestPin_Clear(void)
{
    GPIO_PinWrite(TEST_PIN_GPIO, TEST_PIN_NUM, 0U);
}

void Hardware_DebugConsoleInit(void)
{
    CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
    CLOCK_AttachClk(HW_DEBUG_UART_CLK_ATTACH);
    RESET_PeripheralReset(HW_DEBUG_UART_RST);
    DbgConsole_Init(HW_DEBUG_UART_INSTANCE, HW_DEBUG_UART_BAUDRATE, kSerialPort_Uart, CLOCK_GetFreq(kCLOCK_FroHfDiv));
}
