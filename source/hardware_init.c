/*
 * Copyright 2024 NXP
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
#include "board.h"
#include "hardware_init.h"
#include <stdbool.h>
/*${header:end}*/





/* Test pin for ADC timing measurement */
#define TEST_PIN_GPIO       GPIO1
#define TEST_PIN_PORT       PORT1
#define TEST_PIN_NUM        8U

#define DEMO_OPAMP_COMP_CAP       kOPAMP_FitGain2x
#define DEMO_OPAMP_BIAS_CURRENT   kOPAMP_ChangeToQuarter


void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}


void BOARD_InitRS485_Tamagawa(void)
{
    lpuart_config_t config;
    
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200;
    config.enableTx = true;
    config.enableRx = true;
    
    
    LPUART_Init(LPUART2, &config, CLOCK_GetFreq(kCLOCK_FroHfDiv));
    
    LPUART2->MODIR |= LPUART_MODIR_TXRTSE_MASK;
    LPUART2->MODIR |= LPUART_MODIR_TXRTSPOL_MASK;
}


/*! @brief Initialize test pin P1_8 for ADC timing measurement */
void TEST_PIN_Init(void)
{
    gpio_pin_config_t pin_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0  /* Start with low level */
    };
    
    /* Configure P1_8 as GPIO output */
    PORT_SetPinMux(TEST_PIN_PORT, TEST_PIN_NUM, kPORT_MuxAsGpio);
    GPIO_PinInit(TEST_PIN_GPIO, TEST_PIN_NUM, &pin_config);
    
}

/*! @brief Configure OPAMP modules */
static void BOARD_InitOPAMP(void)
{
    opamp_config_t config;

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

/*${function:start}*/
void BOARD_InitHardware(void)
{
    BOARD_InitDEBUG_UARTPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    BOARD_InitOPAMP();
    TEST_PIN_Init();
    DWT_Init();
    BOARD_InitRS485_Tamagawa();
}

/*! @brief Set test pin to high level */
void TEST_PIN_Set(void)
{
    GPIO_PinWrite(TEST_PIN_GPIO, TEST_PIN_NUM, 1U);
}

/*! @brief Set test pin to low level */
void TEST_PIN_Clear(void)
{
    GPIO_PinWrite(TEST_PIN_GPIO, TEST_PIN_NUM, 0U);
}
