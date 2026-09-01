/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "mcu_stub.h"

#include <string.h>

stub_dwt_t g_stub_dwt;
stub_coredebug_t g_stub_coredebug;
uint32_t SystemCoreClock = 180000000U;

static GPIO_Type s_gpio3;
static PORT_Type s_port3;
static DMA_Type s_dma0;
static LPUART_Type s_lpuart2;

GPIO_Type *const GPIO3 = &s_gpio3;
PORT_Type *const PORT3 = &s_port3;
DMA_Type *const DMA0 = &s_dma0;
LPUART_Type *const LPUART2 = &s_lpuart2;

uint8_t g_stub_tx[32];
uint32_t g_stub_tx_len;
uint32_t g_stub_tx_count;
uint32_t g_stub_uart_status;
bool g_stub_tx_fail;

uint32_t DisableGlobalIRQ(void) { return 0U; }
void EnableGlobalIRQ(uint32_t mask) { (void)mask; }

void GPIO_PinInit(GPIO_Type *base, uint32_t pin, const gpio_pin_config_t *config)
{
    (void)base; (void)pin; (void)config;
}

void GPIO_PinWrite(GPIO_Type *base, uint32_t pin, uint8_t value)
{
    (void)base; (void)pin; (void)value;
}

void PORT_SetPinMux(PORT_Type *base, uint32_t pin, int mux)
{
    (void)base; (void)pin; (void)mux;
}

void CLOCK_SetClockDiv(int name, uint32_t div) { (void)name; (void)div; }
void CLOCK_AttachClk(int attach) { (void)attach; }
uint32_t CLOCK_GetFreq(int name) { (void)name; return SystemCoreClock; }
void RESET_PeripheralReset(int reset) { (void)reset; }

void EDMA_GetDefaultConfig(edma_config_t *config) { (void)config; }
void EDMA_Init(DMA_Type *base, const edma_config_t *config) { (void)base; (void)config; }
void EDMA_SetChannelMux(DMA_Type *base, uint32_t channel, int source)
{
    (void)base; (void)channel; (void)source;
}
void EDMA_CreateHandle(edma_handle_t *handle, DMA_Type *base, uint32_t channel)
{
    (void)handle; (void)base; (void)channel;
}

void LPUART_GetDefaultConfig(lpuart_config_t *config)
{
    memset(config, 0, sizeof(*config));
}

status_t LPUART_Init(LPUART_Type *base, const lpuart_config_t *config, uint32_t srcClock_Hz)
{
    (void)base; (void)config; (void)srcClock_Hz;
    return kStatus_Success;
}

void LPUART_EnableInterrupts(LPUART_Type *base, uint32_t mask) { (void)base; (void)mask; }
void LPUART_DisableInterrupts(LPUART_Type *base, uint32_t mask) { (void)base; (void)mask; }
uint32_t LPUART_GetStatusFlags(LPUART_Type *base) { (void)base; return g_stub_uart_status; }
uint32_t LPUART_GetEnabledInterrupts(LPUART_Type *base) { (void)base; return 0U; }
status_t LPUART_ClearStatusFlags(LPUART_Type *base, uint32_t mask)
{
    (void)base;
    g_stub_uart_status &= ~mask;
    return kStatus_Success;
}
uint8_t LPUART_ReadByte(LPUART_Type *base) { (void)base; return 0U; }

void LPUART_TransferCreateHandleEDMA(LPUART_Type *base,
                                     lpuart_edma_handle_t *handle,
                                     lpuart_edma_transfer_callback_t callback,
                                     void *userData,
                                     edma_handle_t *txEdmaHandle,
                                     edma_handle_t *rxEdmaHandle)
{
    (void)base; (void)txEdmaHandle; (void)rxEdmaHandle;
    handle->callback = callback;
    handle->userData = userData;
}

status_t LPUART_SendEDMA(LPUART_Type *base, lpuart_edma_handle_t *handle, lpuart_transfer_t *xfer)
{
    (void)base; (void)handle;

    if (g_stub_tx_fail)
    {
        return kStatus_Success + 1;
    }

    g_stub_tx_len = (uint32_t)xfer->dataSize;
    memcpy(g_stub_tx, xfer->data, xfer->dataSize);
    g_stub_tx_count++;

    return kStatus_Success;
}

void LPUART_TransferEdmaHandleIRQ(LPUART_Type *base, lpuart_edma_handle_t *handle)
{
    (void)base; (void)handle;
}
