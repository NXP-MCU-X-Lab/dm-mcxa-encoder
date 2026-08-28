/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Just enough of the MCU headers for app_tformat.c to compile on the host, so
 * the protocol layer can be exercised the same way app_encoder.c already is.
 * Nothing here models hardware behaviour -- the test drives the state machine
 * directly and reads back what would have been transmitted.
 */

#ifndef MCU_STUB_H_
#define MCU_STUB_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* --- core --- */

typedef struct _stub_dwt
{
    uint32_t CYCCNT;
    uint32_t CTRL;
} stub_dwt_t;

typedef struct _stub_coredebug
{
    uint32_t DEMCR;
} stub_coredebug_t;

extern stub_dwt_t g_stub_dwt;
extern stub_coredebug_t g_stub_coredebug;
extern uint32_t SystemCoreClock;

#define DWT                        (&g_stub_dwt)
#define CoreDebug                  (&g_stub_coredebug)
#define DWT_CTRL_CYCCNTENA_Msk     (1UL)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)

#define __DMB()             ((void)0)
#define SDK_ISR_EXIT_BARRIER ((void)0)

uint32_t DisableGlobalIRQ(void);
void EnableGlobalIRQ(uint32_t mask);

typedef int32_t status_t;
#define kStatus_Success        (0)
#define kStatus_LPUART_TxIdle  (1)

typedef int stub_irqn_t;
#define DMA_CH7_IRQn  (7)
#define LPUART2_IRQn  (8)
#define NVIC_SetPriority(irq, prio) ((void)(irq), (void)(prio))

/* --- port / gpio --- */

typedef struct _stub_gpio { int id; } GPIO_Type;
typedef struct _stub_port { int id; } PORT_Type;

extern GPIO_Type *const GPIO3;
extern PORT_Type *const PORT3;

typedef enum _gpio_pin_direction { kGPIO_DigitalInput = 0, kGPIO_DigitalOutput = 1 } gpio_pin_direction_t;
typedef struct _gpio_pin_config
{
    gpio_pin_direction_t pinDirection;
    uint8_t outputLogic;
} gpio_pin_config_t;

#define kPORT_MuxAsGpio (1)

void GPIO_PinInit(GPIO_Type *base, uint32_t pin, const gpio_pin_config_t *config);
void GPIO_PinWrite(GPIO_Type *base, uint32_t pin, uint8_t value);
void PORT_SetPinMux(PORT_Type *base, uint32_t pin, int mux);

/* --- clock / reset --- */

#define kCLOCK_DivLPUART2       (0)
#define kFRO_HF_DIV_to_LPUART2  (0)
#define kCLOCK_FroHfDiv         (0)
#define kLPUART2_RST_SHIFT_RSTn (0)

void CLOCK_SetClockDiv(int name, uint32_t div);
void CLOCK_AttachClk(int attach);
uint32_t CLOCK_GetFreq(int name);
void RESET_PeripheralReset(int reset);

/* --- edma --- */

typedef struct _stub_dma { int id; } DMA_Type;
extern DMA_Type *const DMA0;
#define kDma0RequestLPUART2Tx (0)

typedef struct _edma_config { int unused; } edma_config_t;
typedef struct _edma_handle { int unused; } edma_handle_t;

void EDMA_GetDefaultConfig(edma_config_t *config);
void EDMA_Init(DMA_Type *base, const edma_config_t *config);
void EDMA_SetChannelMux(DMA_Type *base, uint32_t channel, int source);
void EDMA_CreateHandle(edma_handle_t *handle, DMA_Type *base, uint32_t channel);

/* --- lpuart --- */

typedef struct _stub_lpuart { int id; } LPUART_Type;
extern LPUART_Type *const LPUART2;

typedef struct _lpuart_config
{
    uint32_t baudRate_Bps;
    bool enableTx;
    bool enableRx;
} lpuart_config_t;

typedef struct _lpuart_transfer
{
    uint8_t *data;
    size_t dataSize;
} lpuart_transfer_t;

struct _lpuart_edma_handle;
typedef void (*lpuart_edma_transfer_callback_t)(LPUART_Type *base,
                                                struct _lpuart_edma_handle *handle,
                                                status_t status,
                                                void *userData);

typedef struct _lpuart_edma_handle
{
    lpuart_edma_transfer_callback_t callback;
    void *userData;
} lpuart_edma_handle_t;

#define kLPUART_RxDataRegFullInterruptEnable        (1UL << 0)
#define kLPUART_TransmissionCompleteInterruptEnable (1UL << 1)
#define kLPUART_RxDataRegFullFlag                   (1UL << 2)
#define kLPUART_TransmissionCompleteFlag            (1UL << 3)
#define kLPUART_RxOverrunFlag                       (1UL << 4)
#define kLPUART_NoiseErrorFlag                      (1UL << 5)
#define kLPUART_FramingErrorFlag                    (1UL << 6)
#define kLPUART_ParityErrorFlag                     (1UL << 7)

void LPUART_GetDefaultConfig(lpuart_config_t *config);
status_t LPUART_Init(LPUART_Type *base, const lpuart_config_t *config, uint32_t srcClock_Hz);
void LPUART_EnableInterrupts(LPUART_Type *base, uint32_t mask);
void LPUART_DisableInterrupts(LPUART_Type *base, uint32_t mask);
uint32_t LPUART_GetStatusFlags(LPUART_Type *base);
uint32_t LPUART_GetEnabledInterrupts(LPUART_Type *base);
status_t LPUART_ClearStatusFlags(LPUART_Type *base, uint32_t mask);
uint8_t LPUART_ReadByte(LPUART_Type *base);
void LPUART_TransferCreateHandleEDMA(LPUART_Type *base,
                                     lpuart_edma_handle_t *handle,
                                     lpuart_edma_transfer_callback_t callback,
                                     void *userData,
                                     edma_handle_t *txEdmaHandle,
                                     edma_handle_t *rxEdmaHandle);
status_t LPUART_SendEDMA(LPUART_Type *base, lpuart_edma_handle_t *handle, lpuart_transfer_t *xfer);
void LPUART_TransferEdmaHandleIRQ(LPUART_Type *base, lpuart_edma_handle_t *handle);

/* What the stub captured from the last LPUART_SendEDMA. */
extern uint8_t g_stub_tx[32];
extern uint32_t g_stub_tx_len;
extern uint32_t g_stub_tx_count;

/* Set to make LPUART_SendEDMA report failure, which is the one path where the
 * driver clears its busy flag without a transfer ever completing. */
extern bool g_stub_tx_fail;

#endif /* MCU_STUB_H_ */
