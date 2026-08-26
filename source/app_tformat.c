/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_tformat.h"

#include <string.h>

#include "fsl_clock.h"
#include "fsl_edma.h"
#include "fsl_edma_soc.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_port.h"
#include "fsl_reset.h"

#define TFORMAT_BAUD_RATE           2500000U
#define TFORMAT_ID0_CF              0x02U
#define TFORMAT_ID3_CF              0x1AU
#define TFORMAT_SF_VALID            0x00U
#define TFORMAT_SF_COUNTING_ERROR   0x10U
#define TFORMAT_ALMC_COUNTING_ERROR 0x04U
#define TFORMAT_ENID                0x10U
#define TFORMAT_ID0_FRAME_SIZE      6U
#define TFORMAT_ID3_FRAME_SIZE      11U
#define TFORMAT_DMA_CHANNEL         7U

#if defined(DEBUG)
#define TFORMAT_DIAG_CF             0xF1U
#define TFORMAT_DIAG_VERSION        1U
#define TFORMAT_DIAG_FRAME_SIZE     25U
#define TFORMAT_DIAG_UPDATE_SAMPLES 100U
#endif

#define TFORMAT_DIR_GPIO           GPIO3
#define TFORMAT_DIR_PORT           PORT3
#define TFORMAT_DIR_PIN            12U

volatile uint32_t tformat_id0_request_count;
volatile uint32_t tformat_id3_request_count;
volatile uint32_t tformat_diag_request_count;
volatile uint32_t tformat_response_count;
volatile uint32_t tformat_busy_count;
volatile uint32_t tformat_unsupported_count;
volatile uint32_t tformat_uart_error_count;

typedef struct _tformat_frame_set
{
    uint8_t id0[TFORMAT_ID0_FRAME_SIZE];
    uint8_t id3[TFORMAT_ID3_FRAME_SIZE];
} tformat_frame_set_t;

static tformat_frame_set_t s_frames[2];
static volatile uint8_t s_active_frame;
static volatile bool s_tx_busy;
static edma_handle_t s_tx_dma_handle;
static lpuart_edma_handle_t s_lpuart_edma_handle;

#if defined(DEBUG)
static uint8_t s_diag_frames[2][TFORMAT_DIAG_FRAME_SIZE];
static volatile uint8_t s_active_diag_frame;
static uint32_t s_diag_last_sample_count;
#endif

static uint8_t tformat_crc(const uint8_t *data, uint32_t size)
{
    uint8_t remainder = 0U;
    uint32_t byte_index;

    for (byte_index = 0U; byte_index < size; byte_index++)
    {
        uint32_t bit_index;

        remainder ^= data[byte_index];
        for (bit_index = 0U; bit_index < 8U; bit_index++)
        {
            remainder = (uint8_t)((remainder & 0x80U) != 0U ? (remainder << 1U) ^ 0x01U
                                                                  : remainder << 1U);
        }
    }

    return remainder;
}

#if defined(DEBUG)
static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static void write_float_le(uint8_t *data, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    write_u32_le(data, bits);
}

static void build_diag_frame(uint8_t *frame,
                             uint32_t encoder_status,
                             bool calibrated,
                             uint32_t calibration_source,
                             float mag16_raw,
                             float mag15_raw,
                             uint32_t adc_sample_count,
                             uint32_t adc_overrun_count)
{
    frame[0] = TFORMAT_DIAG_CF;
    frame[1] = TFORMAT_DIAG_VERSION;
    frame[2] = calibrated ? 1U : 0U;
    frame[3] = (uint8_t)calibration_source;
    write_u32_le(&frame[4], encoder_status);
    write_float_le(&frame[8], mag16_raw);
    write_float_le(&frame[12], mag15_raw);
    write_u32_le(&frame[16], adc_sample_count);
    write_u32_le(&frame[20], adc_overrun_count);
    frame[24] = tformat_crc(frame, TFORMAT_DIAG_FRAME_SIZE - 1U);
}
#endif

static void build_frames(tformat_frame_set_t *frames, uint32_t angle_counts, bool valid)
{
    uint16_t position = (uint16_t)angle_counts;
    uint8_t sf = valid ? TFORMAT_SF_VALID : TFORMAT_SF_COUNTING_ERROR;
    uint8_t almc = valid ? 0U : TFORMAT_ALMC_COUNTING_ERROR;

    frames->id0[0] = TFORMAT_ID0_CF;
    frames->id0[1] = sf;
    frames->id0[2] = (uint8_t)position;
    frames->id0[3] = (uint8_t)(position >> 8U);
    frames->id0[4] = 0U;
    frames->id0[5] = tformat_crc(frames->id0, TFORMAT_ID0_FRAME_SIZE - 1U);

    frames->id3[0] = TFORMAT_ID3_CF;
    frames->id3[1] = sf;
    frames->id3[2] = (uint8_t)position;
    frames->id3[3] = (uint8_t)(position >> 8U);
    frames->id3[4] = 0U;
    frames->id3[5] = TFORMAT_ENID;
    frames->id3[6] = 0U;
    frames->id3[7] = 0U;
    frames->id3[8] = 0U;
    frames->id3[9] = almc;
    frames->id3[10] = tformat_crc(frames->id3, TFORMAT_ID3_FRAME_SIZE - 1U);
}

void TFormat_Publish(uint32_t angle_counts, uint32_t encoder_status, bool calibrated)
{
    uint8_t next_frame = s_active_frame ^ 1U;
    bool valid = calibrated && (encoder_status == 0U);

    build_frames(&s_frames[next_frame], angle_counts, valid);
    __DMB();
    s_active_frame = next_frame;
}

void TFormat_PublishDiagnostics(uint32_t encoder_status,
                                bool calibrated,
                                uint32_t calibration_source,
                                float mag16_raw,
                                float mag15_raw,
                                uint32_t adc_sample_count,
                                uint32_t adc_overrun_count)
{
#if defined(DEBUG)
    uint8_t next_frame;

    if ((adc_sample_count - s_diag_last_sample_count) < TFORMAT_DIAG_UPDATE_SAMPLES)
    {
        return;
    }

    s_diag_last_sample_count = adc_sample_count;
    next_frame = s_active_diag_frame ^ 1U;
    build_diag_frame(s_diag_frames[next_frame],
                     encoder_status,
                     calibrated,
                     calibration_source,
                     mag16_raw,
                     mag15_raw,
                     adc_sample_count,
                     adc_overrun_count);
    __DMB();
    s_active_diag_frame = next_frame;
#else
    (void)encoder_status;
    (void)calibrated;
    (void)calibration_source;
    (void)mag16_raw;
    (void)mag15_raw;
    (void)adc_sample_count;
    (void)adc_overrun_count;
#endif
}

static void tformat_tx_callback(LPUART_Type *base,
                                lpuart_edma_handle_t *handle,
                                status_t status,
                                void *user_data)
{
    (void)base;
    (void)handle;
    (void)user_data;

    GPIO_PinWrite(TFORMAT_DIR_GPIO, TFORMAT_DIR_PIN, 0U);
    s_tx_busy = false;
    if (status == kStatus_LPUART_TxIdle)
    {
        tformat_response_count++;
    }
    else
    {
        tformat_uart_error_count++;
    }
}

static void send_response(uint8_t cf)
{
    tformat_frame_set_t *frames = &s_frames[s_active_frame];
    lpuart_transfer_t transfer;
    status_t status;

    if (cf == TFORMAT_ID0_CF)
    {
        transfer.data = frames->id0;
        transfer.dataSize = TFORMAT_ID0_FRAME_SIZE;
    }
    else
    {
#if defined(DEBUG)
        if (cf == TFORMAT_DIAG_CF)
        {
            transfer.data = s_diag_frames[s_active_diag_frame];
            transfer.dataSize = TFORMAT_DIAG_FRAME_SIZE;
        }
        else
#endif
        {
            transfer.data = frames->id3;
            transfer.dataSize = TFORMAT_ID3_FRAME_SIZE;
        }
    }

    s_tx_busy = true;
    GPIO_PinWrite(TFORMAT_DIR_GPIO, TFORMAT_DIR_PIN, 1U);
    status = LPUART_SendEDMA(LPUART2, &s_lpuart_edma_handle, &transfer);
    if (status != kStatus_Success)
    {
        GPIO_PinWrite(TFORMAT_DIR_GPIO, TFORMAT_DIR_PIN, 0U);
        s_tx_busy = false;
        tformat_uart_error_count++;
    }
}

void TFormat_Init(void)
{
    const gpio_pin_config_t direction_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U,
    };
    lpuart_config_t lpuart_config;
    edma_config_t edma_config;

    build_frames(&s_frames[0], 0U, false);
    build_frames(&s_frames[1], 0U, false);

#if defined(DEBUG)
    build_diag_frame(s_diag_frames[0], 0U, false, 0U, 0.0f, 0.0f, 0U, 0U);
    build_diag_frame(s_diag_frames[1], 0U, false, 0U, 0.0f, 0.0f, 0U, 0U);
#endif

    PORT_SetPinMux(TFORMAT_DIR_PORT, TFORMAT_DIR_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(TFORMAT_DIR_GPIO, TFORMAT_DIR_PIN, &direction_config);

    CLOCK_SetClockDiv(kCLOCK_DivLPUART2, 1U);
    CLOCK_AttachClk(kFRO_HF_DIV_to_LPUART2);
    RESET_PeripheralReset(kLPUART2_RST_SHIFT_RSTn);

    LPUART_GetDefaultConfig(&lpuart_config);
    lpuart_config.baudRate_Bps = TFORMAT_BAUD_RATE;
    lpuart_config.enableTx = true;
    lpuart_config.enableRx = true;
    (void)LPUART_Init(LPUART2, &lpuart_config, CLOCK_GetFreq(kCLOCK_FroHfDiv));

    EDMA_GetDefaultConfig(&edma_config);
    EDMA_Init(DMA0, &edma_config);
    EDMA_SetChannelMux(DMA0, TFORMAT_DMA_CHANNEL, kDma0RequestLPUART2Tx);

    NVIC_SetPriority(DMA_CH7_IRQn, 0U);
    NVIC_SetPriority(LPUART2_IRQn, 0U);
    EDMA_CreateHandle(&s_tx_dma_handle, DMA0, TFORMAT_DMA_CHANNEL);
    LPUART_TransferCreateHandleEDMA(LPUART2,
                                    &s_lpuart_edma_handle,
                                    tformat_tx_callback,
                                    NULL,
                                    &s_tx_dma_handle,
                                    NULL);
    LPUART_EnableInterrupts(LPUART2, (uint32_t)kLPUART_RxDataRegFullInterruptEnable);
}

void LPUART2_IRQHandler(void)
{
    uint32_t status = LPUART_GetStatusFlags(LPUART2);
    uint32_t enabled_interrupts = LPUART_GetEnabledInterrupts(LPUART2);
    const uint32_t error_flags = (uint32_t)kLPUART_RxOverrunFlag | (uint32_t)kLPUART_NoiseErrorFlag |
                                 (uint32_t)kLPUART_FramingErrorFlag | (uint32_t)kLPUART_ParityErrorFlag;

    if (((status & (uint32_t)kLPUART_TransmissionCompleteFlag) != 0U) &&
        ((enabled_interrupts & (uint32_t)kLPUART_TransmissionCompleteInterruptEnable) != 0U))
    {
        LPUART_TransferEdmaHandleIRQ(LPUART2, &s_lpuart_edma_handle);
    }

    if ((status & error_flags) != 0U)
    {
        tformat_uart_error_count++;
        (void)LPUART_ClearStatusFlags(LPUART2, status & error_flags);
    }

    while ((LPUART_GetStatusFlags(LPUART2) & (uint32_t)kLPUART_RxDataRegFullFlag) != 0U)
    {
        uint8_t cf = LPUART_ReadByte(LPUART2);
        bool supported = (cf == TFORMAT_ID0_CF) || (cf == TFORMAT_ID3_CF);

#if defined(DEBUG)
        supported = supported || (cf == TFORMAT_DIAG_CF);
#endif

        if (!supported)
        {
            tformat_unsupported_count++;
        }
        else
        {
            if (cf == TFORMAT_ID0_CF)
            {
                tformat_id0_request_count++;
            }
            else
            {
#if defined(DEBUG)
                if (cf == TFORMAT_DIAG_CF)
                {
                    tformat_diag_request_count++;
                }
                else
#endif
                {
                    tformat_id3_request_count++;
                }
            }

            if (s_tx_busy)
            {
                tformat_busy_count++;
            }
            else
            {
                send_response(cf);
            }
        }
    }

    SDK_ISR_EXIT_BARRIER;
}
