/*
 * Copyright 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "app_tformat.h"

#include "app_adc.h"

#include <string.h>

#include "fsl_clock.h"
#include "fsl_edma.h"
#include "fsl_edma_soc.h"
#include "fsl_gpio.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_port.h"
#include "fsl_reset.h"

extern uint32_t SystemCoreClock;

/* Public T-Format values used by the NXP and TI reference implementations. */
#define TFORMAT_CF_ID0 0x02U
#define TFORMAT_CF_ID1 0x8AU
#define TFORMAT_CF_ID2 0x92U
#define TFORMAT_CF_ID3 0x1AU
#define TFORMAT_CF_ID6 0x32U
#define TFORMAT_CF_ID7 0xBAU
#define TFORMAT_CF_ID8 0xC2U
#define TFORMAT_CF_IDC 0x62U
#define TFORMAT_CF_IDD 0xEAU

#define TFORMAT_LEN_POSITION 6U
#define TFORMAT_LEN_ENID     4U
#define TFORMAT_LEN_ID3      11U
#define TFORMAT_LEN_EEPROM   4U
#define TFORMAT_LEN_MAX      TFORMAT_LEN_ID3

#define TFORMAT_ENID               0x10U
#define TFORMAT_SF_COUNTING_ERROR  0x10U
#define TFORMAT_ADF_ADDRESS_MASK   0x7FU
#define TFORMAT_ADF_BUSY           0x80U

#define TFORMAT_ALMC_OVERSPEED   0x01U
#define TFORMAT_ALMC_COUNT_ERROR 0x04U

#define TFORMAT_BAUD_RATE   2500000U
#define TFORMAT_DMA_CHANNEL 7U

#define TFORMAT_DIR_GPIO GPIO3
#define TFORMAT_DIR_PORT PORT3
#define TFORMAT_DIR_PIN  12U

#define TFORMAT_FRAME_GAP_US         12U
#define TFORMAT_MAX_POSITION_AGE_US  (3000000U / ADC_SAMPLE_RATE_HZ)
#define TFORMAT_OVERSPEED_DPS        (6000.0f * 6.0f)
#define TFORMAT_VELOCITY_Q           20U

volatile uint32_t tformat_id0_request_count;
volatile uint32_t tformat_id3_request_count;
volatile uint32_t tformat_response_count;
volatile uint32_t tformat_busy_count;
volatile uint32_t tformat_unsupported_count;
volatile uint32_t tformat_uart_error_count;
volatile uint32_t tformat_stale_count;
volatile uint32_t tformat_crc_error_count;
volatile uint32_t tformat_desync_count;
volatile uint32_t tformat_eeprom_write_count;
volatile uint32_t tformat_eeprom_error_count;

typedef struct _tformat_snapshot
{
    uint32_t counts;
    int32_t velocity_counts_per_cycle_q20;
    uint32_t status;
    uint32_t cycle;
    bool ready;
    bool overspeed;
} tformat_snapshot_t;

static tformat_snapshot_t s_snapshot[2];
static volatile uint8_t s_active_snapshot;

static uint8_t s_tx_frame[TFORMAT_LEN_MAX];
static volatile bool s_tx_busy;
static edma_handle_t s_tx_dma_handle;
static lpuart_edma_handle_t s_lpuart_edma_handle;

static uint8_t s_eeprom[TFORMAT_EEPROM_SIZE];
static volatile bool s_eeprom_busy;
static uint8_t s_eeprom_pending_address;
static uint8_t s_eeprom_previous_value;

static uint8_t s_rx_frame[4];
static uint8_t s_rx_len;
static uint32_t s_rx_last_cycle;

static volatile uint32_t s_reset_requests;
static volatile bool s_position_fresh;

static uint32_t s_gap_cycles;
static uint32_t s_stale_cycles;
static float s_velocity_scale;

static void discard_until_gap(void)
{
    s_rx_len = (uint8_t)sizeof(s_rx_frame);
}

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
            if ((remainder & 0x80U) != 0U)
            {
                remainder = (uint8_t)((remainder << 1U) ^ 0x01U);
            }
            else
            {
                remainder = (uint8_t)(remainder << 1U);
            }
        }
    }

    return remainder;
}

void TFormat_Publish(const encoder_result_t *result, bool ready)
{
    const uint8_t next = s_active_snapshot ^ 1U;
    tformat_snapshot_t *snapshot = &s_snapshot[next];
    const float velocity_q20 = result->angular_velocity_dps * s_velocity_scale;

    snapshot->counts = result->angle_counts;
    snapshot->velocity_counts_per_cycle_q20 =
        (int32_t)(velocity_q20 + ((velocity_q20 >= 0.0f) ? 0.5f : -0.5f));
    snapshot->status = result->status;
    snapshot->ready = ready;
    snapshot->overspeed = (result->angular_velocity_dps > TFORMAT_OVERSPEED_DPS) ||
                          (result->angular_velocity_dps < -TFORMAT_OVERSPEED_DPS);
    snapshot->cycle = DWT->CYCCNT;
    __DMB();
    s_active_snapshot = next;
    s_position_fresh = true;
}

static uint32_t extrapolate(const tformat_snapshot_t *snapshot, uint32_t age_cycles)
{
    const int64_t product =
        (int64_t)snapshot->velocity_counts_per_cycle_q20 * (int64_t)age_cycles;
    const int32_t delta = (int32_t)(product / (int64_t)(1UL << TFORMAT_VELOCITY_Q));
    int32_t counts = (int32_t)snapshot->counts + delta;

    counts %= (int32_t)ENCODER_COUNTS_PER_REV;
    if (counts < 0)
    {
        counts += (int32_t)ENCODER_COUNTS_PER_REV;
    }

    return (uint32_t)counts;
}

static uint8_t compute_almc(const tformat_snapshot_t *snapshot, uint32_t age_cycles)
{
    uint8_t almc = 0U;

    if (age_cycles > s_stale_cycles)
    {
        s_position_fresh = false;
    }

    if (!s_position_fresh)
    {
        tformat_stale_count++;
        almc |= TFORMAT_ALMC_COUNT_ERROR;
    }

    if (!snapshot->ready ||
        ((snapshot->status & ENCODER_STATUS_POSITION_INVALID_MASK) != 0U))
    {
        almc |= TFORMAT_ALMC_COUNT_ERROR;
    }

    if (snapshot->overspeed)
    {
        almc |= TFORMAT_ALMC_OVERSPEED;
    }

    return almc;
}

static void fill_position(uint8_t offset, uint32_t position)
{
    s_tx_frame[offset] = (uint8_t)position;
    s_tx_frame[offset + 1U] = (uint8_t)(position >> 8U);
    s_tx_frame[offset + 2U] = 0U;
}

static uint8_t build_response(uint8_t cf, const uint8_t *request)
{
    const tformat_snapshot_t *snapshot = &s_snapshot[s_active_snapshot];
    const uint32_t age_cycles = DWT->CYCCNT - snapshot->cycle;
    const uint8_t almc = compute_almc(snapshot, age_cycles);
    const uint8_t sf = ((almc & TFORMAT_ALMC_COUNT_ERROR) != 0U) ?
                           TFORMAT_SF_COUNTING_ERROR : 0U;
    const uint32_t position = ((almc & TFORMAT_ALMC_COUNT_ERROR) == 0U) ?
                                  extrapolate(snapshot, age_cycles) :
                                  snapshot->counts;
    uint8_t length;

    s_tx_frame[0] = cf;

    switch (cf)
    {
        case TFORMAT_CF_ID1:
            s_tx_frame[1] = sf;
            s_tx_frame[2] = 0U;
            s_tx_frame[3] = 0U;
            s_tx_frame[4] = 0U;
            length = TFORMAT_LEN_POSITION;
            break;

        case TFORMAT_CF_ID2:
            s_tx_frame[1] = sf;
            s_tx_frame[2] = TFORMAT_ENID;
            length = TFORMAT_LEN_ENID;
            break;

        case TFORMAT_CF_ID3:
            s_tx_frame[1] = sf;
            fill_position(2U, position);
            s_tx_frame[5] = TFORMAT_ENID;
            s_tx_frame[6] = 0U;
            s_tx_frame[7] = 0U;
            s_tx_frame[8] = 0U;
            s_tx_frame[9] = almc;
            length = TFORMAT_LEN_ID3;
            break;

        case TFORMAT_CF_ID6:
        {
            const uint8_t address = request[1] & TFORMAT_ADF_ADDRESS_MASK;

            if (!s_eeprom_busy)
            {
                s_eeprom_pending_address = address;
                s_eeprom_previous_value = s_eeprom[address];
                s_eeprom[address] = request[2];
                s_eeprom_busy = true;
                tformat_eeprom_write_count++;
            }
            s_tx_frame[1] = address | TFORMAT_ADF_BUSY;
            s_tx_frame[2] = request[2];
            length = TFORMAT_LEN_EEPROM;
            break;
        }

        case TFORMAT_CF_IDD:
        {
            const uint8_t address = request[1] & TFORMAT_ADF_ADDRESS_MASK;

            s_tx_frame[1] = address | (s_eeprom_busy ? TFORMAT_ADF_BUSY : 0U);
            s_tx_frame[2] = s_eeprom[address];
            length = TFORMAT_LEN_EEPROM;
            break;
        }

        case TFORMAT_CF_ID7:
            s_reset_requests |= TFORMAT_RESET_ERROR;
            s_tx_frame[1] = sf;
            fill_position(2U, position);
            length = TFORMAT_LEN_POSITION;
            break;

        case TFORMAT_CF_ID8:
            s_reset_requests |= TFORMAT_RESET_POSITION;
            s_tx_frame[1] = sf;
            fill_position(2U, position);
            length = TFORMAT_LEN_POSITION;
            break;

        case TFORMAT_CF_IDC:
            s_reset_requests |= TFORMAT_RESET_MULTITURN;
            s_tx_frame[1] = sf;
            fill_position(2U, position);
            length = TFORMAT_LEN_POSITION;
            break;

        default:
            s_tx_frame[1] = sf;
            fill_position(2U, position);
            length = TFORMAT_LEN_POSITION;
            break;
    }

    s_tx_frame[length - 1U] = tformat_crc(s_tx_frame, length - 1U);
    return length;
}

static uint8_t request_length(uint8_t cf)
{
    switch (cf)
    {
        case TFORMAT_CF_ID0:
        case TFORMAT_CF_ID1:
        case TFORMAT_CF_ID2:
        case TFORMAT_CF_ID3:
        case TFORMAT_CF_ID7:
        case TFORMAT_CF_ID8:
        case TFORMAT_CF_IDC:
            return 1U;

        case TFORMAT_CF_IDD:
            return 3U;

        case TFORMAT_CF_ID6:
            return 4U;

        default:
            return 0U;
    }
}

static void count_request(uint8_t cf)
{
    if (cf == TFORMAT_CF_ID0)
    {
        tformat_id0_request_count++;
    }
    else if (cf == TFORMAT_CF_ID3)
    {
        tformat_id3_request_count++;
    }
}

uint32_t TFormat_TakeResetRequests(void)
{
    const uint32_t irq_mask = DisableGlobalIRQ();
    const uint32_t requests = s_reset_requests;

    s_reset_requests = 0U;
    EnableGlobalIRQ(irq_mask);
    return requests;
}

void TFormat_LoadEeprom(const uint8_t data[TFORMAT_EEPROM_SIZE])
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    memcpy(s_eeprom, data, sizeof(s_eeprom));
    s_eeprom_busy = false;
    EnableGlobalIRQ(irq_mask);
}

void TFormat_CopyEeprom(uint8_t data[TFORMAT_EEPROM_SIZE])
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    memcpy(data, s_eeprom, sizeof(s_eeprom));
    EnableGlobalIRQ(irq_mask);
}

bool TFormat_EepromWritePending(void)
{
    return s_eeprom_busy;
}

void TFormat_CompleteEepromWrite(bool saved)
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    if (s_eeprom_busy)
    {
        if (!saved)
        {
            s_eeprom[s_eeprom_pending_address] = s_eeprom_previous_value;
            tformat_eeprom_error_count++;
        }
        s_eeprom_busy = false;
    }
    EnableGlobalIRQ(irq_mask);
}

bool TFormat_StorageBegin(void)
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    if (s_tx_busy)
    {
        EnableGlobalIRQ(irq_mask);
        return false;
    }

    LPUART_DisableInterrupts(LPUART2, (uint32_t)kLPUART_RxDataRegFullInterruptEnable);
    while ((LPUART_GetStatusFlags(LPUART2) & (uint32_t)kLPUART_RxDataRegFullFlag) != 0U)
    {
        (void)LPUART_ReadByte(LPUART2);
    }
    discard_until_gap();
    EnableGlobalIRQ(irq_mask);
    return true;
}

void TFormat_StorageEnd(void)
{
    const uint32_t irq_mask = DisableGlobalIRQ();

    while ((LPUART_GetStatusFlags(LPUART2) & (uint32_t)kLPUART_RxDataRegFullFlag) != 0U)
    {
        (void)LPUART_ReadByte(LPUART2);
    }
    s_rx_last_cycle = DWT->CYCCNT;
    discard_until_gap();
    LPUART_EnableInterrupts(LPUART2, (uint32_t)kLPUART_RxDataRegFullInterruptEnable);
    EnableGlobalIRQ(irq_mask);
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
    discard_until_gap();

    if (status == kStatus_LPUART_TxIdle)
    {
        tformat_response_count++;
    }
    else
    {
        tformat_uart_error_count++;
    }
}

static void send_response(uint8_t cf, const uint8_t *request)
{
    lpuart_transfer_t transfer;

    transfer.data = s_tx_frame;
    transfer.dataSize = build_response(cf, request);

    s_tx_busy = true;
    GPIO_PinWrite(TFORMAT_DIR_GPIO, TFORMAT_DIR_PIN, 1U);
    if (LPUART_SendEDMA(LPUART2, &s_lpuart_edma_handle, &transfer) != kStatus_Success)
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

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_gap_cycles = (SystemCoreClock / 1000000U) * TFORMAT_FRAME_GAP_US;
    s_stale_cycles = (SystemCoreClock / 1000000U) * TFORMAT_MAX_POSITION_AGE_US;
    s_velocity_scale = ((float)ENCODER_COUNTS_PER_REV * (float)(1UL << TFORMAT_VELOCITY_Q)) /
                       (360.0f * (float)SystemCoreClock);
    memset(s_eeprom, 0xFF, sizeof(s_eeprom));

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

static void receive_byte(uint8_t byte)
{
    const uint32_t now = DWT->CYCCNT;
    const bool gap = (now - s_rx_last_cycle) > s_gap_cycles;
    uint8_t expected;

    s_rx_last_cycle = now;
    if (gap)
    {
        s_rx_len = 0U;
    }

    if (s_rx_len >= sizeof(s_rx_frame))
    {
        tformat_desync_count++;
        return;
    }

    s_rx_frame[s_rx_len++] = byte;
    expected = request_length(s_rx_frame[0]);
    if (expected == 0U)
    {
        tformat_unsupported_count++;
        discard_until_gap();
        return;
    }

    if (s_rx_len < expected)
    {
        return;
    }

    if ((expected > 1U) &&
        (tformat_crc(s_rx_frame, expected - 1U) != s_rx_frame[expected - 1U]))
    {
        tformat_crc_error_count++;
        discard_until_gap();
        return;
    }

    count_request(s_rx_frame[0]);
    if (s_tx_busy)
    {
        tformat_busy_count++;
    }
    else
    {
        send_response(s_rx_frame[0], s_rx_frame);
    }
    discard_until_gap();
}

void LPUART2_IRQHandler(void)
{
    const uint32_t status = LPUART_GetStatusFlags(LPUART2);
    const uint32_t enabled_interrupts = LPUART_GetEnabledInterrupts(LPUART2);
    const uint32_t error_flags = (uint32_t)kLPUART_RxOverrunFlag |
                                 (uint32_t)kLPUART_NoiseErrorFlag |
                                 (uint32_t)kLPUART_FramingErrorFlag |
                                 (uint32_t)kLPUART_ParityErrorFlag;

    if (((status & (uint32_t)kLPUART_TransmissionCompleteFlag) != 0U) &&
        ((enabled_interrupts & (uint32_t)kLPUART_TransmissionCompleteInterruptEnable) != 0U))
    {
        LPUART_TransferEdmaHandleIRQ(LPUART2, &s_lpuart_edma_handle);
    }

    if ((status & error_flags) != 0U)
    {
        tformat_uart_error_count++;
        (void)LPUART_ClearStatusFlags(LPUART2, status & error_flags);
        discard_until_gap();
        while ((LPUART_GetStatusFlags(LPUART2) & (uint32_t)kLPUART_RxDataRegFullFlag) != 0U)
        {
            (void)LPUART_ReadByte(LPUART2);
        }
        SDK_ISR_EXIT_BARRIER;
        return;
    }

    while ((LPUART_GetStatusFlags(LPUART2) & (uint32_t)kLPUART_RxDataRegFullFlag) != 0U)
    {
        receive_byte(LPUART_ReadByte(LPUART2));
    }

    SDK_ISR_EXIT_BARRIER;
}
