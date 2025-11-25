/*
 * Tamagawa T-Format minimal driver using LPUART2 + eDMA (DMA0 CH7 TX, CH6 RX)
 * Implements:
 *  - ABS read (CF ID0: 0x02)
 *  - Combined read (CF ID3: 0x1A)
 *  - EEPROM read (CF IDD: 0xEA) for temperature
 *  Notes:
 *  - CRC mode supports simple XOR or CRC-8(0x07). Verify with your encoder datasheet.
 *  - Transaction baud switches LPUART2 to 2.5Mbps during operation, then restores previous baud.
 */

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_edma.h"
#include "app_tformat.h"
#include "app_sampler.h"
#include "app_encoder.h"

#include <stdio.h>

#define TFORMAT_UART                LPUART2
#define TFORMAT_DMA                 DMA0
#define TFORMAT_DMA_TX_CHANNEL      (7U)
#define TFORMAT_DMA_RX_CHANNEL      (6U)

/* CF codes (per commonly referenced mapping) */
#define CF_ID0_ABS                  (0x02U)
#define CF_ID3_ALL                  (0x1AU)
#define CF_IDD_EEPROM_READ          (0xEAU)

/* Expected response lengths */
#define RESP_LEN_ID0                (1U /*CF*/ + 1U /*SF*/ + 3U /*ABS0..2*/ + 1U /*CRC*/)  /* 6 bytes */
#define RESP_LEN_ID3                (1U /*CF*/ + 1U /*SF*/ + 8U /*ABS0..2,ENID,ABM0..2,ALMC*/ + 1U /*CRC*/) /* 11 bytes */
#define RESP_LEN_IDD                (1U /*CF*/ + 1U /*ADF*/ + 1U /*EDF*/ + 1U /*CRC*/)     /* 4 bytes */

static edma_handle_t s_tfEdmaTxHandle;
static edma_handle_t s_tfEdmaRxHandle;
static lpuart_edma_handle_t s_tfLpuartEdmaHandle;
static volatile bool s_tfRxDone = false;

/* Default temperature EEPROM address (ADF); adjust if needed */
static const uint8_t s_tempAdfAddr = 0xE1U;

static void tformat_edma_cb(LPUART_Type *base,
                            lpuart_edma_handle_t *handle,
                            status_t status,
                            void *userData)
{
    (void)base; (void)handle; (void)userData;

    if (status == kStatus_LPUART_RxIdle)
    {
        s_tfRxDone = true;
    }
}

static void tformat_set_baud(uint32_t baud)
{
    uint32_t src = CLOCK_GetFreq(kCLOCK_FroHfDiv);
    LPUART_SetBaudRate(TFORMAT_UART, baud, src);
}

static uint8_t tformat_calc_crc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0U;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

static status_t tformat_rx_bytes(uint8_t *rx, size_t rxLen)
{
    lpuart_transfer_t t;
    status_t st;
    s_tfRxDone = false;
    t.data = rx; t.dataSize = rxLen;
    st = LPUART_ReceiveEDMA(TFORMAT_UART, &s_tfLpuartEdmaHandle, &t);
    if (st != kStatus_Success) return st;
    while (!s_tfRxDone)
    {
        __NOP();
        __NOP();
        __NOP();
    }
    return kStatus_Success;
}

static status_t tformat_tx_bytes(const uint8_t *tx, size_t txLen)
{
    lpuart_transfer_t t;
    status_t st;
    t.data = (uint8_t *)tx; t.dataSize = txLen;
    
  //  SDK_DelayAtLeastUs(100, CLOCK_GetFreq(kCLOCK_CoreSysClk));
    
    st = LPUART_SendEDMA(TFORMAT_UART, &s_tfLpuartEdmaHandle, &t);
    if (st != kStatus_Success) return st;
    for (;;)
    {
        uint32_t flags = EDMA_GetChannelStatusFlags(TFORMAT_DMA, TFORMAT_DMA_TX_CHANNEL);
        if ((flags & (uint32_t)kEDMA_DoneFlag) != 0U)
        {
            EDMA_ClearChannelStatusFlags(TFORMAT_DMA, TFORMAT_DMA_TX_CHANNEL, (uint32_t)kEDMA_DoneFlag);
            break;
        }
        __NOP();
        __NOP();
        __NOP();
    }
    return kStatus_Success;
}

void tformat_init(void)
{
    /* Switch to 2.5 Mbps for T-Format transactions */
    tformat_set_baud(2000000U);
    
    edma_config_t dmaConfig;
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(TFORMAT_DMA, &dmaConfig);

    EDMA_CreateHandle(&s_tfEdmaTxHandle, TFORMAT_DMA, TFORMAT_DMA_TX_CHANNEL);
    EDMA_CreateHandle(&s_tfEdmaRxHandle, TFORMAT_DMA, TFORMAT_DMA_RX_CHANNEL);
    
    EDMA_SetChannelMux(TFORMAT_DMA, TFORMAT_DMA_TX_CHANNEL, (uint32_t)kDma0RequestLPUART2Tx);
    EDMA_SetChannelMux(TFORMAT_DMA, TFORMAT_DMA_RX_CHANNEL, (uint32_t)kDma0RequestLPUART2Rx);
    
    LPUART_TransferCreateHandleEDMA(TFORMAT_UART, &s_tfLpuartEdmaHandle, tformat_edma_cb, NULL, &s_tfEdmaTxHandle, &s_tfEdmaRxHandle);

    EnableIRQ(DMA_CH6_IRQn);
    EnableIRQ(DMA_CH7_IRQn);
}

/* master-side functions removed for slave-only build */

void tformat_slave_loop(void)
{
    uint8_t req[3];
    
    for (;;)
    {
        /* Receive CF (1 byte) */
        if (kStatus_Success != tformat_rx_bytes(req, 1U))
        {
            continue;
        }
        
        uint8_t cf = req[0];

        if (cf == CF_ID0_ABS)
        {
            /* Build response: CF, SF=0, ABS0..ABS2, CRC */
            uint32_t counts = (uint32_t)encoder_get_abs_counts();
            uint8_t resp[RESP_LEN_ID0];
            resp[0] = CF_ID0_ABS;
            resp[1] = encoder_get_status(); /* SF */
            resp[2] = (uint8_t)(counts & 0xFFU);
            resp[3] = (uint8_t)((counts >> 8) & 0xFFU);
            resp[4] = (uint8_t)((counts >> 16) & 0xFFU);
            resp[5] = tformat_calc_crc(resp, RESP_LEN_ID0 - 1U);
            (void)tformat_tx_bytes(resp, sizeof(resp));
        }
        else if (cf == CF_IDD_EEPROM_READ)
        {
            /* Receive ADF + CRC (2 bytes) */
            if (kStatus_Success != tformat_rx_bytes(&req[1], 2U))
            {
                continue;
            }
            uint8_t adf = req[1];
            uint8_t crcReq = req[2];
            uint8_t crcCalcReq = tformat_calc_crc(req, 2U);
            (void)crcReq; (void)crcCalcReq; /* minimal: ignore request CRC mismatch */

            /* Prepare EDF (temperature raw if ADF matches configured address) */
            uint8_t edf = 0x00;
            if (adf == s_tempAdfAddr)
            {
                /* TODO: replace with real temperature source if available */
                edf = 0x20; /* placeholder */
            }
            /* Build response: CF, ADF, EDF, CRC */
            uint8_t resp[RESP_LEN_IDD];
            resp[0] = CF_IDD_EEPROM_READ;
            resp[1] = adf;
            resp[2] = edf;
            resp[3] = tformat_calc_crc(resp, RESP_LEN_IDD - 1U);
            (void)tformat_tx_bytes(resp, sizeof(resp));
        }
        else if (cf == CF_ID3_ALL)
        {
            /* Build response: CF, SF=0, ABS0..ABS2, ENID, ABM0..ABM2, ALMC, CRC */
            uint32_t absCounts = (uint32_t)encoder_get_abs_counts();
            uint32_t abmCounts = encoder_get_abm_counts24();

            uint8_t resp[RESP_LEN_ID3];
            resp[0] = CF_ID3_ALL;
            resp[1] = encoder_get_status(); /* SF */
            resp[2] = (uint8_t)(absCounts & 0xFFU);
            resp[3] = (uint8_t)((absCounts >> 8) & 0xFFU);
            resp[4] = (uint8_t)((absCounts >> 16) & 0xFFU);
            resp[5] = encoder_get_id(); /* ENID */
            resp[6] = (uint8_t)(abmCounts & 0xFFU);
            resp[7] = (uint8_t)((abmCounts >> 8) & 0xFFU);
            resp[8] = (uint8_t)((abmCounts >> 16) & 0xFFU);
            resp[9] = encoder_get_alarm(); /* ALMC */
            resp[10] = tformat_calc_crc(resp, RESP_LEN_ID3 - 1U);
            (void)tformat_tx_bytes(resp, sizeof(resp));
        }
        else
        {
            /* Unsupported CF: ignore */
        }
    }
}
