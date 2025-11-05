/*
 * UART DMA demo/test case using current LPUART2.
 * - Configures EDMA channels for LPUART2 TX/RX
 * - Demonstrates DMA-based transmit of a message
 * - Demonstrates DMA-based receive of a fixed-size buffer
 */

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"
#include "fsl_edma.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app_uart_dma.h"

#include <stdio.h>

#define UART_DMA_TX_CHANNEL   (7U)
#define UART_DMA_RX_CHANNEL   (6U)

static edma_handle_t s_edmaTxHandle;
static edma_handle_t s_edmaRxHandle;
static lpuart_edma_handle_t s_lpuartEdmaHandle;


static void uart_edma_callback(LPUART_Type *base,
                               lpuart_edma_handle_t *handle,
                               status_t status,
                               void *userData)
{
    (void)base;
    (void)handle;
    (void)userData;
    if (status == kStatus_LPUART_TxIdle)
    {
       printf("Tx Idle\r\n");
    }
    else if (status == kStatus_LPUART_RxIdle)
    {
        printf("Rx Idle\r\n");
    }
}

void uart_dma_demo_init(void)
{
    /* Ensure DMA0 is out of reset */
    RESET_ReleasePeripheralReset(kDMA0_RST_SHIFT_RSTn);

    /* Initialize eDMA */
    edma_config_t dmaConfig;
    EDMA_GetDefaultConfig(&dmaConfig);
    EDMA_Init(DMA0, &dmaConfig);

    /* Create EDMA handles for TX and RX channels */
    EDMA_CreateHandle(&s_edmaTxHandle, DMA0, UART_DMA_TX_CHANNEL);
    EDMA_CreateHandle(&s_edmaRxHandle, DMA0, UART_DMA_RX_CHANNEL);

    /* Route DMAMUX request to LPUART2 TX/RX */
    EDMA_SetChannelMux(DMA0, UART_DMA_TX_CHANNEL, (uint32_t)kDma0RequestLPUART2Tx);
    EDMA_SetChannelMux(DMA0, UART_DMA_RX_CHANNEL, (uint32_t)kDma0RequestLPUART2Rx);

    /* Create LPUART EDMA handle using LPUART2 */
    LPUART_TransferCreateHandleEDMA(LPUART2, &s_lpuartEdmaHandle, uart_edma_callback, NULL, &s_edmaTxHandle, &s_edmaRxHandle);

    /* Enable DMA channel IRQs for major loop completion */
    EnableIRQ(DMA_CH6_IRQn);
    EnableIRQ(DMA_CH7_IRQn);
}

void uart_dma_demo_run(void)
{
    /* ===== DMA TX demo ===== */
    static const uint8_t txMsg[] = "[UART2 DMA] TX: Hello DMA!\r\n";
    lpuart_transfer_t txXfer;
    txXfer.data = (uint8_t *)txMsg;
    txXfer.dataSize = (size_t)(sizeof(txMsg) - 1U);

    if (kStatus_Success != LPUART_SendEDMA(LPUART2, &s_lpuartEdmaHandle, &txXfer))
    {
        printf("UART2 DMA TX start failed\r\n");
        return;
    }


    /* ===== DMA RX demo ===== */
    /* Ask user to type up to N bytes on serial to receive */

    uint8_t rxBuf[32] = {0};
    lpuart_transfer_t rxXfer;
    rxXfer.data = rxBuf;
    rxXfer.dataSize = sizeof(rxBuf);

    if (kStatus_Success != LPUART_ReceiveEDMA(LPUART2, &s_lpuartEdmaHandle, &rxXfer))
    {
        printf("UART2 DMA RX start failed\r\n");
        return;
    }

//        /* Abort RX and get received count so far */
//        LPUART_TransferAbortReceiveEDMA(LPUART2, &s_lpuartEdmaHandle);
//        uint32_t rxCount = 0U;
//        if (kStatus_Success == LPUART_TransferGetReceiveCountEDMA(LPUART2, &s_lpuartEdmaHandle, &rxCount))
//        {
//            printf("[UART2 DMA] RX aborted after timeout, received %lu bytes\r\n", (unsigned long)rxCount);
//        }
//        else
//        {
//            printf("[UART2 DMA] RX aborted after timeout\r\n");
//        }
//    }
//    else
//    {
//        printf("[UART2 DMA] RX completed 32 bytes\r\n");
//    }

//   
}