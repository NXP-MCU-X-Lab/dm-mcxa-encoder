/*
 * UART DMA demo/test case using current LPUART2.
 * Provides init and a simple TX/RX demonstration with eDMA.
 */

#ifndef APP_UART_DMA_H
#define APP_UART_DMA_H

#include <stdint.h>

/* Initialize EDMA and LPUART EDMA handle for LPUART2 */
void uart_dma_demo_init(void);

/* Run a simple demonstration: DMA TX a message and DMA RX N bytes */
void uart_dma_demo_run(void);

#endif /* APP_UART_DMA_H */