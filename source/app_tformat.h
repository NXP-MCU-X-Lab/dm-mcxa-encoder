/*
 * Tamagawa T-Format slave-only responder using LPUART2 + eDMA
 * - Responds to CF=ID0 (ABS) and CF=IDD (EEPROM read) and CF=ID3 (combined)
 * - Baud: 2.5Mbps during transaction
 */

#ifndef APP_TFORMAT_H
#define APP_TFORMAT_H

#include <stdint.h>
#include <stddef.h>

/* Initialize T-Format slave (sets up EDMA/DMAMUX and LPUART2 handle). */
void tformat_init(void);

/* Slave: run T-Format responder loop (blocks). */
void tformat_slave_loop(void);

#endif /* APP_TFORMAT_H */