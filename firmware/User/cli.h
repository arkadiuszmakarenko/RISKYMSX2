/*
 * Command-line interface over USART1 (PA9=TX, PA10=RX @ 115200 8N1).
 *
 * Line-oriented ASCII protocol. Commands are terminated by '\r' or '\n'
 * (echoed as received). Each command produces an OK/ERR response followed
 * by a CRLF.
 *
 * Commands:
 *   PING                       - replies OK (liveness check)
 *   HELP                       - prints the command list
 *   RST [ms]                   - pulse MSX ~RESET (PE4) low for [ms] ms
 *                                (default 100 ms). Replies OK.
 *   LOAD <hexaddr> <hexbytes>  - write a sequence of hex bytes into PSRAM
 *                                starting at <hexaddr> (relative to
 *                                PSRAM_BUS_BASE). Whitespace between bytes
 *                                is optional. Replies OK <written> or ERR.
 *   DUMP <hexaddr> <len>       - read <len> bytes from PSRAM at <hexaddr>
 *                                and emit them as 2-digit hex pairs with
 *                                a trailing CRLF. Replies
 *                                  DUMP <hexaddr> <len>: <byte> <byte> ...
 *                                or ERR.
 *
 * The RX side runs from USART1_IRQHandler in cli.c; CLI_Init() configures
 * the USART, registers the NVIC entry, and prints the banner.
 */
#ifndef __CLI_H
#define __CLI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise USART1, enable RXNE IRQ, print banner. Call once from main
 * after SystemCoreClockUpdate() but before the WFI idle loop. */
void CLI_Init(void);

/* USART1 IRQ entry. Wired via ch32v4x7_it.c. */
void CLI_USART1_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __CLI_H */