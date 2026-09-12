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
 *   SRC [SRAM|PSRAM]           - show or switch the active cart image
 *                                window that LOAD/XLOAD/DUMP target.
 *                                Replies "SRC <name> @ <hexaddr>" or
 *                                "OK <name> @ <hexaddr>" on switch.
 *                                PSRAM is refused (ERR) until
 *                                PSRAM_Init() has passed.
 *   LOAD <hexaddr> <hexbytes>  - write a sequence of hex bytes into the
 *                                ACTIVE image window starting at
 *                                <hexaddr>. Whitespace between bytes
 *                                is optional. Replies OK <written> or ERR.
 *   DUMP <hexaddr> <len>       - read <len> bytes from the ACTIVE image
 *                                window at <hexaddr> and emit them as
 *                                2-digit hex pairs with a trailing CRLF.
 *                                Replies
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
 * after SystemCoreClockUpdate() but before the service loop. */
void CLI_Init(void);

/* Main-loop command dispatcher. The USART1 IRQ only assembles lines and
 * sets a pending flag; CLI_Service() executes the command and prints
 * the response + prompt. Must be called from the main loop (e.g. every
 * wfi wake-up) - command responses are never printed from the IRQ, so
 * printf/TXE busy-waits can never drop incoming bytes. */
void CLI_Service(void);

/* USART1 IRQ entry. Wired via ch32v4x7_it.c. */
void CLI_USART1_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __CLI_H */