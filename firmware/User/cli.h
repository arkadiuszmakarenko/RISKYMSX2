/*
 * Command-line interface over USART1 (PA9=TX, PA10=RX @ 921600 8N1).
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
 *   USB                        - poll the USBHS root hub and start
 *                                enumeration if a device is attached.
 *                                Prints OK <speed> <in> <out> on success
 *                                or ERR <code> on failure. Idempotent;
 *                                retries enum/mount up to 5x internally
 *                                to ride out stick-settle delays.
 *   USBD                       - verbose enumeration diagnostic. Walks
 *                                the full USBHS host pipeline (port
 *                                enable, device descriptor, config,
 *                                MSC endpoint scan, SCSI READ CAPACITY
 *                                with REQUEST SENSE dump on failure).
 *                                Does NOT mount the volume, so it's
 *                                safe to run when the stick refuses to
 *                                enumerate.
 *   UREAD                      - low-level USB drive read test. Bypasses
 *                                FATFS - issues SCSI INQUIRY, TEST UNIT
 *                                READY, READ CAPACITY(10), hexdumps LBA
 *                                0 (with MBR parse), spot-reads 4 LBAs
 *                                across the volume, and times a 32 KiB
 *                                contiguous read for throughput.
 *                                Useful for sticks that enumerate but
 *                                won't mount.
 *   FAT                        - full FAT integration test: mounts the
 *                                volume, lists the root directory,
 *                                creates 0:/RISKYMSX2.TXT, reads it
 *                                back and verifies byte-for-byte.
 *                                Exercises the SCSI WRITE(10) path.
 *   LS [path]                  - list the first 16 entries of the root
 *                                directory (or `path` if given) of the
 *                                mounted FAT volume. Path defaults to "/".
 *   CAT <path> <addr> [len]    - copy <len> bytes of <path> from the USB
 *                                stick into PSRAM at <addr>. If [len] is
 *                                omitted, the file size is used. Uses DMA
 *                                to land the data. Replies "OK <copied>"
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