/********************************** (C) COPYRIGHT *******************************
 * File Name          : psram.h
 * Description        : PSRAM bring-up, self-test, and ROM-image mirror
 *                      for the RISKYMSX2 cartridge handler.
 *
 *                      After PSRAM_Init() returns nonzero (PASS), the
 *                      32 KiB hello_rom[] image has been DMA-equivalent-copied
 *                      into PSRAM at PSRAM_BASE (0x80000000). The application
 *                      can then point cartpnt at PSRAM_BASE for zero-wait-state
 *                      read access from the .ramfunc cartridge handler.
 *
 *                      The self-test uses a deterministic 1 KiB pattern and
 *                      runs CPU-side read/write checks at three PSRAM offsets
 *                      (start, mid, end-of-mapped-bank) before declaring the
 *                      chip healthy.
 *********************************************************************************/

#ifndef __PSRAM_H
#define __PSRAM_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

/* PSRAM bus address on CH32V4x7. Note: ch32v4x7.h already defines
 * PSRAM_BASE as the peripheral register block, so we use a different name
 * for the external memory bus address. */
#define PSRAM_BUS_BASE   0x80000000UL

/* Cart ROM size we mirror into PSRAM. Must match hello_rom[]. */
#define PSRAM_ROM_SIZE   32768U

/* Self-test pattern size. 1 KiB is enough to exercise multi-row addressing
 * on a 32 Mbit PSRAM (4 KiB page per row x 1024 rows). */
#define PSRAM_TEST_SIZE  1024U

/* Return value of PSRAM_Init(): nonzero on success. The bitmask matches
 * the failure stages so the caller can report partial diagnostics. */
#define PSRAM_OK                0x00U
#define PSRAM_ERR_CLOCK         0x01U /* RCC_HBPeriphClockCmd failed / not enabled */
#define PSRAM_ERR_INIT          0x02U /* Peripheral init register write rejected */
#define PSRAM_ERR_TEST_PATTERN  0x04U /* Wrote pattern, readback mismatch */
#define PSRAM_ERR_TEST_OFFSETS  0x08U /* Multi-offset test mismatch */
#define PSRAM_ERR_ROM_MIRROR    0x10U /* hello_rom -> PSRAM copy mismatch */
#define PSRAM_ERR_HCLK_TOO_HIGH 0x20U /* HCLK above what this PSRAM device supports */

/* Public entry points. */
uint8_t PSRAM_Init(void);                /* Init + self-test. Returns PSRAM_OK on success. */
uint32_t PSRAM_GetRomMirrorBase(void);   /* 0x80000000 once Init has succeeded. */

#ifdef __cplusplus
}
#endif

#endif /* __PSRAM_H */