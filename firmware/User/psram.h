/********************************** (C) COPYRIGHT *******************************
 * File Name          : psram.h
 * Description        : PSRAM bring-up and self-test for RISKYMSX2.
 *
 *                      The cart handler no longer reads from PSRAM - the
 *                      cart image is mirrored into zero-wait-state
 *                      internal SRAM at boot by the startup copy loop in
 *                      startup_ch32v4x7.S (see Ld/Link.ld CARTROM
 *                      region). PSRAM is left as a diagnostic peripheral:
 *                      PSRAM_Init() initialises the FSMC/PSRAM controller
 *                      and runs a multi-offset read/write self-test, then
 *                      returns PSRAM_OK or a failure bitmask. main() may
 *                      choose to call it (or not).
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
#define PSRAM_ERR_HCLK_TOO_HIGH 0x20U /* HCLK above what this PSRAM device supports */

/* Public entry point. */
uint8_t PSRAM_Init(void);  /* Init + self-test. Returns PSRAM_OK on success. */

#ifdef __cplusplus
}
#endif

#endif /* __PSRAM_H */