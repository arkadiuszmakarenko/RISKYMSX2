#ifndef __CART_H
#define __CART_H

#include "ch32v4x7.h"

/*
 * CH32V467VET6 (RISKYMSX2) MSX cartridge bus pinout.
 *
 *   PD0..PD15   A0..A15   (16-bit address bus)  -> GPIOD->INDR
 *   PB8..PB15   D0..D7    (8-bit data bus)      -> GPIOB high byte
 *
 *   PE0  (pin 97)  ~SLTSL   slot select          -> EXTI0 trigger
 *   PE1  (pin 98)  RD
 *   PE2  (pin  1)  WR
 *   PE3  (pin  2)  ~WAIT     bus-cycle stretch   -> GPIO (open-drain out)
 *   PE5  (pin  4)  MREQ
 *
 *   PA0  (pin 23)  LEDFLASH (status LED)
 */

/* Control-bus bit masks within GPIOE->INDR */
#define CART_SLTSL_MASK   0x0001   /* PE0 */
#define CART_RD_MASK      0x0002   /* PE1 */
#define CART_WR_MASK      0x0004   /* PE2 */
#define CART_WAIT_MASK    0x0008   /* PE3 (mirror of OUTDR bit for diagnostics) */
#define CART_MREQ_MASK    0x0020   /* PE5 */

/* Data bus drive config for GPIOB CFGHR (pins 8..15).
 * 0x3 nibble = 50MHz push-pull output, 0x4 nibble = floating input. */
#define CART_BUS_ON       0x33333333
#define CART_BUS_OFF      0x44444444

/* PE3 (WAIT) drive config in GPIOE->CFGLR.
 *  0x4 = floating input (released, MSX pull-up holds line high).
 *  0x7 = open-drain output with 50 MHz slew.
 * PE3 corresponds to CFGLR bits[19:16]; see ch32v4x7.h GPIO CRL fields. */
#define CART_WAIT_RELEASED 0x4U    /* nibble for "released"  */
#define CART_WAIT_DRIVEN   0x7U    /* nibble for "assert low" */
#define CART_WAIT_NIBBLE_SHIFT 16U /* PE3 is bits[19:16] of GPIOE->CFGLR */

/* Cart ROM mirror in zero-wait-state internal SRAM. The startup copy
 * loop in startup_ch32v4x7.S copies hello_rom[] from flash LMA to this
 * SRAM VMA at reset, so cartpnt can read from it without any flash
 * wait states. The address is exported by the linker as _cartrom_vma
 * (see Ld/Link.ld CARTROM region). */
extern uint32_t _cartrom_vma;
#define SRAM_ROM_BASE ((uint32_t)&_cartrom_vma)

/* Cart ROM size in bytes. Must match hello_rom[]. */
#define CART_ROM_SIZE 32768U

/* Helper accessor - matches the old Cart_GetImageBase() return type
 * (cart image source address). Now always SRAM. */
static inline uint32_t Cart_GetRomMirrorBase(void) { return SRAM_ROM_BASE; }

/* Zero the SRAM cart mirror. Called from main() before the cart IRQ
 * is enabled, so the MSX sees an open-bus (0xFF) pattern until a
 * CLI XLOAD/LOAD command populates it. */
void ROM_Clear(void);

void Init_Cart(void);

/* EXTI0 interrupt handler. Dispatched directly by the PFIC via the VTF
 * (vector-table-free) slot set up by Init_Cart(). Serves one Z80 read on
 * the falling edge of ~SLTSL and releases the data bus on the rising
 * edge. Runs from .ramfunc.
 *
 * Marked WCH-Interrupt-fast so the prologue/epilogue match what the VTF
 * dispatcher expects. */
void Cart_EXTI0_Handler(void) __attribute__((section(".ramfunc"), noinline,
                                              interrupt("WCH-Interrupt-fast")));

/* Legacy polling cart service. Not used by the EXTI-driven path, but
 * retained so existing callers don't have to be updated. Must run with
 * interrupts globally disabled (or with no other timing-critical code
 * sharing the CPU). Returns only on impossible conditions; in normal
 * operation it never returns.
 *
 * Marked __attribute__((section(".ramfunc"), noinline)) so the linker
 * keeps the loop body intact and runs it from zero-wait-state SRAM. */
void CartServiceLoop(void) __attribute__((section(".ramfunc"), noinline));

/* Get the base address of the active image (for diagnostics / logging).
 * Always returns PSRAM_BUS_BASE - the cart source is hard-wired to PSRAM
 * and there is no runtime selector. */
uint32_t Cart_GetImageBase(void);

/* Pulse the MSX ~RESET line (PE4) low for `ms` milliseconds, then release.
 * Used by the CLI's RST command. */
void Cart_AssertMSXReset(uint32_t ms);

#endif
