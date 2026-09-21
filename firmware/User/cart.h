#ifndef __CART_H
#define __CART_H

#include "ch32v4x7.h"

/*
 * CH32V407VET6 (RISKYMSX2) MSX cartridge bus pinout.
 *
 *   PD0..PD15   A0..A15   (16-bit address bus)  -> GPIOD->INDR
 *   PB8..PB15   D0..D7    (8-bit data bus)      -> GPIOB OUTDR[15:8]
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
#define CART_SLTSL_MASK   0x0001U  /* PE0  ~SLTSL  (slot select, EXTIO trigger) */
#define CART_RD_MASK      0x0002U  /* PE1  ~RD */
#define CART_WR_MASK      0x0004U  /* PE2  ~WR */
#define CART_MREQ_MASK    0x0020U  /* PE5  ~MREQ  (memory-cycle qualifier)  */

/* Data bus drive config for GPIOB CFGHR (pins 8..15).
 * 0x3 nibble = 50MHz push-pull output, 0x4 nibble = floating input. */
#define CART_BUS_ON       0x33333333U
#define CART_BUS_OFF      0x44444444U

/* Cart image lives entirely in PSRAM. The 64 Mbit device is mapped at
 * 0x80000000..0x80800000 (8 MiB); we reserve the whole thing as the cart
 * image window. Konami/ASCII mappers can map up to 1 MiB; NEO up to 256
 * KiB; the unused upper portion is just not mapped to a Z80 bank. */
#define PSRAM_CART_BASE   0x80000000UL
#define PSRAM_CART_SIZE   (8U * 1024U * 1024U)   /* 8 MiB */

/* ------------------------------------------------------------------ */
/* GAME BASE (PSRAM placement diagnostic)                             */
/* ------------------------------------------------------------------ */
/* Where inside the PSRAM window the served cart image starts.
 *
 * Every mapper read path adds this constant to the computed PSRAM
 * address (C handlers add CART_GAME_BASE; the asm handlers fold the
 * same constant into their `lui` immediates - see ASM_PSRAM_GAME_HI /
 * ASM_ROM_BIAS_HI in cart.c). The LOAD_ROM / CAT / XLOAD / DUMP paths
 * write to the same offset, so the image moves as one block.
 *
 * Purpose: place the same game at several PSRAM regions in turn and
 * compare behaviour - if a game works at 0 MB but hangs at 2 MB, the
 * failure is address-dependent (a PSRAM row/bank addressing bug),
 * not a mapper/bank-switching logic bug.
 *
 * Set it from the build:
 *     make rebuild GAME_BASE_MB=4        # image at PSRAM + 4 MiB
 *     make rebuild GAME_BASE_MB=0        # default: image at PSRAM + 0
 * Valid: 0..7 (must leave room for the image, max 8 MiB window).
 * CART_GAME_BASE is the byte offset (MiB * 1048576). */
#ifndef CART_GAME_BASE_MB
#define CART_GAME_BASE_MB   0
#endif
#define CART_GAME_BASE      ((uint32_t)(CART_GAME_BASE_MB) * 1024UL * 1024UL)

#if (CART_GAME_BASE_MB) < 0
#error "CART_GAME_BASE_MB must be >= 0"
#endif
#if (CART_GAME_BASE_MB) > 7
#error "CART_GAME_BASE_MB must be <= 7 (8 MiB window)"
#endif

/* Mapper selection.
 *
 * The active mapper is held in g_mapper (in cart.c). The EXTI0 handler
 * dispatches to the right Run<X> function based on it. Simple ROM mappers
 * (ROM16/32/48) have hand-scheduled asm handlers; bank-switching mappers
 * (Konami/ASCII8k/ASCII16k/NEO8/NEO16) have C handlers that mutate
 * bankOffsets[] in zero-wait-state SRAM on writes.
 *
 * SCC support: CART_MAP_KONAMISCC is the port of the legacy v303
 *       "Konami with SCC" mapper. Bank-switch semantics match
 *       CART_MAP_KONAMINOSCC (selector writes accepted anywhere in
 *       0x4000..0xBFFF except the SCC-I register window 0x9800..0x98FF,
 *       which the real hardware decodes as the sound chip). Every
 *       cartridge write is queued for the SCC emulator core (emu2212,
 *       ported verbatim from v303Firmware), which renders samples to
 *       DAC1/PA4 (SCC_OUT) at a timer-derived ~44.1 kHz rate - see
 *       scc.c/scc.h.
 *
 * Note: ROM16k maps a 16 KiB image at 0x4000..0x7FFF (page 1) and the
 *       same image at 0x8000..0xBFFF (page 2) - the standard MSX
 *       "mirrored 16 KiB" layout. ROM32k maps 32 KiB at 0x4000..0xBFFF.
 *       ROM48k maps 48 KiB at 0x4000..0xFFFF (overlaps the BIOS slot at
 *       page 3, but the BIOS still owns page 3 from its perspective; the
 *       cart just mirrors the same 16 KiB bank across all three pages). */
typedef enum {
    CART_MAP_NONE       = 0,   /* no mapper installed (bus floats) */
    CART_MAP_ROM16k     = 1,
    CART_MAP_ROM32k     = 2,
    CART_MAP_ROM48k     = 3,
    CART_MAP_KONAMI     = 4,   /* Konami with bank-switching at 0x6000/0x8000/0xA000 only */
    CART_MAP_KONAMINOSCC= 5,   /* Konami with bank-switching at any addr in 0x4000..0xBFFF */
    CART_MAP_ASCII8k    = 6,
    CART_MAP_ASCII16k   = 7,
    CART_MAP_NEO8       = 8,
    CART_MAP_NEO16      = 9,
    CART_MAP_KONAMISCC  = 10,  /* Konami-with-SCC: banks + writes queued to the
                                  * SCC emulator (see scc.c). Read path treats
                                  * the 0x9800..0x98FF register window as the
                                  * sound chip, like real hardware. */
    CART_MAP_FLASH      = 11,  /* Flash cart: serves the embedded ROM image
                                  * (maptest/selector/etc) directly from
                                  * flash for ordinary reads, and decodes the
                                  * 0x7FF0..0x7FFF mailbox window for the
                                  * MSX-side loader. The boot mapper - stays
                                  * active across SET_MAPPER + RESET so the
                                  * user can pick a new mapper from the menu. */
    CART_MAP_TERMINAL   = 12,  /* Terminal cart: serves the embedded
                                  * terminal ROM (terminal_rom[]) at
                                  * 0x4000..0xBFFF for ordinary reads, and
                                  * decodes the v303-style 0x7FFD/0x7FFE/
                                  * 0x7FFF mailbox. The MSX-side terminal
                                  * program drives the screen + keyboard;
                                  * the firmware hosts the menu logic
                                  * (file list, ROM load, mapper select).
                                  * The boot mapper for the terminal
                                  * workflow - swap away via the menu to
                                  * the FLASH mapper if the user prefers
                                  * the existing mailbox-driven loader. */
    CART_MAP_MAX        = 13,
} Cart_Mapper;

/* Mapper names used by `MAP ?` and CLI error messages. */
extern const char *const Cart_MapperNames[CART_MAP_MAX];

/* Set the active mapper. Installs the corresponding EXTI0 handler in the
 * PFIC VTF slot. Returns 0 on success, -1 on invalid mapper or if PSRAM
 * is not initialised yet (PSRAM_Init() must have run). */
int  Cart_SetMapper (Cart_Mapper m);
Cart_Mapper Cart_GetMapper (void);

/* Hardened mapper-swap primitive (see Cart_SetMapper_Safe() body in
 * cart.c for the full list of hazards it closes).
 *
 * Disables EXTI0 + global IRQ, waits for ~SLTSL to go high (no
 * in-flight cart cycle), drives the data bus off, performs the
 * swap (handler + bankOffsets + g_mapper), clears any phantom
 * EXTI0 edge latched during the wait, issues a DSB/ISB fence, and
 * re-enables IRQ.
 *
 * An earlier version also asserted MSX ~RESET low for the duration
 * of the swap. That path is REMOVED: most MSX2+ machines expose the
 * cart-edge ~RESET as read-only and driving it externally can
 * damage the mainboard. The CMD_SOFTRESET slingshot (running from
 * MSX RAM) provides the equivalent atomicity without touching PE4.
 *
 * Returns the same value as Cart_SetMapper(). */
int  Cart_SetMapper_Safe (Cart_Mapper m);

/* Get the base address of the cart image window in PSRAM. Always
 * PSRAM_CART_BASE. Kept for API symmetry with the previous SRAM/PSRAM
 * dual-window design. */
uint32_t Cart_GetImageBase (void);

/* Byte offset of the served game image within the PSRAM window
 * (CART_GAME_BASE; see the GAME BASE comment block above). Print it
 * in boot/CLI status lines. */
uint32_t Cart_GetGameBase (void);

/* Total cart-image window size (bytes). */
uint32_t Cart_GetImageSize (void);

/* Initialise GPIO + EXTI for the cartridge bus and install a default
 * mapper handler. PSRAM_Init() must have been called first (or the
 * handler reads from an unmapped address and hangs the MSX).
 *
 * NOTE: PE4 (MSX ~RESET) is configured here as a floating input only.
 * The firmware never drives it (read-only on most MSX2+ machines;
 * driving it externally can damage the mainboard). See
 * Cart_SetMapper_Safe for the cart-swap primitive that closes the
 * race window without touching PE4, and loader.c::cmd_softreset for
 * the CMD_SOFTRESET reboot path. */
void Init_Cart (void);

/* Legacy SCC read path: return the emulator's view of a SCC-I read at
 * `address` (0x9800..0x98FF), or -1 if the address is outside that
 * window. Called by the KONAMISCC mapper's read handler so the MSX can
 * read back the SCC register file / status byte, exactly like the
 * legacy v303 RunKonamiWithSCC path did. Returns 0..255 on hit.
 * Runs in IRQ context (EXTI0) - must not printf/block. */
int Cart_SCC_ReadByte (uint16_t address);

/* EXTI0 IRQ: dispatcher. Reads g_mapper and the bus state, calls the
 * active Run<Mapper>() function. Runs from .ramfunc (zero-wait-state
 * SRAM). Marked noinline + WCH-Interrupt-fast so the VTF dispatch path
 * works as documented. */
void Cart_EXTI0_Dispatch (void) __attribute__((section(".ramfunc"), noinline,
                                                interrupt("WCH-Interrupt-fast")));

/* Legacy alias: keep the old name working in case something else links
 * to it. Maps to the dispatcher. */
#define Cart_EXTI0_Handler Cart_EXTI0_Dispatch

/* Diagnostic / CLI polling cart service. Retained for compatibility but
 * unused by the EXTI-driven path. Must be called with global IRQs
 * disabled. */
void CartServiceLoop(void) __attribute__((section(".ramfunc"), noinline));

/* NOTE: there is intentionally no Cart_AssertMSXReset* API in this
 * header. The firmware never drives MSX ~RESET (PE4 is read-only on
 * most MSX2+ machines; driving it externally can damage the
 * mainboard). Reboot is via the MSX-side loader's CMD_SOFTRESET
 * slingshot (loader.c). */

#endif