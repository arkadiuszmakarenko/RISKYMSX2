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
 *   PE3  (pin  2)  ~WAIT     bus-cycle stretch   -> GPIO (low output/input)
 *   PE5  (pin  4)  MREQ
 *
 *   PA0  (pin 23)  LEDFLASH (status LED)
 */

/* Control-bus bit masks within GPIOE->INDR */
#define CART_SLTSL_MASK   0x0001U  /* PE0  ~SLTSL  (slot select, EXTIO trigger) */
#define CART_RD_MASK      0x0002U  /* PE1  ~RD */
#define CART_WR_MASK      0x0004U  /* PE2  ~WR */
#define CART_WAIT_MASK    0x0008U  /* PE3  ~WAIT (active low output) */
#define CART_MREQ_MASK    0x0020U  /* PE5  ~MREQ  (memory-cycle qualifier)  */
#define CART_M1_MASK      0x0040U  /* PE6  M1 (low during opcode fetch) */
#define CART_IORQ_MASK    0x0100U  /* PE8  ~IORQ (MSX mapper bank IRQ) */

/* Data bus drive config for GPIOB CFGHR (pins 8..15).
 * 0x3 nibble = 50MHz push-pull output, 0x4 nibble = floating input. */
#define CART_BUS_ON       0x33333333U
#define CART_BUS_OFF      0x44444444U

/* MSX memory-mapper bank registers used by CART_MAP_SLOTTED. One 8-bit
 * register selects one 16 KiB bank for each CPU page: FC->page 0,
 * FD->page 1, FE->page 2, FF->page 3. Four MiB = 256 x 16 KiB banks.
 * Keep this mapper image in the second half of the 8 MiB PSRAM window. */
#define CART_SLOTTED_BANK_PORT_BASE   0xFCU
#define CART_SLOTTED_BANK_COUNT       256U
#define CART_SLOTTED_BANK_SIZE        (16U * 1024U)
#define CART_SLOTTED_BANK_BASE_OFFSET (4U * 1024U * 1024U)

/* Sub-slots exposed by CART_MAP_SLOTTED.
 *
 * Sub-slot 0 is served by the embedded tiny_rom[] (flash, read-only).
 * Sub-slot 1 is served by PSRAM (read/write, initialized before startup)
 * and uses the MSX mapper registers above.
 * Sub-slots 2 and 3 float as empty slots.
 */

/* Cart image lives entirely in PSRAM. The 64 Mbit device is mapped at
 * 0x80000000..0x80800000 (8 MiB); we reserve the whole thing as the cart
 * image window. Konami/ASCII mappers can map up to 1 MiB; NEO up to 256
 * KiB; the unused upper portion is just not mapped to a Z80 bank. */
#define PSRAM_CART_BASE   0x80000000UL
#define PSRAM_CART_SIZE   (8U * 1024U * 1024U)   /* 8 MiB */

#if (CART_SLOTTED_BANK_BASE_OFFSET + (CART_SLOTTED_BANK_COUNT * CART_SLOTTED_BANK_SIZE)) > PSRAM_CART_SIZE
#error "CART_SLOTTED mapper region exceeds PSRAM_CART_SIZE"
#endif

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
    CART_MAP_NEXTOR     = 13,  /* Nextor kernel mapper: ASCII16K over the
                                   * embedded Nextor 3.0 kernel ROM
                                   * (nextor_rom[], flash-resident). The
                                   * mailbox window at 0x7FF0..0x7FF5 is
                                   * handled by nextor.c. */
    CART_MAP_SLOTTED    = 14,  /* MSX expanded slot with four RAM subslots */
    CART_MAP_MAX        = 15,
} Cart_Mapper;

/* Mapper names used by `MAP ?` and CLI error messages. */
extern const char *const Cart_MapperNames[CART_MAP_MAX];

/* Set the active mapper. Installs the corresponding EXTI0 handler in the
 * PFIC VTF slot. Returns 0 on success, -1 on invalid mapper or if PSRAM
 * is not initialised yet (PSRAM_Init() must have run). */
int  Cart_SetMapper (Cart_Mapper m);
Cart_Mapper Cart_GetMapper (void);

/* Diagnostics: count of cart image-window READ cycles served post-swap
 * (ROM32k/ROM48k handlers). Cleared by soft_reset_into_cart, printed
 * after the 500 ms boot-probe observation window. rd>0 proves the MSX
 * BIOS probed the cart after the reset dance; rd==0 means it never
 * touched the cart window (rom_start/reset path failed MSX-side). */
extern volatile uint32_t g_cart_rd_cycles;
extern volatile uint32_t g_cart_wr_cycles;
uint32_t Cart_GetRdCycles (void);
uint32_t Cart_GetWrCycles (void);

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

/* Print throttled diagnostics for the expanded-slot IRQ path. The handler
 * only records counters/snapshots; this function performs the printf from
 * main context so the Z80 bus cycle is not blocked by UART output. */


/* CART_MAP_SLOTTED owns the MSX memory-mapper ports 0xFC..0xFF through
 * the PE8/~IORQ handler. Each port selects one 16 KiB PSRAM bank for a
 * corresponding CPU page. */

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
 * PE3 (~WAIT) is preserved as a low output when the startup wait hold is
 * active; PE4 (~RESET) remains a floating input. */
void Init_Cart (void);

/* Hold/release the Z80 using the standard cartridge ~WAIT input around
 * cart startup. Begin drives PE3 low. End returns PE3 to a floating input
 * and waits for the held bus cycle to finish. */
void Cart_HoldMSXWait_Begin (void);
void Cart_HoldMSXWait_End (void);
void Cart_SetSlottedPSRAMReady (uint8_t ready);

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

#endif
