#ifndef __CART_H
#define __CART_H

#include "ch32v4x7.h"

/* Minimal cart core ported from /firmware for firmware2.
 *
 * Scope: ONLY the pieces required to boot and run code -
 *   1. the Nextor mapper path (CART_MAP_NEXTOR: ASCII16K banking
 *      over the flash-resident nextor_rom[], mailbox routed to
 *      nextor.c),
 *   2. the expanded-slot / sub-slot layer (secondary slot register
 *      at 0xFFFF, M1-qualified, read inverted),
 *   3. the MSX memory mapper (sub-slot 1 mapper RAM, bank registers
 *      latched from I/O ports 0xFC..0xFF via the PE8/~IORQ EXTI9_5
 *      decoder, backed by PSRAM).
 *
 * Everything else from the original cart.c (game mappers, SCC,
 * terminal, loader, ~RESET control, diagnostics counters) is
 * deliberately NOT ported: this firmware installs CART_MAP_NEXTOR
 * once at boot and never swaps it, and the hot path carries no
 * instrumentation so every served Z80 cycle is as short as the
 * hardware allows. */

/* MSX memory-mapper RAM backing store: external PSRAM ONLY, 1024 KiB
 * (64 x 16 KiB banks) at PSRAM_CART_BASE + CART_SLOTTED_BANK_BASE_OFFSET.
 * Sub-slot 1 stays hidden until PSRAM_Init() succeeds and main() calls
 * Cart_SetMapperRamReady(1). */

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
 *   PE6  (pin  3)  M1        (low during opcode fetch)
 *
 *   PA0  (pin 23)  LEDFLASH (status LED)
 */

/* Control-bus bit masks within GPIOE->INDR */
#define CART_SLTSL_MASK   0x0001U  /* PE0  ~SLTSL  (slot select, EXTIO trigger) */
#define CART_RD_MASK      0x0002U  /* PE1  ~RD */
#define CART_WR_MASK      0x0004U  /* PE2  ~WR */
#define CART_WAIT_MASK    0x0008U  /* PE3  ~WAIT (active-low cart output) */
#define CART_RESET_MASK   0x0010U  /* PE4  ~RESET (unused: never driven) */
#define CART_MREQ_MASK    0x0020U  /* PE5  ~MREQ  (memory-cycle qualifier)  */
#define CART_M1_MASK      0x0040U  /* PE6  M1 (low during opcode fetch) */
#define CART_IORQ_MASK    0x0100U  /* PE8  ~IORQ  (I/O-cycle qualifier for
                                    * the 0xFC..0xFF memory-mapper bank
                                    * register decoder) */

/* Data bus drive config for GPIOB CFGHR (pins 8..15).
 * 0x3 nibble = 50MHz push-pull output, 0x4 nibble = floating input. */
#define CART_BUS_ON       0x33333333U
#define CART_BUS_OFF      0x44444444U

/* PSRAM mapper window. Keep it separate from game images. */
#define CART_SLOTTED_BANK_PORT_BASE   0xFCU
#define CART_SLOTTED_BANK_COUNT       256U
#define CART_SLOTTED_BANK_SIZE        (16U * 1024U)

/* Allow mapper RAM in every CPU page by default. The SSR selects whether
 * page 3 uses this cartridge's sub-slot 1; until software selects it,
 * the reset value leaves all pages in sub-slot 0. Set to 0 only for a
 * diagnostic build that must keep page 3 host-owned. */
#ifndef CART_SLOTTED_PAGE3_RAM
#define CART_SLOTTED_PAGE3_RAM 1
#endif
#define CART_SLOTTED_BANK_BASE_OFFSET (4U * 1024U * 1024U)

/* PSRAM mapper capacity: 64 banks x 16 KiB = 1024 KiB. Bank numbers
 * above 63 alias in the same way as on a 1 MiB physical MSX memory
 * mapper. */
#define CART_SLOTTED_PSRAM_BANKS_PRESENT 64U
#define CART_SLOTTED_PSRAM_BANK_MASK \
    (CART_SLOTTED_PSRAM_BANKS_PRESENT - 1U)

_Static_assert ((CART_SLOTTED_PSRAM_BANKS_PRESENT
                 & (CART_SLOTTED_PSRAM_BANKS_PRESENT - 1U)) == 0U,
                "CART_SLOTTED_PSRAM_BANKS_PRESENT must be a power of two "
                "(mask-based aliasing)");
_Static_assert (CART_SLOTTED_PSRAM_BANKS_PRESENT <= CART_SLOTTED_BANK_COUNT,
                "CART_SLOTTED_PSRAM_BANKS_PRESENT exceeds the physical region");

/* Cart image lives entirely in PSRAM. The 64 Mbit device is mapped at
 * 0x80000000..0x80800000 (8 MiB); we reserve the whole thing as the cart
 * image window. The USB layer stages ROM loads and sector buffers here.
 *
 * LAYOUT (verified by the static asserts below):
 *   0x80000000 + 0  ............  +CART_GAME_MAX_SIZE   game image window
 *   0x80000000 + CART_SLOTTED_BANK_BASE_OFFSET
 *                            ......  +PSRAM_CART_SIZE    sub-slot 1
 *                                                    mapper RAM (4 MiB)
 *
 * The two regions MUST NOT overlap. */
#define PSRAM_CART_BASE   0x80000000UL
#define PSRAM_CART_SIZE   (8U * 1024U * 1024U)   /* 8 MiB */

#if (CART_SLOTTED_BANK_BASE_OFFSET \
     + (CART_SLOTTED_BANK_COUNT * CART_SLOTTED_BANK_SIZE)) > PSRAM_CART_SIZE
#error "CART_SLOTTED mapper region exceeds PSRAM_CART_SIZE"
#endif

/* ------------------------------------------------------------------ */
/* GAME BASE (PSRAM placement diagnostic)                             */
/* ------------------------------------------------------------------ */
/* Where inside the PSRAM window the served cart image starts. Kept from
 * the original cart.h because the USB staging path (usb_disk.c) compiles
 * against these constants. Default 0 = image at PSRAM + 0. */
#ifndef CART_GAME_BASE_MB
#define CART_GAME_BASE_MB   0
#endif
#define CART_GAME_BASE      ((uint32_t)(CART_GAME_BASE_MB) * 1024UL * 1024UL)

#if (CART_GAME_BASE_MB) < 0
#error "CART_GAME_BASE_MB must be >= 0"
#endif

/* Max PSRAM MiB that can be used for the cart image, kept clear of the
 * sub-slot 1 mapper RAM region. */
#define CART_GAME_MAX_BYTES  (CART_SLOTTED_BANK_BASE_OFFSET - CART_GAME_BASE)
#define CART_GAME_MAX_MB     ((CART_GAME_MAX_BYTES) / (1024UL * 1024UL))

#if (CART_GAME_BASE_MB) >= (CART_SLOTTED_BANK_BASE_OFFSET / (1024U * 1024U))
#error "CART_GAME_BASE_MB must leave room for the sub-slot 1 mapper RAM at PSRAM + CART_SLOTTED_BANK_BASE_OFFSET"
#endif

/* Compile-time tripwire: the game image window must stay clear of the
 * sub-slot 1 mapper RAM region. */
_Static_assert (CART_GAME_BASE + CART_GAME_MAX_BYTES
                <= CART_SLOTTED_BANK_BASE_OFFSET,
                "game image window overlaps sub-slot 1 PSRAM region");
_Static_assert (CART_GAME_MAX_BYTES > 0U,
                "game image window is empty - sub-slot 1 region too large for PSRAM");
_Static_assert (PSRAM_CART_SIZE >= (CART_SLOTTED_BANK_BASE_OFFSET
                                    + (CART_SLOTTED_BANK_COUNT
                                       * CART_SLOTTED_BANK_SIZE)),
                "sub-slot 1 region must fit inside PSRAM_CART_SIZE");

/* Mapper selection. CART_MAP_NEXTOR is the only installable mapper.
 * CART_MAP_NONE is the pre-install state only (what Cart_GetMapper
 * reports before Cart_SetMapper has run): no handler is installed and
 * the EXTI0 bus interrupt is masked, so the cart data bus floats. */
typedef enum {
    CART_MAP_NONE       = 0,   /* pre-install state; NOT installable -
                                  Cart_SetMapper rejects it. The bus
                                  floats because EXTI0 stays masked. */
    CART_MAP_NEXTOR     = 1,   /* Nextor kernel mapper: ASCII16K over the
                                  embedded Nextor 3.0 kernel ROM
                                  (nextor_rom[], flash-resident). The
                                  16 KiB bank visible at 0x4000..0x7FFF
                                  is selected by writing the bank number
                                  to 0x6000; 0x7000/0x77FF do the same for
                                  page 2. The 0x7FF0..0x7FF5 mailbox is
                                  the kernel driver's only disk path.
                                  Expanded-slot mapper: secondary-slot
                                  register at FFFF (inverted read,
                                  M1-qualified latches), 0xFC..0xFF I/O
                                  bank registers latched by the EXTI9_5
                                  handler, sub-slot 0 = kernel ROM,
                                  sub-slot 1 = mapper RAM (1024 KiB
                                  PSRAM), sub-slots 2/3 empty. */
    CART_MAP_MAX        = 2,
} Cart_Mapper;

/* Install the Nextor mapper: resets all mapper/mailbox state, re-points
 * the PFIC VTF slot from the placeholder to Cart_EXTI0_Nextor_Handler,
 * and enables the PE8/~IORQ bank-register decoder. The EXTI0 bus
 * interrupt itself is armed by Init_Cart(); this call is idempotent.
 * Returns 0 on success, -1 on any mapper other than CART_MAP_NEXTOR.
 * Installed once during startup and never swapped. */
int  Cart_SetMapper (Cart_Mapper m);
Cart_Mapper Cart_GetMapper (void);

/* Embedded Nextor kernel ROM (flash-resident, .cartrom section).
 * 128 KiB = 8 banks x 16 KiB, served through the ASCII16K window. */
extern const uint8_t  nextor_rom[];
extern const uint32_t nextor_rom_len;

/* Initialise GPIO + EXTI routing for the cartridge bus, install the
 * no-mapper placeholder handler (floating bus) and arm the EXTI0 cart
 * interrupt - the first working firmware2 port's boot order.
 * Cart_SetMapper(NEXTOR), called immediately after, installs the real
 * handler and completes the mapper/mailbox state. The Nextor kernel
 * ROM is flash-served, so no PSRAM is required for the ROM path; the
 * mapper-RAM sub-slot stays disabled until PSRAM_Init() succeeds.
 *
 * NOTE: PE4 (MSX ~RESET) is configured here as a floating input only.
 * The firmware never drives it (read-only on most MSX2+ machines;
 * driving it externally can damage the mainboard).
 *
 * PE3 (~WAIT) is configured as a floating input during normal startup; this
 * firmware does not hold the MSX while peripherals initialize. */
void Init_Cart (void);

/* Cartridge ~WAIT line (PE3) - the Z80 bus-cycle stretch input. ~WAIT is
 * the cartridge's line to drive by MSX spec (the mainboard pulls it up),
 * so asserting it is safe on every machine - unlike ~RESET. Asserting it
 * freezes the Z80 inside its next bus cycle; wait states are transparent
 * to Z80 software, so the machine resumes with no state lost.
 *
 * The USB layer uses these helpers to bracket PSRAM DMA staging so a
 * DMA cannot race a Z80 bus cycle. Never hold the line for long
 * periods (the MSX is frozen while it is asserted).
 *
 * NOTE: a boot-time ~WAIT hold (assert until the cart is fully armed)
 * was tried and reverted - on the target MSX it made cold boots
 * consistently hang early; the first-port boot order (placeholder
 * handler, no hold) is the proven one. */
void Cart_HoldMSXWait_Begin (void);
void Cart_HoldMSXWait_End (void);

/* Optional boot diagnostics: see cart.c for the serve-latency
 * instrumentation these knobs enable. */
#ifndef CART_TIMING_STATS
#define CART_TIMING_STATS 0
#endif
#ifndef CART_LA_MARKER
#define CART_LA_MARKER 0
#endif

#if CART_TIMING_STATS
/* PSRAM mapper-RAM serve counters, incremented inside the bus interrupt;
 * printed by main()'s debug heartbeat. COUNTS ONLY by design: timestamp
 * or threshold instrumentation on the hot path adds enough latency to
 * break bus serving outright (the /firmware authors documented the same
 * regression). Timing/latency questions are answered externally with the
 * CART_LA_MARKER pulse on a logic analyser. */
extern volatile uint32_t g_cart_slotted_reads, g_cart_slotted_writes;
#endif

/* Enable/disable sub-slot 1 mapper RAM after its backing store is ready.
 * PSRAM is marked ready only after PSRAM_Init succeeds; the mapper remains
 * inactive until CART_MAP_NEXTOR is installed. */
void Cart_SetMapperRamReady (uint8_t ready);

#endif
