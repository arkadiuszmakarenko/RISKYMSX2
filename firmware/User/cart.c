#include "cart.h"
#include "psram.h"
#include "scc.h"
#include "loader.h"
#include "sunrise_ide.h"
#include "terminal.h"
#include "ch32v4x7.h"
#include "debug.h"

#pragma GCC push_options
#pragma GCC optimize("Ofast")


struct MSXState {
    uint32_t bankOffsets[16]; /* bank bias = (page * bankSize) - (Z80 - 0x4000) */
} s_state;


volatile Cart_Mapper g_mapper = CART_MAP_NONE;

static struct MSXState *const g_state = &s_state;
static void Cart_Banked_Dispatch (void) __attribute__((section(".ramfunc"), noinline,
                                                         interrupt("WCH-Interrupt-fast")));
static void RunKonamiNOSCC (void) __attribute__((section(".ramfunc"), noinline));
static void RunKonami  (void) __attribute__((section(".ramfunc"), noinline));
static void RunKonamiSCC (void) __attribute__((section(".ramfunc"), noinline));
static void Run8kASCII (void) __attribute__((section(".ramfunc"), noinline));
static void Run16kASCII(void) __attribute__((section(".ramfunc"), noinline));
static void RunNEO8    (void) __attribute__((section(".ramfunc"), noinline));
static void RunNEO16   (void) __attribute__((section(".ramfunc"), noinline));

#ifndef KONAMISCC_SCC_QUEUE
#define KONAMISCC_SCC_QUEUE 1
#endif


void Cart_EXTI0_KonamiNOSCC_Handler (void) __attribute__((section(".ramfunc"), noinline,
                                                            interrupt("WCH-Interrupt-fast"))) __attribute__((unused));
void Cart_EXTI0_ASCII8k_Handler (void) __attribute__((section(".ramfunc"), noinline,
                                                      interrupt("WCH-Interrupt-fast")));
void Cart_EXTI0_ASCII16k_Handler (void) __attribute__((section(".ramfunc"), noinline,
                                                       interrupt("WCH-Interrupt-fast")));
void Cart_EXTI0_ROM16k_Handler (void) __attribute__((section(".ramfunc"), noinline,
                                                       interrupt("WCH-Interrupt-fast")));
void Cart_EXTI0_ROM32k_Handler (void) __attribute__((section(".ramfunc"), noinline,
                                                       interrupt("WCH-Interrupt-fast")));
void Cart_EXTI0_ROM48k_Handler (void) __attribute__((section(".ramfunc"), noinline,
                                                       interrupt("WCH-Interrupt-fast")));




/* Embedded terminal ROM image (flash-resident, generated from
 * MSXSoftware/RomLoader/terminal.bin). Served by CART_MAP_TERMINAL
 * via Cart_EXTI0_Terminal_Handler. terminal_rom[] is sized to fit
 * just the actually-used bytes (~256 bytes: 0x4000..0x40FF) - the
 * rest of the 32 KiB cart window is floating-bus (0xFF) since the
 * MSX-side terminal program never fetches from 0x4100..0xBFFF. */
extern const uint8_t terminal_rom[];

/* Embedded Nextor Sunrise IDE kernel ROM (flash-resident, generated
 * from MSXSoftware/Nextor/nextor_sunrise.bin via the Makefile's
 * regenerate_nextor_rom target -> tools/rom2c.py). The Sunrise IDE
 * mapper decoder uses 8 banks x 16 KiB = 128 KiB; bank n of the Z80
 * window is selected by writing (bank|0x80) to 0x4104 (bit-reverse of
 * bits 0..2, IDE-enable in bit 7). See sunrise_ide.c::write_control
 * for the bit-reverse math.
 *
 * If MSXSoftware/Nextor/nextor_sunrise.bin is missing, the Makefile
 * falls back to the hand-written placeholder nextor_rom.c. That
 * placeholder will NOT boot Nextor - the Sunrise IDE mapper decode is
 * useless without a real Sunrise kernel image. The placeholder is
 * only there to keep the build green until the user drops the
 * correct ROM file in place. */
extern const uint8_t  nextor_rom[];
extern const uint32_t nextor_rom_len;
extern const uint32_t terminal_rom_len;

/* Flash-selector mapper: serves the embedded ROM-selector image
 * (selector_rom[]) for ordinary cart reads, and decodes the mailbox
 * window 0x7FF0..0x7FFF that the RAM-resident loader uses to talk to
 * the firmware (see loader.h / MSXSoftware/RomLoader). Keep this
 * handler in flash: the cart flash path is zero-wait, so there is no
 * reason to delay boot on PSRAM configuration just to reach the
 * selector. */

/* Terminal mapper: serves the embedded terminal ROM (terminal_rom[])
 * for ordinary cart reads, and decodes the v303-style 3-byte mailbox
 * window 0x7FFD/0x7FFE/0x7FFF. The MSX-side program (MSXSoftware/
 * RomLoader/asm/terminal.asm) drives the screen and forwards keystrokes
 * on 0x7FFD; the firmware hosts the menu logic (file list, ROM load,
 * mapper select) via the FIFO at 0x7FFF. Kept in flash: the cart
 * flash path is zero-wait, and the boot path benefits from having the
 * terminal ROM live before PSRAM is even initialised. */
void Cart_EXTI0_Sunride_Handler (void) __attribute__((noinline,
                                                      interrupt("WCH-Interrupt-fast")));
void Cart_EXTI0_Terminal_Handler (void) __attribute__((noinline,
                                                        interrupt("WCH-Interrupt-fast")));

/* No-mapper fallback: just clears the pending bit and releases the bus.
 * The Z80 reads 0xFF (floating bus). */
void Cart_EXTI0_None_Handler (void) __attribute__((section(".ramfunc"), noinline,
                                                    interrupt("WCH-Interrupt-fast")));

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

const char *const Cart_MapperNames[CART_MAP_MAX] = {
    "NONE",
    "ROM16k",
    "ROM32k",
    "ROM48k",
    "KONAMI",
    "KONAMINOSCC",
    "ASCII8k",
    "ASCII16k",
    "NEO8",
    "NEO16",
    "KONAMISCC",
    "FLASH",
    "TERMINAL",
    "SUNRIDE",
};

Cart_Mapper Cart_GetMapper (void) { return g_mapper; }

/* Diagnostics (currently DORMANT - nothing increments these): an
 * earlier instrumented build incremented them from inside the
 * .ramfunc EXTI0 handlers, but a volatile global RMW (~5-8 cycles)
 * on the cycle-critical read path (must drive the Z80 data bus
 * before its sampling point) broke the BIOS slot probe ("games
 * don't boot" regression). Keep the symbols for a future LA-gated
 * variant, but NEVER increment from the EXTI0 hot path. */
volatile uint32_t g_cart_rd_cycles = 0U;
volatile uint32_t g_cart_wr_cycles = 0U;
uint32_t Cart_GetRdCycles (void) { return g_cart_rd_cycles; }
uint32_t Cart_GetWrCycles (void) { return g_cart_wr_cycles; }

uint32_t Cart_GetImageBase (void) { return PSRAM_CART_BASE; }
uint32_t Cart_GetImageSize (void) { return PSRAM_CART_SIZE; }
uint32_t Cart_GetGameBase (void) { return CART_GAME_BASE; }

/* ------------------------------------------------------------------ */
/* Cart_SetMapper_Safe                                                */
/* ------------------------------------------------------------------ */
/* Hardened mapper-swap primitive. The bare `Cart_SetMapper()` writes
 * `g_mapper`, then rewrites VTF slot 0 (`SetVTFIRQ` writes VTFADDR[0]),
 * then mutates `bankOffsets[]`. Three independent writes. A `~SLTSL`
 * falling edge that latches while any of them is in flight gets served
 * by EITHER:
 *   - the old VTF entry (slot not yet rewritten) reading the new
 *     `g_mapper` -- handler-state mismatch, can drive stale bytes.
 *   - the new VTF entry (slot rewritten) reading bankOffsets[] that
 *     was zeroed for the swap but not yet re-initialised -- serves
 *     `0xFF` to the Z80 (`RST 38h`).
 *   - either handler while the data bus is tri-stated but a write is
 *     queued for the old mapper's bank (writes go to the wrong page).
 *
 * Cart_SetMapper_Safe() closes the race window by:
 *   1. Disabling EXTI0 + global IRQ (no IRQ can fire mid-swap).
 *   2. Spinning until `~SLTSL` is HIGH (no in-flight cart cycle) with
 *      a bounded timeout; if it never goes high we still drive the
 *      bus off and proceed - better to lose one cycle than wedge.
 *   3. Driving the data bus OFF (`GPIOB->CFGHR = CART_BUS_OFF`) so no
 *      stale OUTDR value lingers through the swap.
 *   4. Calling the pure `Cart_SetMapper()` to do the actual swap.
 *   5. Clearing `EXTI->INTFR` (any phantom edge latched during the
 *      wait).
 *   6. Issuing `__DSB(); __ISB();` so the new VTFADDR[0] is visible
 *      to the core's pipeline before the IRQ is re-enabled.
 *   7. Re-enabling IRQ.
 *
 * NOTE: an earlier version of this function also drove MSX ~RESET low
 * for the duration of the swap (hold_msx_reset=1 from the loader's
 * CMD_RESET flow). That path is REMOVED: most MSX2+ machines expose
 * the cart-edge ~RESET as read-only and driving it externally can
 * damage the reset circuit. The CMD_SOFTRESET slingshot (running
 * from MSX RAM) provides the equivalent atomicity without touching
 * PE4 - see cmd_softreset in loader.c.
 *
 * Returns the same value as Cart_SetMapper(). */
int Cart_SetMapper_Safe (Cart_Mapper m);
int Cart_SetMapper_Safe (Cart_Mapper m) {
    printf ("CART: SetMapper_Safe -> %d\r\n", (int)m);
    /* Phase 1: stop the IRQ. The order matters:
     *   - EXTI0's pending bit is in EXTI->INTFR (edge-triggered on
     *     PE0 falling edge). Masking it via EXTI->INTENR bit is
     *     harmless but adds a register touch we'd have to undo.
     *   - PFIC->IER[EXTI0_IRQn] (NVIC_DisableIRQ) is enough to prevent
     *     the IRQ from dispatching. The pending EXTI0 edge stays in
     *     EXTI->INTFR and we'll clear it in phase 5.
     *   - `__disable_irq()` masks everything (also blocks TIM4/SCC
     *     pump; we want THAT paused too, see Cart_EXTI0_SCC_QueueWrite
     *     being unpaused mid-swap would race with our bankOffsets[]
     *     rewrite). */
    NVIC_DisableIRQ (EXTI0_IRQn);
    __disable_irq ();
    /* Full-memory DSB: drain any in-flight writes (including the
     * SetVTFIRQ register write inside Cart_SetMapper) before we
     * observe state. RV32 doesn't have a DSB/ISB builtin symbol in
     * WCH's core_riscv.h, emit the fences inline. */
    __asm__ volatile ("fence iorw, iorw");
    __asm__ volatile ("fence.i");

    /* Phase 2: wait for ~SLTSL to go high (no in-flight Z80 cycle).
     * Bounded loop so a stuck-low ~SLTSL doesn't wedge the firmware;
     * we still drive the bus off and proceed after the timeout. */
    {
        uint32_t spin = 200000U;       /* ~2 ms at 200 MHz HCLK      */
        while (((GPIOE->INDR & CART_SLTSL_MASK) == 0U) && (--spin)) {
            __asm__ volatile ("nop");
        }
    }
    /* Phase 3: drive the data bus tri-stated, in case the handler
     * left it on (e.g. crashed inside a `while (SLTSL low)` spin). */
    GPIOB->CFGHR = CART_BUS_OFF;

    /* Phase 4: the actual swap (handler + bankOffsets + g_mapper). */
    const int rc = Cart_SetMapper (m);
    printf ("CART: SetMapper(%d) -> rc=%d\r\n", (int)m, rc);

    /* Phase 5: clear any phantom EXTI0 edge that latched during the
     * wait. Writing 1 to the bit clears it (WCH edge-triggered IRQ
     * design). */
    EXTI->INTFR = EXTI_INTENR_MR0;

    /* Phase 6: serialise the writes - the VTFADDR[0] write inside
     * SetVTFIRQ needs an ISB so the next IRQ observable by the core
     * sees the new handler address. */
    __asm__ volatile ("fence iorw, iorw");
    __asm__ volatile ("fence.i");

    /* Phase 7: re-arm the IRQ path. */
    NVIC_EnableIRQ (EXTI0_IRQn);
    __enable_irq ();

    return rc;
}

int Cart_SetMapper (Cart_Mapper m) {
    if ((unsigned)m >= CART_MAP_MAX) return -1;
    if (m != CART_MAP_NONE
        && m != CART_MAP_ROM16k
        && m != CART_MAP_ROM32k
        && m != CART_MAP_ROM48k
        && m != CART_MAP_FLASH
        && m != CART_MAP_TERMINAL
        && m != CART_MAP_SUNRIDE
        && PSRAM_GetRomMirrorBase() == 0U) return -1;

    /* Reset bank state so a switch from one mapper to another does not
     * leak stale offsets. Only the bank-switching mappers touch these;
     * the simple ROM mappers ignore them entirely. */
    for (unsigned i = 0; i < 16; i++) {
        s_state.bankOffsets[i] = 0U;
    }

    g_mapper = m;

    /* Install the matching handler in VTF slot 0. SetVTFIRQ writes
     * VTFADDR[0] and VTFIDR[0] - the VTF dispatcher uses VTFADDR[0]
     * when EXTI0 fires (the IRQ number is matched by VTFIDR[0]). */
    uint32_t h = (uint32_t)Cart_EXTI0_None_Handler;
    switch (m) {
    case CART_MAP_NONE:        h = (uint32_t)Cart_EXTI0_None_Handler; break;
    case CART_MAP_ROM16k:      h = (uint32_t)Cart_EXTI0_ROM16k_Handler; break;
    case CART_MAP_ROM32k:      h = (uint32_t)Cart_EXTI0_ROM32k_Handler; break;
    case CART_MAP_ROM48k:      h = (uint32_t)Cart_EXTI0_ROM48k_Handler; break;
    case CART_MAP_KONAMI:
    case CART_MAP_KONAMISCC:
    case CART_MAP_KONAMINOSCC:
    case CART_MAP_NEO8:
    case CART_MAP_NEO16:       h = (uint32_t)Cart_Banked_Dispatch; break;
    case CART_MAP_ASCII8k:     h = (uint32_t)Cart_EXTI0_ASCII8k_Handler; break;
    case CART_MAP_ASCII16k:    h = (uint32_t)Cart_EXTI0_ASCII16k_Handler; break;

    case CART_MAP_TERMINAL:   h = (uint32_t)Cart_EXTI0_Terminal_Handler; break;
    case CART_MAP_SUNRIDE:    h = (uint32_t)Cart_EXTI0_Sunride_Handler; break;
    default:                   return -1;
    }
    SetVTFIRQ (h, EXTI0_IRQn, 0, ENABLE);

    /* Flash selector mappers: reset the mailbox so the loader starts clean. */
    if (m == CART_MAP_ROM32k || m == CART_MAP_FLASH) {
        Loader_Reset ();
    }
    /* Sunrise IDE mapper: reset the ATA state machine + PATA device
     * signature + IDE USB lifecycle so the kernel starts with a clean
     * engine (no stale LBA / sector buffer / IDENTIFY data from a
     * previous run, no stale state from a half-completed READ/WRITE).
     * Safe to call repeatedly. */
    if (m == CART_MAP_SUNRIDE) {
        Sunrise_IDE_Init ();
    }

    /* Initial-bank assignment for bank-switching mappers so the first
     * Z80 read sees bank 0 (or bank 2 for Konami-with-SCC-mirror-less
     * layouts). Mirrors the legacy v303 firmware defaults. */
    if (m == CART_MAP_KONAMI) {
        /* Konami mapper. Bank slots 2..5 cover Z80 0x4000..0xBFFF via
         * (addr >> 13) indexing. Bias[i] = -page_start so a read at
         * the page's first address lands at PSRAM offset 0.
         *
         * The legacy v303 firmware had bias[3] = -0x8000, bias[4..5]
         * = 0 - those values worked because that firmware's cart
         * image was in flash (where negative biases wrap within the
         * cart region). With PSRAM at 0x80000000, those biases under-
         * flow past 0x80000000 into unmapped space and fault. Use
         * biases that map each page to PSRAM offset 0. */
        s_state.bankOffsets[2] = 0U - 0x4000U;  /* page 1 -> bank 0 */
        s_state.bankOffsets[3] = 0U - 0x6000U;  /* page 2 -> bank 0 */
        s_state.bankOffsets[4] = 0U - 0x8000U;  /* page 3 -> bank 0 */
        s_state.bankOffsets[5] = 0U - 0xA000U;  /* page 4 -> bank 0 */
    } else if (m == CART_MAP_KONAMINOSCC) {
        /* Konami-without-SCC: bank-switch writes accepted at ANY
         * address in 0x4000..0xBFFF (not just 0x6000/0x8000/0xA000).
         * Uses the SAME KonamiUltimateCollection reset layout as
         * KONAMISCC (pages 1..4 -> banks 0..3) so carts whose init
         * LDIRs from page 2+ before the first bank write (Nemesis,
         * Space Manbow, Metal Gear 2, ...) read the right bytes at
         * boot. The old "all pages -> bank 0" bias (-0x6000/
         * -0x8000/-0xA000) regressed once (2026-09-19) and froze
         * >256 KiB Konami carts on a blue screen - keep ALL four
         * biases at -0x4000. The plain CART_MAP_KONAMI branch keeps
         * its own page-2..4->bank-0 layout; do not "fix" that one. */
        s_state.bankOffsets[2] = 0U - 0x4000U;  /* page 1 -> bank 0 */
        s_state.bankOffsets[3] = 0U - 0x4000U;  /* page 2 -> bank 1 */
        s_state.bankOffsets[4] = 0U - 0x4000U;  /* page 3 -> bank 2 */
        s_state.bankOffsets[5] = 0U - 0x4000U;  /* page 4 -> bank 3 */
    } else if (m == CART_MAP_KONAMISCC) {
        /* Konami-with-SCC (WebMSX 6.x CartridgeKonamiUltimateCollection
         * reset state). Pages 1..4 pre-map to image banks 0..3 (each
         * page gets a DISTINCT 8 KiB bank) so the cart's init code in
         * pages 2..4 reads the right bytes WITHOUT a bank-switch write.
         * Required by Konami-SCC carts whose init LDIRs from page 2+
         * before any bank write (Nemesis, Space Manbow, Metal Gear 2,
         * etc.). Plain Konami-without-SCC carts would BREAK with this
         * layout - they assume all pages read bank 0 at boot (see the
         * CART_MAP_KONAMI / CART_MAP_KONAMINOSCC branches above). */
        s_state.bankOffsets[2] = 0U - 0x4000U;  /* page 1 -> bank 0 */
        s_state.bankOffsets[3] = 0U - 0x4000U;  /* page 2 -> bank 1 */
        s_state.bankOffsets[4] = 0U - 0x4000U;  /* page 3 -> bank 2 */
        s_state.bankOffsets[5] = 0U - 0x4000U;  /* page 4 -> bank 3 */
        /* Bring up the SCC emulator core (emu2212) + its DMA->DAC
         * sample pump. scc.c guards the hardware config (RCC + DMA2
         * + TIM4 + DAC1) with a `s_scc_hw_up` flag - the first call
         * configures the registers, every subsequent call only
         * resets the emulator + clears the SPSC queue. Re-running
         * the full hardware setup while TIM4_IRQHandler / DMA2_Ch3
         * are live would latch the DMA state machine and freeze the
         * chip, which is exactly what was happening before (the
         * boot path calls SCC_Init() once from main() and again from
         * here). See scc.c::SCC_Init / s_scc_hw_up. */
        (void)SCC_Init();
    } else if (m == CART_MAP_ASCII8k) {
        /* ASCII 8k: 8 KiB banks at 0x6000/0x6800/0x7000/0x7800 (write
         * addresses). Matches the legacy v303 firmware defaults exactly
         * so carts that don't issue a bank-select write before reading
         * still see image[0] at the standard read addresses. */
        s_state.bankOffsets[0] = 0U - 0x4000U;
        s_state.bankOffsets[1] = 0U - 0x6000U;
        s_state.bankOffsets[2] = 0U - 0x8000U;
        s_state.bankOffsets[3] = 0U - 0xA000U;
    } else if (m == CART_MAP_ASCII16k) {
        /* ASCII 16k: 16 KiB banks at 0x6000 and 0x7000. */
        s_state.bankOffsets[0] = 0x0000U - 0x4000U;
        s_state.bankOffsets[8] = 0x0000U - 0x8000U;
    }
    /* NEO8/NEO16 default to bank 0 for all pages - their writes
     * compose the 12-bit bank number, no init needed.
     *
     * CART_MAP_SUNRIDE does NOT touch bankOffsets[] - the Sunrise IDE
     * mapper decode lives entirely inside sunrise_ide.c (the cart IRQ
     * handler just routes the read/write to Sunrise_IDE_ReadByte /
     * Sunrise_IDE_WriteByte). The bank state is held in the
     * s_ide.segment byte inside sunrise_ide.c and pre-selected by
     * Sunrise_IDE_Init() (called above). */

    return 0;
}

/* ------------------------------------------------------------------ */
/* Init_Cart - GPIO + EXTI wiring                                      */
/* ------------------------------------------------------------------ */

void Init_Cart (void) {
    /* Enable GPIO + AFIO clocks */
    RCC_PB2PeriphClockCmd (RCC_PB2Periph_GPIOA |
                               RCC_PB2Periph_GPIOB |
                               RCC_PB2Periph_GPIOD |
                               RCC_PB2Periph_GPIOE |
                               RCC_PB2Periph_AFIO,
                           ENABLE);

    /* Address bus PD0..PD15 as floating inputs. */
    GPIOD->CFGLR = 0x44444444U;
    GPIOD->CFGHR = 0x44444444U;

    /* Control bus PE0(SLTSL) PE1(RD) PE2(WR) PE5(MREQ) as floating
     * inputs. PE3 = WAIT (left floating). PE4 = MSX ~RESET (floating
     * - the firmware never drives it; see the cart.h note near
     * Init_Cart). */
    GPIOE->CFGLR = 0x44444444U;

    /* Data bus PB8..PB15 tri-stated (bus off) */
    GPIOB->CFGHR = CART_BUS_OFF;

    /* LEDFLASH PA0 as push-pull output, 50 MHz slew */
    GPIOA->CFGLR &= ~(0xFu << 0);
    GPIOA->CFGLR |=  (0x3u << 0);

    /* Route PE0 -> EXTI0, FALLING edge only. One IRQ per Z80 bus
     * cycle: the handler serves the byte AND releases the bus inside
     * the same invocation (spins until SLTSL rises), so no rising-
     * edge IRQ is needed. */
    AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~(0xFU << 0)) |
                      (AFIO_EXTICR1_EXTI0_PE << 0);
    EXTI->INTENR = (EXTI->INTENR & ~EXTI_INTENR_MR0) | EXTI_INTENR_MR0;
    EXTI->RTENR &= ~EXTI_RTENR_TR0;
    EXTI->FTENR = (EXTI->FTENR & ~EXTI_FTENR_TR0) | EXTI_FTENR_TR0;
    EXTI->INTFR = EXTI_INTENR_MR0;

    /* The legacy ASCII16+mailbox NEXTOR mapper used a Cart_EXTI95_IORQ
     * handler on PE8 (/IORQ) to decode ports 0xFC..0xFF and adopt the
     * cart as the kernel's primary memory mapper. The Sunrise IDE
     * kernel uses an internal mapper (RAM-backed, not a cart mapper)
     * so it never touches 0xFC..0xFF; the IDE register / data window
     * (0x7C00..0x7EFF) is decoded purely by address inside the cart
     * EXTI0 handler. The IORQ decoder is therefore removed.
     *
     * PE8 is still wired on the cart edge and not used; the EXTI8 path
     * is left disabled so spurious /IORQ transitions from other MSX
     * expansions do not generate spurious IRQs. */

    /* Install the no-mapper handler by default. main()/CLI installs the
     * real mapper via Cart_SetMapper() once PSRAM_Init() has succeeded.
     * SetVTFIRQ writes VTFADDR[0]; NVIC_EnableIRQ sets the per-IRQ
     * enable bit in PFIC->IENR. Both must be done for VTF dispatch to
     * fire. */
    SetVTFIRQ ((uint32_t)Cart_EXTI0_None_Handler, EXTI0_IRQn, 0, ENABLE);
    NVIC_EnableIRQ (EXTI0_IRQn);
    NVIC_SetPriority (EXTI0_IRQn, 0x00);
    __enable_irq();
}

/* ------------------------------------------------------------------ */
/* MSX ~RESET line (PE4) - NOT driven by this firmware.                */
/* ------------------------------------------------------------------ */
/* The cart edge on most MSX2+ machines exposes ~RESET as a read-only
 * signal (the reset circuit lives inside the mainboard; the cart
 * edge is either unconnected or input-only). Driving PE4 low here
 * would not reboot those MSXs anyway, and on some designs it can
 * damage the mainboard's reset driver.
 *
 * PE4 is therefore left as a floating input at boot (Init_Cart below)
 * and is NEVER driven by Cart_SetMapper_Safe or any other firmware
 * path. The MSX-side loader's CMD_SOFTRESET slingshot
 * (loader.c / romloader.c) provides the reboot without touching
 * PE4. */

/* ------------------------------------------------------------------ */
/* Shared C-side helpers used by the bank-switching mappers.           */
/* ------------------------------------------------------------------ */

/* Drive one byte from PSRAM onto GPIOB[15:8] and turn on the drivers. */
static inline __attribute__((always_inline))
void Cart_DriveByteFromPSRAM (uint32_t addr, uint32_t bias) {
    /* Read the byte with the bus STILL TRI-STATED. PSRAM read takes
     * ~30 cycles; the old handler relied on this exact order to avoid
     * driving a stale OUTDR value during the controller latency.
     * CART_GAME_BASE (compile-time, cart.h) shifts the whole image
     * within the PSRAM window - every read path adds the same
     * constant, so the image moves as one block. */
    uint8_t b = *(const volatile uint8_t *)
        (PSRAM_CART_BASE + CART_GAME_BASE + addr + bias);
    GPIOB->OUTDR = (uint32_t)b << 8;
    GPIOB->CFGHR = CART_BUS_ON;
}

/* Inner C handler: clear the EXTI0 pending bit, spin until SLTSL
 * rises, release the data bus. All bank-switching handlers end with
 * this. */
static inline __attribute__((always_inline))
void Cart_EndCycle (void) {
    EXTI->INTFR = EXTI_INTENR_MR0;
    /* Release spin: poll GPIOE->INDR bit 0 until SLTSL goes high. */
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
    GPIOB->CFGHR = CART_BUS_OFF;
}

/* Read the data-bus byte (write-data from Z80) on the way into a write
 * cycle. Bits land in PB[15:8] which is the upper byte of GPIOB->INDR. */
static inline __attribute__((always_inline))
uint8_t Cart_ReadWriteData (void) {
    return (uint8_t)((GPIOB->INDR >> 8) & 0xFFU);
}

/* ------------------------------------------------------------------ */
/* Bank-switching mapper handlers (C, .ramfunc).                       */
/* ------------------------------------------------------------------ */

/*
 * Konami mapper (no SCC).
 *
 *   page 0 (0x0000..0x3FFF) : BIOS - cart does not respond.
 *   page 1 (0x4000..0x7FFF) : fixed 16 KiB from image[0..0x3FFF].
 *   page 2 (0x8000..0xBFFF) : user-selected 8 KiB bank (low half).
 *   page 3 (0xC000..0xFFFF) : user-selected 8 KiB bank (high half).
 *
 *   writes:
 *     0x6000 -> select page-2 low bank
 *     0x8000 -> select page-3 low bank
 *     0xA000 -> select page-3 high bank
 *
 *   "Bank" is 8 KiB; bank byte << 13 gives the image offset.
 *   bias[i] = (bank << 13) - (Z80_page_start - 0x4000) where
 *   Z80_page_start = i * 0x2000.
 */
static void RunKonami (void) {
    /* Debug: toggle PA0 so we can see on the LA that this fired. */
    GPIOA->OUTDR ^= (1u << 0);
    const uint16_t address = (uint16_t)GPIOD->INDR;
    const uint32_t ctrl   = GPIOE->INDR;
    const uint32_t bank   = address >> 13;  /* 2..3 valid, 0..1 ignored */

    if ((ctrl & CART_RD_MASK) == 0U) {
        /* READ cycle. Drive one byte from PSRAM onto the bus. */
        if (bank >= 2U && bank <= 3U) {
            const uint32_t bias = g_state->bankOffsets[bank];
            Cart_DriveByteFromPSRAM (address, bias);
        } else {
            /* Page 0/1 (BIOS / cart header) - cart does not drive the
             * bus; release and let the MSX's own devices respond. */
            GPIOB->CFGHR = CART_BUS_OFF;
        }
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        /* Bus already released above for the BIOS case; for the cart
         * case the read is done so we leave it driven until SLTSL
         * rises. */
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* WRITE cycle: only meaningful at 0x6000/0x8000/0xA000. WR is
     * active-low so we test == 0. The Z80 may also pulse WR with
     * SLTSL low for addresses we don't care about - ignore. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        if ((GPIOE->INDR & CART_WR_MASK) == 0U) {
            const uint8_t w = Cart_ReadWriteData();
            /* Bank 0 -> page-2 low (offset 2). Bank 1 -> page-2 high.
             * Bank 2 -> page-3 low (offset 4). Bank 3 -> page-3 high
             * (offset 5). */
            switch (address) {
            case 0x6000: g_state->bankOffsets[3] = ((uint32_t)w << 13) - 0x6000U; break;
            case 0x8000: g_state->bankOffsets[4] = ((uint32_t)w << 13) - 0x8000U; break;
            case 0xA000: g_state->bankOffsets[5] = ((uint32_t)w << 13) - 0xA000U; break;
            default: break;
            }
            return;
        }
    }
}

/*
 * Konami mapper "with SCC-NOSCC" variant.
 *
 * Same bank layout as KONAMI (page 0=BIOS, page 1=fixed 16 KiB header,
 * pages 2..3 = user-selectable 8 KiB banks), BUT any write to the
 * cart's address range 0x4000..0xBFFF selects the bank for that page
 * (slot = address >> 13). This is needed by cartridges that issue
 * bank-switch writes from anywhere in the page, not just the three
 * standard switch addresses 0x6000/0x8000/0xA000.
 *
 * Bank bias: bankOffsets[page] = (data << 13) - page_start, where
 * page_start = page * 0x2000.
 *
 * NOTE: this C body is kept as a reference / fallback. The asm
 * Cart_EXTI0_KonamiNOSCC_Handler is installed directly in the VTF
 * slot for KONAMINOSCC, so this C function is never called at runtime.
 */
static void RunKonamiNOSCC (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    const uint32_t ctrl    = GPIOE->INDR;
    const uint32_t page    = address >> 13;

    if ((ctrl & CART_RD_MASK) == 0U) {
        if (page >= 2U && page <= 5U) {
            const uint32_t bias = g_state->bankOffsets[page];
            Cart_DriveByteFromPSRAM (address, bias);
        } else {
            GPIOB->CFGHR = CART_BUS_OFF;
        }
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    EXTI->INTFR = EXTI_INTENR_MR0;
    if (address > 0xB000U) return;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        if ((GPIOE->INDR & CART_WR_MASK) == 0U) {
            const uint8_t w = Cart_ReadWriteData();
            const uint32_t page_start = (uint32_t)(address & 0xE000U);
            g_state->bankOffsets[page] = ((uint32_t)w << 13) - page_start;
            return;
        }
    }
}

/*
 * Konami mapper with SCC sound chip (port of legacy v303
 * RunKonamiWithSCC, with the page-filter discipline aligned to
 * RunKonamiNOSCC).
 *
 * This handler is the LINE-UP of RunKonamiNOSCC + SCC write-queuing.
 * The two used to be subtly different - NOSCC refused to drive the
 * bus on page 0/1 reads, SCC drove every slot from 0..7, which let the
 * cart image leak into the BIOS address range (page 0 = 0x0000..0x3FFF)
 * and crashed the MSX with "goes to RESET" the moment the BIOS probed
 * the cart for an 'AB' header. After aligning the filter with NOSCC,
 * KONAMISCC and KONAMINOSCC behave identically apart from the SCC
 * write-path.
 *
 * The four boot biases are shared with KONAMINOSCC
 * (KonamiUltimateCollection reset state: pages 1..4 -> banks 0..3, all
 * biases = -0x4000). SCC_Init() is called once from Cart_SetMapper().
 *
 * Semantics:
 *   READ cycle at 0x4000..0xBFFF (page 2..5):
 *      drive PSRAM byte via bankOffsets[page], like NOSCC.
 *   READ cycle at any other address (page 0 BIOS, page 1 cart header
 *      mirrored, page 6/7 'unused'):
 *      bus off - the BIOS owns the bus, the cart is electrically
 *      silent (no bus contention).
 *   WRITE cycle at 0x4000..0xBFFF:
 *      update bankOffsets[page] = (w << 13) - page_start, AND queue
 *      (address<<16 | data) for the SCC emulator BUT only if the
 *      write landed inside the SCC-I register window
 *      (0x9800..0x98FF). Writes anywhere else in the cart window are
 *      bank-switch writes - the SCC doesn't care, the TIM4 IRQ would
 *      just ignore them, and queueing them wastes queue slots.
 *   WRITE cycle at 0x0000..0x3FFF (BIOS page 0 / RAM 0xC000..0xFFFF):
 *      return without touching bankOffsets[] (the BIOS is writing its
 *      own scratch RAM, the cart is NOT selected). Matches NOSCC.
 *
 * The bias formula (w << 13) - page_start (where page_start =
 * (address & 0xE000)) is identical to NOSCC's and produces the same
 * bias for the standard bank-switch addresses 0x6000/0x8000/0xA000:
 * page_start == 0x6000 / 0x8000 / 0xA000, so the formula collapses to
 * (w << 13) - 0x6000 / etc., the same as if the legacy v303 used the
 * (w << 13) - (address - 0x1000) form. For SCC-window writes
 * (0x9800..0x98FF) the two forms differ: v303 produced (w << 13) -
 * 0x8800 (because address - 0x1000 = 0x8800 for 0x9800); the aligned
 * form produces (w << 13) - 0x9800. The difference is moot because
 * SCC_window writes do NOT update bankOffsets[slot=4] in any version
 * of the formula that respects the page boundary (any cart test that
 * writes to 0x9800-0x98FF and immediately reads back is broken by
 * design - those writes go to the SCC and reads come from the cart
 * image, NOT from a re-mapped bank).
 *
 * The SCC IRQ path: writes queued by SCC_QueueWrite are drained by
 * the TIM4 IRQ (scc.c) at ~44 kHz into SCC_write(), which ignores any
 * write falling outside the SCC's `base_adr..base_adr+0x100` window.
 * The queue is a 64-entry SPSC ring in zero-wait-state SRAM.
 *
 * Cycle budget (write path, SCC-window hit case vs. plain bank-switch):
 *   bank-switch write:  ~6 extra cycles on top of NOSCC (the
 *                        ANDI-XORI mask check + skipped branch).
 *   SCC-window write:   ~10 extra cycles (mask-check taken + sll/orr +
 *                        SCC_QueueWrite jal: 1 SUB-equivalent
 *                        compare + 1 sll + 1 orr + 1 jal + 1 ret-like).
 *   non-cart-page write (0x0000..0x3FFF, 0xC000..0xFFFF): 0 extra
 *                        cycles - filtered at the top of the write
 *                        path the same way NOSCC does.
 * The hot read-path is byte-for-byte NOSCC (no SCC touch at all
 * on reads - cart image is what the BIOS wants, not the SCC).
 */
static void RunKonamiSCC (void) {
    /* Common prologue - shared with RunKonamiNOSCC. The address +
     * page extraction is hot enough that we inlined it here rather
     * than factoring into a helper (helper call would cost a jal +
     * return + an extra register spill). */
    const uint16_t address = (uint16_t)GPIOD->INDR;
    const uint32_t ctrl    = GPIOE->INDR;
    const uint32_t page    = address >> 13;

    /* ---- READ cycle (byte-for-byte identical to RunKonamiNOSCC). */
    if ((ctrl & CART_RD_MASK) == 0U) {
        if (page >= 2U && page <= 5U) {
            const uint32_t bias = g_state->bankOffsets[page];
            Cart_DriveByteFromPSRAM (address, bias);
        } else {
            GPIOB->CFGHR = CART_BUS_OFF;
        }
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* ---- WRITE cycle. Filter non-cart addresses identically to
     *      RunKonamiNOSCC: `address > 0xB000U` catches page 0 (BIOS
     *      0x0000..0x3FFF), page 6 + 7 (high RAM 0xC000..0xFFFF) in
     *      one unsigned SLTIU compare against an immediate - cheaper
     *      than a pair of page-bound compares plus an OR. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    if (address > 0xB000U) return;

    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        if ((GPIOE->INDR & CART_WR_MASK) == 0U) {
            /* Latch data bus ONCE while ~WR is low (Z80 only drives
             * the data bus during the ~WR pulse; reading earlier
             * latches stale / floating garbage, which corrupts both
             * the bank value AND the SCC queue entry - that was the
             * second KONAMISCC freeze cause, fixed here by latching
             * inside the WR-low block, matching NOSCC exactly). */
            const uint8_t w = Cart_ReadWriteData();

            /* Common bank-switch update - identical formula to
             * RunKonamiNOSCC. (w << 13) - page_start where
             * page_start = (address & 0xE000). */
            const uint32_t page_start = (uint32_t)(address & 0xE000U);
            g_state->bankOffsets[page] = ((uint32_t)w << 13) - page_start;

            /* SCC-window test: a single ANDI against 0xF000 plus a
             * branch on equal-to-0x9000 catches the entire
             * 0x9000..0x9FFF SCC window:
             *   0x9000..0x97FF  SCC enable / SCC+ activation byte
             *                    (offset 0 from base_adr; emu2212
             *                    treats both as the same write)
             *   0x9800..0x9FFE  SCC control registers (offset
             *                    0x800..0x8FE: wave, freq, vol,
             *                    mode, ch-enable)
             * Cheaper than two loads against SCC_I_WINDOW_BASE /
             * LAST (those constants live in flash; loading them
             * from flash inside the IRQ window costs a flash
             * waitstate at HCLK 200 MHz). Cheaper than the legacy
             * ">= && <=" two-compare form. Bank-switch writes
             * (addresses with high nibble != 0x9) skip the queue
             * path entirely - TIM4 would just drop them anyway. */
            if ((address & 0xF000U) == 0x9000U) {
                /* Pack: bits[31:16] = address, bits[7:0] = w.
                 * `address` is already a uint16_t so the shift can
                 * stay in the lower 32; `w` is uint8_t so the OR is
                 * a single-byte write. */
                const uint32_t packed =
                    ((uint32_t)address << 16) | (uint32_t)w;
                (void)SCC_QueueWrite (packed);   /* drop on overflow */
            }
            return;
        }
    }
}

/*
 * ASCII 8k mapper (port of legacy v303 Run8kASCII).
 *
 * Four 8 KiB windows cover 0x4000..0xBFFF. Window N (8 KiB starting at
 * 0x4000 + 0x2000*N) is served from bankOffsets[N]; its bank register
 * is the 2 KiB-aligned write address 0x4000 + 0x2000*N, i.e. any write
 * into that window's own 2 KiB page - 0x6000/0x6800/0x7000/0x7800 for
 * the standard registers (slot = (addr >> 11) & 3):
 *
 *   0x4000..0x5FFF <- bankOffsets[0], register @ 0x6000
 *   0x6000..0x7FFF <- bankOffsets[1], register @ 0x6800
 *   0x8000..0x9FFF <- bankOffsets[2], register @ 0x7000
 *   0xA000..0xBFFF <- bankOffsets[3], register @ 0x7800
 *
 * The boot biases pre-map each window to image offset 0 (so the 'AB'
 * header is visible at 0x4000): [0]=-0x4000 [1]=-0x6000 [2]=-0x8000
 * [3]=-0xA000, identical to the v303 defaults.
 */
static void Run8kASCII (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    const uint32_t ctrl    = GPIOE->INDR;

    /* Cycle qualifier: the cart is a memory-mapped device and must NOT
     * respond on ~IORQ (port I/O), INTA, RFSH, or any other cycle type
     * that happens to assert SLTSL. ~MREQ is the canonical "this is a
     * memory access" signal - low during memory read/write and during
     * opcode fetch (M1). If MREQ is high, release the bus and clear
     * INTFR; the slot's other devices (or the bus itself) handle it. */
    if ((ctrl & CART_MREQ_MASK) != 0U) {
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    if ((ctrl & CART_RD_MASK) == 0U) {
        /* READ. Read slot = v303 formula ((addr >> 12) - 4) >> 1, i.e.
         * the 8 KiB window starting at 0x4000 + 0x2000*slot is served
         * from bankOffsets[slot] - the bank register written at
         * 0x6000/0x6800/0x7000/0x7800 respectively:
         *   0x4000..0x5FFF -> slot 0 (register @ 0x6000)
         *   0x6000..0x7FFF -> slot 1 (register @ 0x6800)
         *   0x8000..0x9FFF -> slot 2 (register @ 0x7000)
         *   0xA000..0xBFFF -> slot 3 (register @ 0x7800)
         * 0x4000..0x5FFF IS bank-switched on ASCII 8k: the cart header
         * ('AB') lives there, so slot 0 must be served or the MSX BIOS
         * reads 0xFF at 0x4000 and never detects the cartridge. */
        if (address < 0x4000U || address >= 0xC000U) {
            /* Pages 0/3 belong to the BIOS/RAM. SLTSL should never
             * assert there; if it ever does, keep the bus off so the
             * real device answers. */
            GPIOB->CFGHR = CART_BUS_OFF;
            EXTI->INTFR  = EXTI_INTENR_MR0;
            while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
            return;
        }
        const uint32_t rslot = (address >= 0xA000U) ? 3U
                             : (address >= 0x8000U) ? 2U
                             : (address >= 0x6000U) ? 1U : 0U;
        const uint32_t bias = g_state->bankOffsets[rslot];
        Cart_DriveByteFromPSRAM (address, bias);
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* WRITE: writes to 0x6000/0x6800/0x7000/0x7800 select a bank. slot =
     * (addr >> 11) & 3 -> 0..3. Same guard as v303: ignore writes
     * outside the cart window. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    if (address > 0xB000U) return;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        if ((GPIOE->INDR & CART_WR_MASK) == 0U) {
            const uint8_t w = Cart_ReadWriteData();
            const uint32_t slot = (uint32_t)((address >> 11) & 0x3U);
            const uint32_t base  = 0x4000U + (0x2000U * slot);
            g_state->bankOffsets[slot] = ((uint32_t)w << 13) - base;
            return;
        }
    }
}

/*
 * ASCII 16k mapper.
 *
 *   writes to 0x6000 select the 16 KiB bank at 0x4000..0x7FFF (page 1)
 *   writes to 0x7000 (or 0x77FF) select the 16 KiB bank at 0x8000..0xBFFF
 *   page 3 mirrors page 2 by the BIOS slot logic.
 */
static void Run16kASCII (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    const uint32_t ctrl    = GPIOE->INDR;

    if ((ctrl & CART_RD_MASK) == 0U) {
        const uint32_t bias = (address < 0x8000U)
                              ? g_state->bankOffsets[0]
                              : g_state->bankOffsets[8];
        Cart_DriveByteFromPSRAM (address, bias);
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        if ((GPIOE->INDR & CART_WR_MASK) == 0U) {
            const uint8_t w = Cart_ReadWriteData();
            if (address == 0x6000U) {
                g_state->bankOffsets[0] = ((uint32_t)w << 14) - 0x4000U;
            } else if (address == 0x7000U || address == 0x77FFU) {
                g_state->bankOffsets[8] = ((uint32_t)w << 14) - 0x8000U;
            }
            return;
        }
    }
}

/*
 * NEO 8 mapper. Three 8 KiB banks at 0x4000/0x6000/0x8000/0xA000.
 * Bank number is 12 bits, composed from sequential writes:
 *   write to addr+0   -> low byte of bank for bank @ (addr>>13)-1
 *   write to addr+1   -> high nibble of bank
 */
static void RunNEO8 (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    const uint32_t ctrl    = GPIOE->INDR;

    if ((ctrl & CART_RD_MASK) == 0U) {
        const uint32_t bank = (uint32_t)(address >> 13);
        if (bank <= 5U) {
            const uint32_t bias = g_state->bankOffsets[bank] << 13;
            Cart_DriveByteFromPSRAM (address & 0x1FFFU, bias);
        } else {
            /* Outside the cart's bank range. Drive 0xFF on the data
             * bus (open-bus pattern for unpopulated slots) so the Z80
             * reads a deterministic value. */
            GPIOB->OUTDR = 0xFFU << 8;
            GPIOB->CFGHR = CART_BUS_ON;
        }
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* WRITE: 12-bit bank number composed from sequential writes. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        if ((GPIOE->INDR & CART_WR_MASK) == 0U) {
            const uint8_t w = Cart_ReadWriteData();
            uint32_t bank = ((uint32_t)(address >> 11) & 0x7U) - 2U;
            if (bank > 5U) return;
            if (address & 1U) {
                g_state->bankOffsets[bank] =
                    (((uint32_t)w & 0x0FU) << 8) |
                    (g_state->bankOffsets[bank] & 0x00FFU);
            } else {
                g_state->bankOffsets[bank] =
                    (g_state->bankOffsets[bank] & 0xFF00U) | (uint32_t)w;
            }
            return;
        }
    }
}

/*
 * NEO 16 mapper. Three 16 KiB banks at 0x4000/0x8000/0xC000 (page 3
 * is BIOS-mirrored from page 2 by the slot logic).
 */
static void RunNEO16 (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    const uint32_t ctrl    = GPIOE->INDR;

    if ((ctrl & CART_RD_MASK) == 0U) {
        const uint32_t bank = (uint32_t)(address >> 14);
        if (bank <= 2U) {
            const uint32_t bias = g_state->bankOffsets[bank] << 14;
            Cart_DriveByteFromPSRAM (address & 0x3FFFU, bias);
        } else {
            /* Outside the cart's bank range. Drive 0xFF on the data
             * bus so the Z80 reads a deterministic value. */
            GPIOB->OUTDR = 0xFFU << 8;
            GPIOB->CFGHR = CART_BUS_ON;
        }
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* WRITE: 12-bit bank number composed from sequential writes. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        if ((GPIOE->INDR & CART_WR_MASK) == 0U) {
            const uint8_t w = Cart_ReadWriteData();
            uint32_t bank = ((uint32_t)(address >> 12) & 0x3U) - 1U;
            if (bank > 2U) return;
            if (address & 1U) {
                g_state->bankOffsets[bank] =
                    (((uint32_t)w & 0x0FU) << 8) |
                    (g_state->bankOffsets[bank] & 0x00FFU);
            } else {
                g_state->bankOffsets[bank] =
                    (g_state->bankOffsets[bank] & 0xFF00U) | (uint32_t)w;
            }
            return;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Cart_Banked_Dispatch - trampoline that reads g_mapper and jumps.    */
/* ------------------------------------------------------------------ */

static void Cart_Banked_Dispatch (void) {
    /* Late-entry check: SLTSL may have already risen before dispatch
     * (heavy PSRAM reads can stretch the latency past the next cycle's
     * rising edge). Bail without driving the bus. */
    if ((GPIOE->INDR & CART_SLTSL_MASK) != 0U) {
        GPIOB->CFGHR = CART_BUS_OFF;
        EXTI->INTFR  = EXTI_INTENR_MR0;
        return;
    }

    switch (g_mapper) {
    case CART_MAP_KONAMI:       RunKonami();       break;
    case CART_MAP_KONAMISCC:    RunKonamiSCC();    break;
    case CART_MAP_KONAMINOSCC:  RunKonamiNOSCC();  break;
    case CART_MAP_ASCII8k:      Run8kASCII();      break;
    case CART_MAP_ASCII16k:     Run16kASCII();     break;
    case CART_MAP_NEO8:         RunNEO8();         break;
    case CART_MAP_NEO16:        RunNEO16();        break;
    default:                 /* NONE or ROM mappers (shouldn't reach here) */
        GPIOB->CFGHR = CART_BUS_OFF;
        EXTI->INTFR  = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Hand-asm handlers for simple ROM mappers. Each runs from .ramfunc.  */
/* All three are structurally identical to Cart_EXTI0_PSRAM_Handler    */
/* from the original cart.c; only the page selection differs.         */
/* ------------------------------------------------------------------ */

/* Static asserts tying asm immediates to cart.h constants. The asm
 * uses these exact values; if cart.h drifts, the build breaks here
 * instead of silently corrupting data on the bus.
 *
 * The asm handlers only test single INDR bits, so RD/WR/SLTSL must stay
 * mask-1 bits (PE1=RD bit1, PE2=WR bit2, PE0=SLTSL bit0). */
_Static_assert (CART_BUS_ON       == 0x33333333U, "asm immediate drift");
_Static_assert (CART_BUS_OFF      == 0x44444444U, "asm immediate drift");
_Static_assert (PSRAM_CART_BASE   == 0x80000000UL, "asm bias drift");
_Static_assert (CART_SLTSL_MASK   == 0x0001U, "asm SLTSL drift");
_Static_assert (CART_RD_MASK     == 0x0002U, "asm RD drift");
_Static_assert (CART_WR_MASK     == 0x0004U, "asm WR drift");
_Static_assert (CART_MREQ_MASK    == 0x0020U, "asm MREQ drift");
_Static_assert (CART_IORQ_MASK    == 0x0100U, "asm IORQ drift (PE8 = bit 8)");

/* Game-base immediates for the asm handlers below. These must be
 * GAS-evaluable constant expressions (no C type suffixes) because
 * they are stringified into the asm templates; the assembler folds
 * them exactly like the compiler folds the C-side CART_GAME_BASE.
 * The two asserts pin the expressions to the cart.h constants so a
 * drift in either direction breaks the build instead of silently
 * serving bytes from the wrong PSRAM offset. */
#define ASM_STR_(s)  #s
#define ASM_STR(s)   ASM_STR_(s)
#define ASM_GAME_BASE_BYTES  ((CART_GAME_BASE_MB) * (1024) * (1024))
#define ASM_PSRAM_GAME_HI    (((0x80000000) + ASM_GAME_BASE_BYTES) >> 12)
#define ASM_ROM_BIAS_HI      (((0x80000000) + ASM_GAME_BASE_BYTES - (0x4000)) >> 12)

_Static_assert (ASM_PSRAM_GAME_HI
                == (uint32_t)((PSRAM_CART_BASE + CART_GAME_BASE) >> 12),
                "asm PSRAM game-base immediate drifted");
_Static_assert (ASM_ROM_BIAS_HI
                == (uint32_t)((PSRAM_CART_BASE + CART_GAME_BASE
                               - 0x4000UL) >> 12),
                "asm ROM bias immediate drifted");

/* Konami-no-SCC, hand-scheduled asm. Port of RunKonamiNOSCC. Direct
 * VTF entry - no Cart_Banked_Dispatch hop on any cycle.
 *
 *   t0 = 0x40011000  GPIOB (INDR -0x3F8, CFGHR -0x3FC, OUTDR -0x3F4)
 *                    + GPIOD (INDR +0x408)
 *   t1 = 0x40012000  GPIOE (INDR -0x7F8)
 *   t2 = 0x40010000  EXTI  (INTFR +0x414)
 *   a2 = %hi(s_state) [+ page*4] base for bankOffsets[] access
 *
 * Semantics (identical to the C body):
 *   if (SLTSL already high) { bus off; INTFR clear; return; }  // late
 *   if (RD low) {                       // read cycle
 *       page = addr >> 13
 *       if (2 <= page <= 5)
 *           drive PSRAM[addr + bankOffsets[page]]
 *       INTFR clear; wait SLTSL high; bus off
 *   } else {                            // write cycle
 *       INTFR clear;
 *       if (addr > 0xB000) return;
 *       while (SLTSL low) {
 *           if (WR low) {
 *               bankOffsets[addr >> 13] = (GPIOB->INDR>>8<<13) - (addr&0xE000);
 *               break;
 *           }
 *       }
 *   }
 *
 * The structure mirrors the working ROM16/32/48 handlers: lui of the
 * base registers at the top, late-entry bail, then the read/write
 * dispatch. Optimisations kept conservative to match the C semantics
 * exactly. The .option norelax wrap protects the %hi/%lo + index
 * arithmetic from gp-relaxation (without it the linker rewrites
 * "lw %lo(s_state)(a2)" into "lw off(gp)" and discards the page*4
 * index, making every read/write hit bankOffsets[0]).
 */
void Cart_EXTI0_KonamiNOSCC_Handler (void) {
    __asm__ volatile (
        ".option push                    \n"
        ".option norelax                 \n"
        "lui   t0, 0x40011                 \n" /* GPIOB/GPIOD window       */
        "lui   t1, 0x40012                 \n" /* GPIOE window             */
        "lui   t2, 0x40010                 \n" /* EXTI base (load up front) */
        "lw    a0, 1032(t0)                \n" /* a0 = GPIOD->INDR = A0..15*/
        "lw    a6, -2040(t1)               \n" /* a6 = GPIOE->INDR         */
        "andi  a6, a6, 3                   \n" /* SLTSL(bit0) | RD(bit1)   */
        "beqz  a6, 1f                      \n" /* both low -> read, fast   */
        "andi  a7, a6, 1                   \n"
        "bnez  a7, 2f                      \n" /* SLTSL high -> late entry */
        /* SLTSL low but RD high: a write, or ~RD has not fallen yet.
         * Poll until RD low (read) or WR low (write) - a single early
         * sample races the SLTSL->RD gate delay and misclassifies
         * early READ cycles as writes (same bug as ASCII8k). */
        "3:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a7, a6, 1                   \n"
        "bnez  a7, 2f                      \n" /* SLTSL rose -> cycle over */
        "andi  a7, a6, 2                   \n"
        "beqz  a7, 1f                      \n" /* RD low -> read           */
        "andi  a7, a6, 4                   \n" /* WR(bit2)                 */
        "bnez  a7, 3b                      \n" /* neither -> keep polling  */
        /* WRITE cycle: WR is low NOW, write data valid on GPIOB. */
        "li    a5, 1                        \n" /* a5 = MR0 bit (for INTFR) */
        "sw    a5, 1044(t2)                \n" /* clear INTFR              */
        "lui   a7, 0xB                     \n" /* a7 = 0xB000              */
        "bltu  a7, a0, 2f                  \n" /* addr > 0xB000 -> bail    */
        "lw    a3, -1016(t0)               \n" /* GPIOB->INDR              */
        "srli  a3, a3, 8                   \n" /* write data byte          */
        "lui   a2, %%hi(s_state)           \n"
        "slli  a3, a3, 13                  \n" /* w << 13                  */
        "srli  a1, a0, 13                  \n" /* page                     */
        "slli  a7, a1, 13                  \n" /* page_start = addr&0xE000 */
        "sub   a3, a3, a7                  \n" /* bias                     */
        "slli  a1, a1, 2                   \n" /* page * 4                 */
        "add   a2, a2, a1                  \n" /* &s_state + page*4        */
        "sw    a3, %%lo(s_state)(a2)       \n" /* bankOffsets[page] = bias */
        "j     2f                          \n"
        /* ---- READ CYCLE ---- */
        "1:                                \n"
        "srli  a1, a0, 13                  \n" /* a1 = page                */
        "addi  a2, a1, -2                  \n" /* page - 2                 */
        "sltiu a7, a2, 4                   \n" /* page 2..5 ?              */
        "bnez  a7, 5f                      \n" /* in-range: drive byte     */
        /* out-of-range: skip to INTFR clear + BusOff + spin */
        "j     6f                          \n"
        /* ---- IN-RANGE READ ---- */
        "5:                                \n"
        "lui   a2, %%hi(s_state)           \n"
        "slli  a1, a1, 2                   \n" /* page * 4                 */
        "add   a2, a2, a1                  \n" /* %hi(s_state) + page*4    */
        "lw    a3, %%lo(s_state)(a2)       \n" /* a3 = bankOffsets[page]   */
        "lui   a2, " ASM_STR(ASM_PSRAM_GAME_HI) "\n" /* PSRAM base + game base */
        "add   a2, a2, a3                  \n" /* base + bias              */
        "add   a2, a2, a0                  \n" /* + addr                   */
        "lbu   a3, 0(a2)                   \n" /* byte, bus tri-stated     */
        "slli  a3, a3, 8                   \n"
        "sw    a3, -1012(t0)               \n" /* GPIOB->OUTDR             */
        "lui   a4, 0x33333                 \n"
        "addi  a4, a4, 0x333               \n"
        "sw    a4, -1020(t0)               \n" /* CFGHR = BusOn            */
        "6:                                \n" /* shared read tail         */
        "li    a5, 1                        \n"
        "sw    a5, 1044(t2)                \n" /* clear INTFR              */
        "lui   a7, 0x44444                 \n" /* BusOff const             */
        "addi  a7, a7, 0x444               \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a6, a6, 1                   \n"
        "beqz  a6, 6b                      \n" /* wait SLTSL high          */
        "sw    a7, -1020(t0)               \n" /* CFGHR = BusOff           */
        "j     2f                          \n"
        /* ---- LATE ENTRY: SLTSL already high ---- */
        "2:                                \n"
        "lui   a7, 0x44444                 \n"
        "addi  a7, a7, 0x444               \n"
        "sw    a7, -1020(t0)               \n" /* CFGHR = BusOff           */
        "li    a5, 1                        \n"
        "sw    a5, 1044(t2)                \n" /* clear INTFR              */
        "9:                                \n"
        ".option pop                     \n"
        : : : "t0", "t1", "t2", "a0", "a1", "a2", "a3",
              "a4", "a5", "a6", "a7", "memory");
}

/* ASCII 8k, hand-scheduled asm. Port of Run8kASCII. Direct VTF entry -
 * no Cart_Banked_Dispatch hop on any cycle.
 *
 *   t0 = 0x40011000  GPIOB (INDR -0x3F8, CFGHR -0x3FC, OUTDR -0x3F4)
 *                    + GPIOD (INDR +0x408)
 *   t1 = 0x40012000  GPIOE (INDR -0x7F8)
 *   t2 = 0x40010000  EXTI  (INTFR +0x414)
 *   t5 = 0x33333333 (BusOn)   t6 = 0x44444444 (BusOff)
 *   a2 = %hi(s_state) [+ slot*4] base for bankOffsets[] access
 *
 * Semantics (identical to the C body):
 *   if (SLTSL already high) { bus off; INTFR clear; return; }  // late
 *   if (~MREQ high)        { bus off; INTFR clear; spin SLTSL; return; }
 *                                              // not a memory cycle:
 *                                              // port I/O, INTA, etc.
 *   if (RD low) {                       // read cycle
 *       if (~MREQ high on re-sample)   bail to late tail
 *       if (addr < 0x4000 || addr >= 0xC000) bus off + tail
 *       slot  = (addr >> 13) - 2          // 0..3, page-2 offset
 *       drive PSRAM[addr + bankOffsets[slot]]
 *       INTFR clear; wait SLTSL high; bus off
 *   } else {                            // write cycle
 *       INTFR clear;
 *       if (addr > 0xB000) return;
 *       while (SLTSL low) {
 *           if (~MREQ high) keep polling // non-memory cycle, ignore
 *           if (WR low) {
 *               slot = (addr >> 11) & 3
 *               bankOffsets[slot] = (w - 2 - slot) << 13
 *                 // == (w << 13) - (0x4000 + 0x2000*slot)
 *               break;
 *           }
 *       }
 *   }
 *
 * The read slot is exactly (page - 2), which the range check already
 * computes (page-2 in 0..3 <=> sltiu passes), so slot selection costs
 * nothing. The write bias collapses because 0x4000 = 2 << 13:
 *   (w << 13) - (0x4000 + 0x2000*slot) = ((w - 2 - slot) << 13)
 * (sub before the shift - no 12-bit-imm problem, no extra lui).
 * The .option norelax wrap protects the %hi/%lo + index arithmetic
 * from gp-relaxation, as for the KonamiNOSCC handler. */
void Cart_EXTI0_ASCII8k_Handler (void) {
    __asm__ volatile (
        ".option push                    \n"
        ".option norelax                 \n"
        "lui   t0, 0x40011                 \n" /* GPIOB/GPIOD window       */
        "lui   t1, 0x40012                 \n" /* GPIOE window             */
        "lui   t2, 0x40010                 \n" /* EXTI base                */
        "lw    a0, 1032(t0)                \n" /* a0 = GPIOD->INDR (early) */
        "lw    a6, -2040(t1)               \n" /* a6 = GPIOE->INDR         */
        "andi  a6, a6, 3                   \n" /* SLTSL(bit0) | RD(bit1)   */
        "beqz  a6, 1f                      \n" /* both low -> read, fast   */
        "andi  a7, a6, 1                   \n"
        "bnez  a7, 2f                      \n" /* SLTSL high -> late entry */
        /* Cycle qualifier (memory-cycle gate). The cart is a
         * memory-mapped device and must NOT respond on ~IORQ (port
         * I/O), INTA, or other non-memory cycles that may slip
         * through. ~MREQ is the canonical "this is a memory access"
         * signal - low for memory read, memory write, and opcode
         * fetch (M1). If MREQ is high at entry, the cycle is not for
         * us: keep the bus off, clear INTFR, and wait for SLTSL to
         * rise before returning. This avoids the bug where a port
         * write to 0x6000/0x6800/0x7000/0x7800 silently re-banks
         * the cart. */
        "andi  a7, a6, 0x20                \n" /* ~MREQ = PE5 = bit5      */
        "bnez  a7, 2f                      \n" /* MREQ high -> skip cycle */
        /* SLTSL low but RD high: a write, or ~RD has not fallen yet.
         * The C path samples ~RD after the dispatcher hop (always
         * settled); a single sample here races the SLTSL->RD gate
         * delay and misclassifies early READ cycles as writes (Z80
         * sees 0xFF all cycle - the observed failure). Poll until RD
         * low (read) or WR low (write). Re-check MREQ on every poll
         * iteration so the spin cannot miss a cycle that transitions
         * from MREQ-high to MREQ-low (rare, but cheap to gate). */
        "3:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a7, a6, 1                   \n"
        "bnez  a7, 2f                      \n" /* SLTSL rose -> cycle over */
        "andi  a7, a6, 0x20                \n" /* ~MREQ                    */
        "bnez  a7, 3b                      \n" /* MREQ high -> spin on    */
        "andi  a7, a6, 2                   \n"
        "beqz  a7, 1f                      \n" /* RD low -> read           */
        "andi  a7, a6, 4                   \n" /* WR(bit2)                 */
        "bnez  a7, 3b                      \n" /* neither -> keep polling  */
        /* WRITE cycle: WR is low NOW, write data valid on GPIOB. */
        "li    a5, 1                        \n"
        "sw    a5, 1044(t2)                \n" /* clear INTFR              */
        "lui   a7, 0xB                     \n" /* a7 = 0xB000              */
        "bltu  a7, a0, 2f                  \n" /* addr > 0xB000 -> bail    */
        "lw    a3, -1016(t0)               \n" /* GPIOB->INDR              */
        "srli  a3, a3, 8                   \n" /* write data byte w        */
        "lui   a2, %%hi(s_state)           \n"
        "srli  a1, a0, 11                  \n" /* addr >> 11               */
        "andi  a1, a1, 3                   \n" /* slot = (addr>>11)&3      */
        "addi  a7, a3, -2                  \n" /* w - 2                    */
        "sub   a7, a7, a1                  \n" /* w - 2 - slot             */
        "slli  a7, a7, 13                  \n" /* bias = (w-2-slot)<<13    */
        "slli  a1, a1, 2                   \n" /* slot * 4                 */
        "add   a2, a2, a1                  \n" /* %hi(s_state) + slot*4    */
        "sw    a7, %%lo(s_state)(a2)       \n" /* bankOffsets[slot] = bias */
        "j     2f                          \n"
        /* ---- READ CYCLE (re-check ~MREQ: bail if this is actually
         * a port/INTA cycle that happened to assert RD) ---- */
        "1:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a7, a6, 0x20                \n" /* ~MREQ                    */
        "bnez  a7, 2f                      \n" /* not a memory cycle ->    */
        "srli  a1, a0, 13                  \n" /* a1 = page (0..7)         */
        "addi  a2, a1, -2                  \n" /* a2 = slot = page - 2     */
        "sltiu a7, a2, 4                   \n" /* slot 0..3 ? (page 2..5)  */
        "beqz  a7, 6f                      \n" /* out of range -> tail     */
        "5:                                \n"
        "slli  a1, a2, 2                   \n" /* slot * 4 (a2 still slot) */
        "lui   a2, %%hi(s_state)           \n"
        "add   a2, a2, a1                  \n" /* %hi(s_state) + slot*4    */
        "lw    a3, %%lo(s_state)(a2)       \n" /* a3 = bankOffsets[slot]   */
        "lui   a2, " ASM_STR(ASM_PSRAM_GAME_HI) "\n" /* PSRAM base + game base */
        "add   a2, a2, a3                  \n" /* base + bias              */
        "add   a2, a2, a0                  \n" /* + addr                   */
        "lbu   a3, 0(a2)                   \n" /* byte, bus tri-stated     */
        "slli  a3, a3, 8                   \n"
        "sw    a3, -1012(t0)               \n" /* GPIOB->OUTDR             */
        "lui   a4, 0x33333                 \n"
        "addi  a4, a4, 0x333               \n"
        "sw    a4, -1020(t0)               \n" /* CFGHR = BusOn            */
        "6:                                \n" /* shared read tail         */
        "li    a5, 1                        \n"
        "sw    a5, 1044(t2)                \n" /* clear INTFR (once)       */
        "lui   a7, 0x44444                 \n" /* BusOff const             */
        "addi  a7, a7, 0x444               \n"
        "7:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a6, a6, 1                   \n"
        "beqz  a6, 7b                      \n" /* wait SLTSL high (tight)  */
        "sw    a7, -1020(t0)               \n" /* CFGHR = BusOff           */
        "j     2f                          \n"
        /* ---- LATE ENTRY: SLTSL already high ---- */
        "2:                                \n"
        "lui   a7, 0x44444                 \n"
        "addi  a7, a7, 0x444               \n"
        "sw    a7, -1020(t0)               \n" /* CFGHR = BusOff           */
        "li    a5, 1                        \n"
        "sw    a5, 1044(t2)                \n" /* clear INTFR              */
        "9:                                \n"
        ".option pop                     \n"
        : : : "t0", "t1", "t2", "a0", "a1", "a2", "a3",
              "a4", "a5", "a6", "a7", "memory");
}

/* ASCII 16k, hand-scheduled asm. Port of Run16kASCII. Direct VTF entry
 * - no Cart_Banked_Dispatch hop on any cycle.
 *
 *   t0 = 0x40011000  GPIOB (INDR -0x3F8, CFGHR -0x3FC, OUTDR -0x3F4)
 *                    + GPIOD (INDR +0x408)
 *   t1 = 0x40012000  GPIOE (INDR -0x7F8)
 *   t2 = 0x40010000  EXTI  (INTFR +0x414)
 *   t3 = 0x6000 (write decode: low bank register)
 *   t4 = 0x7000 (write decode: high bank register)
 *   t5 = 0x33333333 (BusOn)   t6 = 0x44444444 (BusOff)
 *   a5 = 1 (INTFR write-1-to-clear value)
 *   a2 = %hi(s_state) [+ byteOffset] base for bankOffsets[] access
 *
 * Semantics (identical to the C body):
 *   if (SLTSL already high) { bus off; INTFR clear; return; }  // late
 *   if (RD low) {                       // read cycle (NO range check -
 *       byteOffset = (addr<0x8000) ? 0 : 32     C serves every read)
 *       drive PSRAM[addr + bankOffsets[byteOffset>>2]]
 *       INTFR clear; wait SLTSL high; bus off
 *   } else {                            // write cycle
 *       INTFR clear;
 *       while (SLTSL low) {
 *           if (WR low) {
 *               if (addr == 0x6000)
 *                   bankOffsets[0] = (w - 1) << 14  // = (w<<14)-0x4000
 *               else if (addr == 0x7000 || addr == 0x77FF)
 *                   bankOffsets[8] = (w - 2) << 14  // = (w<<14)-0x8000
 *               return;  // C returns after ONE WR-low check
 *           }
 *       }
 *   }
 *
 * Read slot is a single bit test: addr bit 15 -> byte offset 0 or 32
 * (slli by 16 lands bit15 on bit31, then bltz). Write decode is exact
 * address (0x6000/0x7000/0x77FF), no range guard in C, so none here.
 * Write bias collapses because 0x4000 = 1 << 14 and 0x8000 = 2 << 14
 * (sub before the shift). The .option norelax wrap protects the
 * %hi/%lo + index arithmetic from gp-relaxation. */
void Cart_EXTI0_ASCII16k_Handler (void) {
    __asm__ volatile (
        ".option push                    \n"
        ".option norelax                 \n"
        "lui   t0, 0x40011                 \n" /* GPIOB/GPIOD window       */
        "lui   t1, 0x40012                 \n" /* GPIOE window             */
        "lui   t2, 0x40010                 \n" /* EXTI base                */
        "lui   t3, 0x6                     \n" /* 0x6000 (write decode)    */
        "lui   t4, 0x7                     \n" /* 0x7000 (write decode)    */
        "lui   t5, 0x33333                 \n" /* BusOn const              */
        "addi  t5, t5, 0x333               \n"
        "lui   t6, 0x44444                 \n" /* BusOff const             */
        "addi  t6, t6, 0x444               \n"
        "li    a5, 1                        \n" /* INTFR clear value        */
        "lw    a0, 1032(t0)                \n" /* a0 = GPIOD->INDR (early) */
        "lw    a6, -2040(t1)               \n" /* a6 = GPIOE->INDR         */
        "andi  a6, a6, 3                   \n" /* SLTSL(bit0) | RD(bit1)   */
        "beqz  a6, 1f                      \n" /* both low -> read, fast   */
        "andi  a7, a6, 1                   \n"
        "bnez  a7, 2f                      \n" /* SLTSL high -> late entry */
        /* SLTSL low but RD high: a write, or ~RD has not fallen yet.
         * Poll until RD low (read) or WR low (write) - never trust the
         * entry sample (SLTSL->RD gate delay misclassifies early
         * reads as writes; same bug as ASCII8k). */
        "3:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a7, a6, 1                   \n"
        "bnez  a7, 2f                      \n" /* SLTSL rose -> cycle over */
        "andi  a7, a6, 2                   \n"
        "beqz  a7, 1f                      \n" /* RD low -> read           */
        "andi  a7, a6, 4                   \n" /* WR(bit2)                 */
        "bnez  a7, 3b                      \n" /* neither -> keep polling  */
        /* WRITE cycle: WR is low NOW, write data valid on GPIOB. */
        "sw    a5, 1044(t2)                \n" /* clear INTFR              */
        "bne   a0, t3, 4f                  \n" /* addr != 0x6000           */
        "lw    a3, -1016(t0)               \n" /* GPIOB->INDR              */
        "srli  a3, a3, 8                   \n" /* w                        */
        "addi  a3, a3, -1                  \n" /* w - 1                    */
        "slli  a3, a3, 14                  \n" /* bias = (w<<14) - 0x4000  */
        "lui   a2, %%hi(s_state)           \n"
        "sw    a3, %%lo(s_state)(a2)       \n" /* bankOffsets[0] = bias    */
        "j     2f                          \n"
        "4:                                \n"
        "bne   a0, t4, 5f                  \n" /* addr != 0x7000           */
        "lw    a3, -1016(t0)               \n" /* GPIOB->INDR              */
        "srli  a3, a3, 8                   \n" /* w                        */
        "addi  a3, a3, -2                  \n" /* w - 2                    */
        "slli  a3, a3, 14                  \n" /* bias = (w<<14) - 0x8000  */
        "lui   a2, %%hi(s_state+32)        \n"
        "sw    a3, %%lo(s_state+32)(a2)    \n" /* bankOffsets[8] = bias    */
        "j     2f                          \n"
        "5:                                \n"
        "addi  a1, t4, 0x7FF               \n" /* a1 = 0x77FF              */
        "bne   a0, a1, 2f                  \n" /* addr != 0x77FF           */
        "lw    a3, -1016(t0)               \n" /* GPIOB->INDR              */
        "srli  a3, a3, 8                   \n" /* w                        */
        "addi  a3, a3, -2                  \n" /* w - 2                    */
        "slli  a3, a3, 14                  \n" /* bias = (w<<14) - 0x8000  */
        "lui   a2, %%hi(s_state+32)        \n"
        "sw    a3, %%lo(s_state+32)(a2)    \n" /* bankOffsets[8] = bias    */
        "j     2f                          \n"
        /* ---- READ CYCLE (no range check - C serves every read) ---- */
        "1:                                \n"
        "slli  a1, a0, 16                  \n" /* bit15 -> bit31           */
        "bltz  a1, 7f                      \n" /* addr >= 0x8000           */
        "li    a2, 0                       \n" /* bankOffsets[0]           */
        "j     8f                          \n"
        "7:                                \n"
        "li    a2, 32                      \n" /* bankOffsets[8]           */
        "8:                                \n"
        "lui   a4, %%hi(s_state)           \n"
        "add   a2, a4, a2                  \n" /* %hi + byte offset        */
        "lw    a3, %%lo(s_state)(a2)       \n" /* bankOffsets[slot]        */
        "lui   a2, " ASM_STR(ASM_PSRAM_GAME_HI) "\n" /* PSRAM base + game base */
        "add   a2, a2, a3                  \n" /* base + bias              */
        "add   a2, a2, a0                  \n" /* + addr                   */
        "lbu   a3, 0(a2)                   \n" /* byte, bus tri-stated     */
        "slli  a3, a3, 8                   \n"
        "sw    a3, -1012(t0)               \n" /* GPIOB->OUTDR             */
        "sw    t5, -1020(t0)               \n" /* CFGHR = BusOn            */
        "sw    a5, 1044(t2)                \n" /* clear INTFR (once)       */
        "9:                                \n" /* shared read tail: spin   */
        "lw    a6, -2040(t1)               \n"
        "andi  a6, a6, 1                   \n"
        "beqz  a6, 9b                      \n" /* wait SLTSL high (tight)  */
        "sw    t6, -1020(t0)               \n" /* CFGHR = BusOff           */
        /* fall through into late-entry tail (harmless re-stores) */
        /* ---- LATE ENTRY: SLTSL already high ---- */
        "2:                                \n"
        "sw    t6, -1020(t0)               \n" /* CFGHR = BusOff           */
        "sw    a5, 1044(t2)                \n" /* clear INTFR              */
        ".option pop                     \n"
        : : : "t0", "t1", "t2", "t3", "t4", "t5", "t6",
              "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "memory");
}

/* ROM16k: 16 KiB image mirrored at 0x4000 and 0x8000. Bias = img_base -
 * 0x4000, so addr + bias gives the right byte for both halves. */
void Cart_EXTI0_ROM16k_Handler (void) {
    __asm__ volatile (
        "lui   t0, 0x40011                 \n" /* GPIOB + GPIOD window      */
        "lui   t1, 0x40012                 \n" /* GPIOE window              */
        "lui   t2, 0x40010                 \n" /* EXTI window               */
        "lw    a6, -2040(t1)               \n" /* GPIOE->INDR               */
        "andi  a6, a6, 1                   \n" /* isolate ~SLTSL            */
        "beqz  a6, 1f                      \n"
        /* ---- late entry: SLTSL already high, cycle over ---- */
        "lui   a7, 0x44444                 \n"
        "addi  a7, a7, 0x444               \n"
        "sw    a7, -1020(t0)               \n"
        "sw    a6, 1044(t2)                \n"
        "j     2f                          \n"
        "1:                                \n"
        "lw    a0, 1032(t0)                \n" /* GPIOD->INDR = A0..A15     */
        "lui   a1, " ASM_STR(ASM_ROM_BIAS_HI) "\n" /* bias = PSRAM base + game base - 0x4000 */
        "add   a2, a0, a1                  \n" /* byte index                */
        "lbu   a3, 0(a2)                   \n" /* PSRAM byte, bus tri-state */
        "lui   a4, 0x33333                 \n"
        "addi  a4, a4, 0x333               \n"
        "slli  a3, a3, 8                   \n"
        "sw    a3, -1012(t0)               \n" /* GPIOB->OUTDR              */
        "sw    a4, -1020(t0)               \n" /* CFGHR=BusOn               */
        "lui   a7, 0x44444                 \n"
        "addi  a7, a7, 0x444               \n"
        "addi  a5, zero, 1                 \n"
        "sw    a5, 1044(t2)                \n"
        "3:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a6, a6, 1                   \n"
        "beqz  a6, 3b                      \n"
        "sw    a7, -1020(t0)               \n"
        "2:                                \n"
        : : : "t0", "t1", "t2", "a0", "a1", "a2", "a3", "a4",
              "a5", "a6", "a7", "memory");
}

/* Mailbox window: cart address 0x7FF0..0x7FFF. Defined here so the
 * FLASH handler at the bottom of this file can decode mailbox reads/
 * writes for the loader protocol. The mailbox is ONLY intercepted by
 * the FLASH handler - the ROM32k/ROM48k/ROM16k handlers serve the
 * cart image verbatim from PSRAM, so a real game ROM's bytes at
 * 0x7FF0 are not silently replaced by the loader's status byte (which
 * would crash the game the moment it touches that address).
 *
 * The FLASH handler at the bottom of this file also has its own copy
 * of the define so it compiles standalone. */
#define LOADER_MBOX_ADDR   0x7FF0U



/* ROM32k: 32 KiB cart image at 0x4000..0xBFFF, served from PSRAM
 * verbatim. NO mailbox interception - the loader mailbox window
 * 0x7FF0..0x7FFF lives ONLY inside the FLASH handler, not here.
 * Real game ROMs can have any byte at 0x7FF0 (it's just an ordinary
 * address inside their code/data window); the previous version of
 * this handler returned g_loader_mbox.status (= 0xC0 after a successful
 * LOAD_ROM) for every read at 0x7FF0 and 0xFF for everything else
 * in the mailbox window, which silently corrupted game code that
 * touched those addresses. The envelope test that needed the mailbox
 * under ROM32k is now obsolete - the FLASH handler serves both the
 * loader menu AND any future mailbox-based diagnostics. */
void Cart_EXTI0_ROM32k_Handler (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    uint32_t ctrl = GPIOE->INDR;

    if ((ctrl & CART_SLTSL_MASK) != 0U) {
        GPIOB->CFGHR = CART_BUS_OFF;
        EXTI->INTFR = EXTI_INTENR_MR0;
        return;
    }

    while ((ctrl & (CART_RD_MASK | CART_WR_MASK)) == (CART_RD_MASK | CART_WR_MASK)) {
        ctrl = GPIOE->INDR;
        if ((ctrl & CART_SLTSL_MASK) != 0U) {
            GPIOB->CFGHR = CART_BUS_OFF;
            EXTI->INTFR = EXTI_INTENR_MR0;
            return;
        }
    }

    if ((ctrl & CART_RD_MASK) == 0U && address >= 0x4000U && address < 0xC000U) {
        /* Serve from PSRAM with the bus still tri-stated - hides the
         * ~30-cycle PSRAM read latency before the data-bus drivers
         * come up. CART_GAME_BASE shifts the image within the window
         * (the same constant every other read path adds). */
        uint8_t v = *(const volatile uint8_t *)
            (PSRAM_CART_BASE + CART_GAME_BASE + address - 0x4000U);
        GPIOB->OUTDR = (GPIOB->OUTDR & ~(0xFFU << 8)) | ((uint32_t)v << 8);
        GPIOB->CFGHR = CART_BUS_ON;
    } else {
        /* Write cycle, or read outside the cart window: release the
         * bus so the MSX's own devices can respond. Writes to the
         * cart image window go nowhere (the Z80 is reading-only here,
         * but harmless to ignore). */
        GPIOB->CFGHR = CART_BUS_OFF;
    }

    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
    GPIOB->CFGHR = CART_BUS_OFF;
}

/* ROM48k: 48 KiB image at 0x4000..0xFFFF. Bias = img_base - 0x4000 (same
 * as ROM32k). The Z80 reads at 0xC000..0xFFFF overlap the BIOS slot
 * but the BIOS still owns page 3 from its own perspective; the cart
 * mirroring is transparent if the slot is configured for it. The MSX
 * BIOS leaves the cart slot unslotted and page 3 stays RAM. */
void Cart_EXTI0_ROM48k_Handler (void) {
    /* Same shape as ROM32k - the address space 0x4000..0xFFFF simply
     * spans a 48 KiB image instead of a 32 KiB one. The bias stays
     * img_base - 0x4000; image[48K] returns 0xFF (or whatever the
     * upper 16 KiB of the image holds) for addr >= 0xC000. */
    __asm__ volatile (
        "lui   t0, 0x40011                 \n"
        "lui   t1, 0x40012                 \n"
        "lui   t2, 0x40010                 \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a6, a6, 1                   \n"
        "beqz  a6, 1f                      \n"
        "lui   a7, 0x44444                 \n"
        "addi  a7, a7, 0x444               \n"
        "sw    a7, -1020(t0)               \n"
        "sw    a6, 1044(t2)                \n"
        "j     2f                          \n"
        "1:                                \n"
        "lw    a0, 1032(t0)                \n"
        "lui   a1, " ASM_STR(ASM_ROM_BIAS_HI) "\n" /* PSRAM base + game base - 0x4000 */
        "add   a2, a0, a1                  \n"
        "lbu   a3, 0(a2)                   \n"
        "lui   a4, 0x33333                 \n"
        "addi  a4, a4, 0x333               \n"
        "slli  a3, a3, 8                   \n"
        "sw    a3, -1012(t0)               \n"
        "sw    a4, -1020(t0)               \n"
        "lui   a7, 0x44444                 \n"
        "addi  a7, a7, 0x444               \n"
        "addi  a5, zero, 1                 \n"
        "sw    a5, 1044(t2)                \n"
        "3:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a6, a6, 1                   \n"
        "beqz  a6, 3b                      \n"
        "sw    a7, -1020(t0)               \n"
        "2:                                \n"
        : : : "t0", "t1", "t2", "a0", "a1", "a2", "a3", "a4",
              "a5", "a6", "a7", "memory");
}


/* ------------------------------------------------------------------ */
/* Flash mapper stub: legacy CART_MAP_FLASH slot.                     */
/* ------------------------------------------------------------------ */
/* selector_rom[] (the embedded 32 KiB cart image) was removed along
 * with the MSX-side romloader.c, so CART_MAP_FLASH has no image left
 * to serve. This stub:
 *
 *   0x4000..0x7FEF  : read returns 0xFF (open bus - matches an
 *                      empty / missing cart, doesn't fault the MSX if
 *                      the F-key or CMD_SOFTRESET fallback path
 *                      installs CART_MAP_FLASH by accident).
 *   0x7FF0           : read = g_loader_mbox.status (READY|DONE bits)
 *   0x7FF1           : read = pop one result byte from g_loader_mbox
 *   0x7FF0           : write = first byte is the command; further
 *                      bytes fill g_loader_mbox.args until arg_n
 *                      reaches the per-command count, then the
 *                      command is dispatched via have_cmd.
 *
 * This is enough to keep loader.c / terminal.c / Cart_SetMapper_Safe
 * calling Cart_SetMapper(CART_MAP_FLASH) without a linker error. Any
 * caller that actually wants to boot a cart must pick a real PSRAM-
 * backed mapper instead. */

static const uint8_t s_loader_args_of[8] = {
    0,  /* LOADER_CMD_DIR_OPEN    (0x00) */
    0,  /* LOADER_CMD_DIR_READ    (0x01) */
    0,  /* LOADER_CMD_DIR_CLOSE   (0x02) */
    12, /* LOADER_CMD_LOAD_ROM    (0x03): 12-byte SFN */
    1,  /* LOADER_CMD_SET_MAPPER  (0x04): 1 byte mapper id */
    0,  /* 0x05 unused (was RESET) */
    0,  /* LOADER_CMD_BOOT_NEXTOR (0x06) */
    0,  /* LOADER_CMD_SOFTRESET   (0x07) */
};

void Cart_EXTI0_Flash_Handler (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    uint32_t ctrl = GPIOE->INDR;

    /* Late entry: SLTSL already high, this cycle is already over. */
    if ((ctrl & CART_SLTSL_MASK) != 0U) {
        GPIOB->CFGHR = CART_BUS_OFF;
        EXTI->INTFR = EXTI_INTENR_MR0;
        return;
    }

    /* RD/WR lag SLTSL by a gate delay - poll until one settles, or
     * bail if SLTSL rises first. Same trick as the other handlers. */
    while ((ctrl & (CART_RD_MASK | CART_WR_MASK)) == (CART_RD_MASK | CART_WR_MASK)) {
        ctrl = GPIOE->INDR;
        if ((ctrl & CART_SLTSL_MASK) != 0U) {
            GPIOB->CFGHR = CART_BUS_OFF;
            EXTI->INTFR = EXTI_INTENR_MR0;
            return;
        }
    }

    if ((ctrl & CART_RD_MASK) == 0U) {
        /* READ cycle. */
        uint8_t v = 0xFFU;
        if (address == LOADER_MBOX_ADDR) {
            /* status */
            v = g_loader_mbox.status;
        } else if (address == (uint16_t)(LOADER_MBOX_ADDR + 1U)) {
            /* pop one result byte */
            if (g_loader_mbox.res_n > 0U) {
                v = g_loader_mbox.res[g_loader_mbox.res_head];
                __asm__ volatile ("fence r, r" ::: "memory");
                g_loader_mbox.res_head = (uint8_t)(
                    (g_loader_mbox.res_head + 1U) % LOADER_FIFO_DEPTH);
                g_loader_mbox.res_n--;
            } else {
                v = 0xFFU;
            }
        }
        /* Any other read in the cart window returns 0xFF (open bus). */
        GPIOB->OUTDR = (GPIOB->OUTDR & ~(0xFFU << 8))
                     | ((uint32_t)v << 8);
        GPIOB->CFGHR = CART_BUS_ON;
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* WRITE cycle: WR is low now; data valid on PB8..15. */
    if ((ctrl & CART_WR_MASK) == 0U) {
        const uint8_t w = (uint8_t)(GPIOB->INDR >> 8);
        if (address == LOADER_MBOX_ADDR) {
            /* Mailbox command byte stream. */
            if (g_loader_mbox.have_cmd == 0U && g_loader_mbox.arg_n == 0U) {
                /* No command in flight: this byte IS the cmd. */
                g_loader_mbox.cmd = w;
                uint8_t need = (w < 8U) ? s_loader_args_of[w] : 0U;
                if (need == 0U) {
                    g_loader_mbox.have_cmd = 1U;
                    g_loader_mbox.status &= (uint8_t)~LOADER_ST_DONE;
                } else {
                    g_loader_mbox.arg_n = need;
                }
            } else if (g_loader_mbox.arg_n > 0U) {
                /* Continuation byte of an in-flight command. */
                uint8_t slot = (uint8_t)(LOADER_FIFO_DEPTH
                                         - g_loader_mbox.arg_n);
                g_loader_mbox.args[slot] = w;
                g_loader_mbox.arg_n--;
                if (g_loader_mbox.arg_n == 0U) {
                    g_loader_mbox.have_cmd = 1U;
                    g_loader_mbox.status &= (uint8_t)~LOADER_ST_DONE;
                }
            }
        }
        /* Other writes: ignore (the loader never writes elsewhere). */
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* Neither RD nor WR asserted - spurious; release. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
    GPIOB->CFGHR = CART_BUS_OFF;
}

/* ------------------------------------------------------------------ */
/* Terminal mapper: serve the embedded terminal ROM (terminal_rom[])  */
/* for ordinary cart reads; decode the v303-style 3-byte mailbox      */
/* window 0x7FFD/0x7FFE/0x7FFF.                                       */
/* ------------------------------------------------------------------ */
/*
 * The MSX-side terminal program (MSXSoftware/RomLoader/asm/terminal.asm)
 * is a port of v303'smsxterminal.asm. Its 32 KiB image lives in flash
 * (terminal_rom[]). The cart-window contract is:
 *
 *   0x4000..0x7FFF  : the terminal ROM (32 KiB at 0x4000, accessed
 *                     directly - identical layout to the FLASH mapper)
 *   0x7FFD  WRITE   : MSX -> firmware keyboard FIFO (push 1 byte)
 *   0x7FFE  WRITE   : MSX -> firmware control byte
 *                        0x00 = "show the menu / stay in terminal"
 *                        0x04 = "user held GRPH - skip terminal, boot
 *                                the cart image straight away"
 *                        0x03 = "user chose to launch the cart image"
 *   0x7FFF  READ    : firmware -> MSX output FIFO (pop 1 byte)
 *                        0x00 = empty (no byte to print, MSX loops)
 *                        0x01..0xFF = printable / control byte to print
 *                        0x04 = "move the cursor" (followed by reading
 *                               X then Y on the FIFO)
 *                        0x03 = "boot the cart image"
 *
 * terminal.c owns the menu state machine and pumps bytes into
 * g_term_mbox.out_fifo; Cart_EXTI0_Terminal_Handler drains that FIFO
 * via reads at 0x7FFF and pushes keystrokes from 0x7FFD writes into
 * g_term_mbox.kbd_fifo. The control byte at 0x7FFE is latched into
 * g_term_mbox.control; terminal.c polls that flag and acts on it.
 *
 * Mailbox storage is in zero-wait-state SRAM. terminal.c and the cart
 * IRQ handler share it through the same kind of SPSC discipline used
 * by the FLASH loader mailbox.
 */

#define TERM_KBD_FIFO_DEPTH  16U
#define TERM_OUT_FIFO_DEPTH  2048U

/* TerminalMailbox struct + g_term_mbox definition live in terminal.h /
 * terminal.c. cart.c just touches the fields through the extern decl
 * below. */

extern TerminalMailbox g_term_mbox;

void Cart_EXTI0_Terminal_Handler (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    uint32_t ctrl = GPIOE->INDR;

    /* Late entry: SLTSL already high, this cycle is already over. */
    if ((ctrl & CART_SLTSL_MASK) != 0U) {
        GPIOB->CFGHR = CART_BUS_OFF;
        EXTI->INTFR = EXTI_INTENR_MR0;
        return;
    }

    /* RD/WR lag SLTSL by a gate delay - poll until one settles, or
     * bail if SLTSL rises first. Same fix as Cart_EXTI0_Flash_Handler. */
    while ((ctrl & (CART_RD_MASK | CART_WR_MASK)) == (CART_RD_MASK | CART_WR_MASK)) {
        ctrl = GPIOE->INDR;
        if ((ctrl & CART_SLTSL_MASK) != 0U) {
            GPIOB->CFGHR = CART_BUS_OFF;
            EXTI->INTFR = EXTI_INTENR_MR0;
            return;
        }
    }

    if ((ctrl & CART_RD_MASK) == 0U) {
        /* READ cycle. */
        uint8_t v;
        if (address == 0x7FFFU) {
            /* Output FIFO pop. Paired with Terminal_Service::out_push:
             * the producer does `fence w,w` between byte-write and
             * tail-increment, so reading tail and then the byte is safe
             * in the IRQ context. */
                        if (g_term_mbox.out_n > 0U) {
                v = g_term_mbox.out_buf[g_term_mbox.out_head];
                __asm__ volatile ("fence r, r" ::: "memory");
                g_term_mbox.out_head = (uint16_t)(
                    (g_term_mbox.out_head + 1U) % 2048U);
                g_term_mbox.out_n--;
            } else {
                v = 0x00U;  /* empty: MSX loops and waits */
            }
        } else if (address >= 0x4000U && address < 0xC000U) {
            /* Serve the embedded terminal ROM. The ROM image is sized
             * to fit just the actually-used bytes (terminal_rom_len),
             * not the full 32 KiB - bytes above terminal_rom_len
             * float to 0xFF (open-bus pattern, matches what an empty
             * cart would deliver if the user reads beyond the ROM).
             * The MSX-side terminal never fetches from 0x4100..0xBFFF,
             * so this saves ~31 KiB of flash per build. */
            const uint32_t off = (uint32_t)(address - 0x4000U);
            if (off < terminal_rom_len) {
                v = terminal_rom[off];
            } else {
                v = 0xFFU;
            }
        } else {
            /* Out of the terminal's window: float. */
            EXTI->INTFR = EXTI_INTENR_MR0;
            while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
            GPIOB->CFGHR = CART_BUS_OFF;
            return;
        }
        /* Drive the byte. */
        GPIOB->OUTDR = (GPIOB->OUTDR & ~(0xFFU << 8))
                     | ((uint32_t)v << 8);
        GPIOB->CFGHR = CART_BUS_ON;
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* WRITE cycle: WR is low now; data valid on PB8..15. */
    if ((ctrl & CART_WR_MASK) == 0U) {
        const uint8_t w = (uint8_t)(GPIOB->INDR >> 8);
        if (address == 0x7FFDU) {
            /* Keyboard FIFO push. Drop on overflow - the terminal
             * never sends faster than human typing, so overflow is a
             * firmware-side bug worth noticing. */
            uint8_t next = (uint8_t)((g_term_mbox.kbd_tail + 1U) % 16U);
            if (next != g_term_mbox.kbd_head) {
                g_term_mbox.kbd_buf[g_term_mbox.kbd_tail] = w;
                g_term_mbox.kbd_tail = next;
                g_term_mbox.kbd_n++;
            }
        } else if (address == 0x7FFEU) {
            /* Control byte. Latched verbatim - terminal.c reads and
             * clears it (so the cart-side IRQ sees each control byte
             * exactly once). */
            g_term_mbox.control = w;
        }
        /* Other writes: ignore. */
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* Neither RD nor WR asserted - spurious; release. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
    GPIOB->CFGHR = CART_BUS_OFF;
}
/* ------------------------------------------------------------------ */
/* Sunrise IDE mapper handler.                                         */
/*                                                                     */
/* Cart-window contract (see sunrise_ide.h for the full map):          */
/*                                                                     */
/*   0x4000..0x7FFF     : Nextor ROM (nextor_rom[]) - the lower 14      */
/*                        address bits index into the 16 KiB bank       */
/*                        selected by the last 0x4104 write.            */
/*   0x4104 (W)         : control register (bank bit-reversed +        */
/*                        IDE enable).                                 */
/*   0x7C00..0x7DFF     : ATA data register (16-bit PIO).              */
/*   0x7E00..0x7EFF     : ATA task-file register file.                 */
/*                                                                     */
/* The handler defers the register file + sector buffer + LBA math to  */
/* sunrise_ide.c (Sunrise_IDE_ReadByte / Sunrise_IDE_WriteByte) - both */
/* are IRQ-safe (no blocking, no printf, no long loops). The handler  */
/* just routes the bus cycle and drives the data byte on a read.      */
/*                                                                     */
/* Kept in flash (not .ramfunc): the IDE register / data accesses are  */
/* infrequent (the kernel's PIO loop runs 256-512 bytes per LBA, but  */
/* each iteration is dozens of Z80 cycles) and the cart flash path is  */
/* zero-wait, so there is no benefit to paying the PSRAM cost.         */
/* ------------------------------------------------------------------ */

void Cart_EXTI0_Sunride_Handler (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    uint32_t ctrl = GPIOE->INDR;

    /* Late entry: SLTSL already high, this cycle is over. */
    if ((ctrl & CART_SLTSL_MASK) != 0U) {
        GPIOB->CFGHR = CART_BUS_OFF;
        EXTI->INTFR = EXTI_INTENR_MR0;
        return;
    }

    /* RD/WR lag SLTSL by a gate delay - poll until one settles, or
     * bail if SLTSL rises first. Same pattern as the terminal handler. */
    while ((ctrl & (CART_RD_MASK | CART_WR_MASK)) ==
           (CART_RD_MASK | CART_WR_MASK)) {
        ctrl = GPIOE->INDR;
        if ((ctrl & CART_SLTSL_MASK) != 0U) {
            GPIOB->CFGHR = CART_BUS_OFF;
            EXTI->INTFR = EXTI_INTENR_MR0;
            return;
        }
    }

    if ((ctrl & CART_RD_MASK) == 0U) {
        /* READ cycle. */
        const uint8_t v = Sunrise_IDE_ReadByte (address);
        GPIOB->OUTDR = (GPIOB->OUTDR & ~(0xFFU << 8))
                     | ((uint32_t)v << 8);
        GPIOB->CFGHR = CART_BUS_ON;
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* WRITE cycle. */
    if ((ctrl & CART_WR_MASK) == 0U) {
        const uint8_t w = (uint8_t)(GPIOB->INDR >> 8);
        Sunrise_IDE_WriteByte (address, w);
        EXTI->INTFR = EXTI_INTENR_MR0;
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
        return;
    }

    /* Neither RD nor WR - spurious; release. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
    GPIOB->CFGHR = CART_BUS_OFF;
}

/* ------------------------------------------------------------------ */
/* None handler: bus floats. Z80 reads 0xFF (open-bus pattern).       */
/* ------------------------------------------------------------------ */

void Cart_EXTI0_None_Handler (void) {
    __asm__ volatile (
        "lui   t0, 0x40011                 \n"
        "lui   t1, 0x40012                 \n"
        "lui   t2, 0x40010                 \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a6, a6, 1                   \n"
        "beqz  a6, 1f                      \n"
        "lui   a7, 0x44444                 \n"
        "addi  a7, a7, 0x444               \n"
        "sw    a7, -1020(t0)               \n"
        "sw    a6, 1044(t2)                \n"
        "j     2f                          \n"
        "1:                                \n"
        /* Just clear pending and spin. Bus remains tri-stated. */
        "addi  a5, zero, 1                 \n"
        "sw    a5, 1044(t2)                \n"
        "3:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a6, a6, 1                   \n"
        "beqz  a6, 3b                      \n"
        "2:                                \n"
        : : : "t0", "t1", "t2", "a5", "a6", "a7", "memory");
}

/* ------------------------------------------------------------------ */
/* CartServiceLoop - legacy polling driver. Kept for API compatibility */
/* but unused by the EXTI-driven path.                                 */
/* ------------------------------------------------------------------ */

void CartServiceLoop (void) {
    /* No-op stub. The actual cart serving happens in Cart_EXTI0_*. */
    __asm__ volatile ("wfi");
}

#pragma GCC pop_options