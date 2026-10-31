/* cart.c - minimal cartridge core for firmware2.
 *
 * Ported from /firmware User/cart.c, reduced to the three elements this
 * firmware exists for:
 *
 *   1. CART_MAP_NEXTOR  - the Nextor kernel mapper: ASCII16K banking over
 *      the flash-resident nextor_rom[] with the 0x7FF0..0x7FF5 mailbox
 *      routed to nextor.c (Cart_EXTI0_Nextor_Handler, one IRQ per Z80
 *      bus cycle via EXTI0 on ~SLTSL).
 *   2. the expanded-slot layer - secondary-slot register at 0xFFFF
 *      (M1-qualified, read inverted), sub-slot 0 = kernel ROM,
 *      sub-slot 1 = mapper RAM, sub-slots 2/3 electrically empty
 *      (Cart_SubslotRead / Cart_SubslotWrite).
 *   3. the MSX memory mapper - sub-slot 1 RAM, 16 KiB banks selected by
 *      OUT (0xFC..0xFF),A, latched by the PE8/~IORQ EXTI9_5 decoder,
 *      backed by PSRAM (EXTI9_5_IRQHandler + Cart_*MapperRamByte).
 *
 * Not ported, on purpose: the game mappers (ROM16/32/48, Konami, NEO,
 * ASCII8/16), SCC, terminal, loader, ~RESET control, mapper swapping
 * (Cart_SetMapper_Safe), the legacy polling service loop, and every
 * hot-path diagnostics counter. The counters in particular were volatile
 * read-modify-writes on the cycle-critical read path; leaving them out
 * is worth several CPU cycles per served Z80 cycle.
 *
 * The mapper is installed once during startup (main() calls
 * Cart_SetMapper(CART_MAP_NEXTOR), which installs the handler and arms
 * the cart bus interrupt) and never swapped. */

#include "cart.h"
#include "psram.h"
#include "nextor.h"
#include "ch32v4x7.h"
#include "debug.h"

#pragma GCC push_options
#pragma GCC optimize("Ofast")


struct MSXState {
    uint32_t bankOffsets[16]; /* bank bias = (page * bankSize) - (Z80 - 0x4000) */
} s_state;


volatile Cart_Mapper g_mapper = CART_MAP_NONE;

static struct MSXState *const g_state = &s_state;

/* Diagnostic ~WAIT hold flag. Normal startup leaves it clear; only
 * Cart_HoldMSXWait_Begin/End modify it (used by the USB layer around
 * PSRAM DMA transfers). While set, every bus-cycle handler skips the
 * post-cycle SLTSL spin (the cycle never finishes until End() releases
 * ~WAIT). */
static volatile uint8_t s_msx_wait_held;

void Cart_EXTI0_Nextor_Handler (void) __attribute__((noinline,
                                                      interrupt("WCH-Interrupt-fast")));
void Cart_EXTI0_None_Handler (void) __attribute__((noinline,
                                                   interrupt("WCH-Interrupt-fast")));
void EXTI9_5_IRQHandler (void) __attribute__((section(".ramfunc"), noinline,
                                              interrupt("WCH-Interrupt-fast")));

/* Embedded Nextor kernel ROM (flash-resident, generated from
 * MSXSoftware/NextorDriver/Nextor-3.0.RISKYMSX2.ROM by the Makefile's
 * regenerate_nextor_rom target -> tools/rom2c.py). That image is the
 * Nextor 3.0 kernel plus the RISKYMSX2 disk driver, so it is 128 KiB =
 * 8 banks x 16 KiB. The mapper is ASCII16K: bank n of the Z80 page-1
 * window is selected by writing n to 0x6000, and a byte at cart address
 * A is nextor_rom[(n << 14) + (A & 0x3FFF)]. The driver reaches the
 * disk only through the mailbox window at 0x7FF0..0x7FF5. */
extern const uint8_t  nextor_rom[];
extern const uint32_t nextor_rom_len;

/* Expanded-slot state. The sub-slot register follows the MSX convention:
 * two bits select the sub-slot for each 16 KiB CPU page. Sub-slot 0 is
 * flash-backed/read-only, sub-slot 1 is mapper RAM, and sub-slots 2/3 are
 * electrically empty. */
static volatile uint8_t g_subslot_reg = 0x00U;
static volatile uint8_t s_mapper_ram_ready;
/* Mapper-RAM bank registers, one per CPU page (FCh..FFh).
 *
 * FCh/FDh/FEh/FFh select the 16 KiB bank windowed into pages 0/1/2/3.
 * Only sub-slot 1 exposes the mapper RAM. Sub-slots 2 and 3 are
 * electrically empty, so a slot checker sees RAM only at 1-1 and the four
 * sub-slots are distinguishable (ROM / RAM / empty / empty). */
static volatile uint8_t s_slotted_bank_regs[4] = { 0U, 0U, 0U, 0U };

/* ------------------------------------------------------------------ */
/* Bus diagnostics (CART_TIMING_STATS / CART_LA_MARKER)                */
/* ------------------------------------------------------------------ */
/* The sub-slot-1 (PSRAM mapper RAM) read is the most timing-critical
 * serve on this cart: a byte driven onto GPIOB after the Z80's data
 * sampling point reads back wrong and - when the kernel executes or
 * tests RAM in the mapper window - crashes the machine.
 *
 *   CART_LA_MARKER     PA0 (LEDFLASH, next to the cart connector) is
 *                      driven HIGH while a PSRAM access is in flight
 *                      and LOW once the byte is on the bus, giving a
 *                      logic analyser three directly measurable
 *                      intervals: ~SLTSL fall -> marker rise (interrupt
 *                      entry + dispatch), marker high time (PSRAM HB
 *                      read), and marker fall vs the Z80's data
 *                      sampling point (~RD rise). Cost: two single-
 *                      constant stores (~2 cycles) - safe to run
 *                      permanently.
 *
 *   CART_TIMING_STATS  plain serve COUNTERS (reads/writes) printed by
 *                      main()'s debug heartbeat. Cost: one increment
 *                      per PSRAM serve (~6 cycles), the level /firmware
 *                      ran with successfully.
 *
 * IMPORTANT: an earlier version of CART_TIMING_STATS timed every serve
 * (mcycle timestamp at handler entry on EVERY bus cycle, min/max/late
 * updates per serve) - that added ~275 ns to each PSRAM serve and the
 * machine did not boot at all, the exact regression /firmware's source
 * warns about ("NEVER increment from the EXTI0 hot path"). Do not put
 * timestamps or multi-counter updates back into the handler; measure
 * latency externally with the LA marker. Build the diagnostics:
 *   make rebuild QUIET=0 DEBUG=1 CART_TIMING_STATS=1 CART_LA_MARKER=1
 */
#if CART_TIMING_STATS
/* Serve counters for the sub-slot-1 (PSRAM mapper RAM) path.
 * COUNTS ONLY - deliberately no timestamps, min/max or late detection:
 * /firmware's own comments document that volatile RMWs on the hot read
 * path broke the BIOS slot probe ("NEVER increment from the EXTI0 hot
 * path"), and the first instrumented firmware2 build reproduced that
 * regression exactly (added ~275 ns per PSRAM serve -> no boot).
 * One increment per serve (~6 cycles) is the level /firmware's release
 * build ran with successfully. Timing/latency questions are answered
 * by the logic analyser via CART_LA_MARKER, which costs ~2 cycles. */
volatile uint32_t g_cart_slotted_reads  = 0U;
volatile uint32_t g_cart_slotted_writes = 0U;
#endif

#if CART_LA_MARKER
/* PA0 (LEDFLASH) marker for the logic analyser: driven HIGH while the
 * PSRAM access is in flight, LOW once the byte is on the bus. Written
 * as single constant stores (PA0 is the only push-pull output on
 * GPIOA, and AF pins ignore ODR), so each mark costs one store -
 * about 2 cycles per access, negligible even on the marginal PSRAM
 * serve path. */
#define CART_LA_MARK(level) (GPIOA->OUTDR = (uint32_t)(level))
#else
#define CART_LA_MARK(level) ((void)0)
#endif

void Cart_SetMapperRamReady (uint8_t ready) {
    s_mapper_ram_ready = (ready != 0U) ? 1U : 0U;
}

/* Mapper RAM lives in PSRAM at CART_SLOTTED_BANK_BASE_OFFSET (4 MiB into
 * the 8 MiB window), one 16 KiB bank per bank-register value, aliased by
 * CART_SLOTTED_PSRAM_BANK_MASK exactly like a 1 MiB physical MSX memory
 * mapper. */
static inline __attribute__((always_inline))
void Cart_DriveMapperRamByte (uint16_t address, unsigned page) {
    CART_LA_MARK (1U);
    const uint32_t off = CART_SLOTTED_BANK_BASE_OFFSET
                       + (((uint32_t)s_slotted_bank_regs[page]
                           & CART_SLOTTED_PSRAM_BANK_MASK)
                          * CART_SLOTTED_BANK_SIZE)
                       + ((uint32_t)address & 0x3FFFU);
    const uint8_t value = *(const volatile uint8_t *)(PSRAM_CART_BASE + off);
    GPIOB->OUTDR = (uint32_t)value << 8;
    GPIOB->CFGHR = CART_BUS_ON;
    CART_LA_MARK (0U);
#if CART_TIMING_STATS
    g_cart_slotted_reads++;
#endif
}

static inline __attribute__((always_inline))
void Cart_WriteMapperRamByte (uint16_t address, unsigned page,
                              uint8_t value) {
    CART_LA_MARK (1U);
    const uint32_t off = CART_SLOTTED_BANK_BASE_OFFSET
                       + (((uint32_t)s_slotted_bank_regs[page]
                           & CART_SLOTTED_PSRAM_BANK_MASK)
                          * CART_SLOTTED_BANK_SIZE)
                       + ((uint32_t)address & 0x3FFFU);
    *(volatile uint8_t *)(PSRAM_CART_BASE + off) = value;
    __asm__ volatile ("fence iorw, iorw" ::: "memory");
    CART_LA_MARK (0U);
#if CART_TIMING_STATS
    g_cart_slotted_writes++;
#endif
}

/* Expanded-slot layer used by the Nextor handler. Both return nonzero
 * when the cycle was fully handled (the caller must then skip its own
 * ROM/mailbox decode); zero means sub-slot 0 is selected for the page
 * and the caller serves its own ROM. No diagnostics counters here: this
 * is the hot path, one IRQ per Z80 bus cycle. */
static inline __attribute__((always_inline))
int Cart_SubslotRead (uint16_t address, uint32_t ctrl) {
    const unsigned page = (unsigned)(address >> 14);

    /* SSR is read inverted. M1 low is an opcode fetch from FFFF and must
     * not be mistaken for a register read. */
    if (address == 0xFFFFU && (ctrl & CART_M1_MASK) != 0U) {
        GPIOB->OUTDR = (uint32_t)(uint8_t)~g_subslot_reg << 8;
        GPIOB->CFGHR = CART_BUS_ON;
        return 1;
    }
    const unsigned subslot = (g_subslot_reg >> (page * 2U)) & 0x3U;
#if !CART_SLOTTED_PAGE3_RAM
    /* Optional compatibility mode: only answer the SSR in page 3. */
    if (page == 3U) {
        GPIOB->CFGHR = CART_BUS_OFF;
        return 1;
    }
#endif
    if (subslot == 0U) return 0;
    /* Only sub-slot 1 exposes mapper RAM; sub-slots 2 and 3 are electrically
     * empty so slot probing can distinguish them. */
    if (subslot == 1U && s_mapper_ram_ready != 0U) {
        Cart_DriveMapperRamByte (address, page);
    } else {
        GPIOB->CFGHR = CART_BUS_OFF;
    }
    return 1;
}

static inline __attribute__((always_inline))
int Cart_SubslotWrite (uint16_t address, uint32_t ctrl, uint8_t value) {
    const unsigned page = (unsigned)(address >> 14);

    if (address == 0xFFFFU && (ctrl & CART_M1_MASK) != 0U) {
        g_subslot_reg = value;
        return 1;
    }
    const unsigned subslot = (g_subslot_reg >> (page * 2U)) & 0x3U;
#if !CART_SLOTTED_PAGE3_RAM
    /* Optional compatibility mode: leave page 3 host-owned. */
    if (page == 3U) {
        return 1;
    }
#endif
    if (subslot == 0U) return 0;
    /* Only sub-slot 1 exposes mapper RAM; sub-slots 2 and 3 are empty. */
    if (subslot == 1U && s_mapper_ram_ready != 0U) {
        Cart_WriteMapperRamByte (address, page, value);
    }
    return 1;
}

/* Drive one byte from a FLASH-resident const array onto GPIOB[15:8] and
 * turn on the drivers. Read the byte while the bus is STILL TRI-STATED,
 * then enable the drivers, so a stale OUTDR is never driven during the
 * load latency. Used by the Nextor mapper, whose ROM is nextor_rom[]. */
static inline __attribute__((always_inline))
void Cart_DriveByteFromFlash (const uint8_t *rom, uint32_t off) {
    uint8_t b = (off < nextor_rom_len) ? rom[off] : 0xFFU;
    GPIOB->OUTDR = (uint32_t)b << 8;
    GPIOB->CFGHR = CART_BUS_ON;
}

/* Read the data-bus byte (write-data from Z80) on the way into a write
 * cycle. Bits land in PB[15:8] which is the upper byte of GPIOB->INDR. */
static inline __attribute__((always_inline))
uint8_t Cart_ReadWriteData (void) {
    return (uint8_t)((GPIOB->INDR >> 8) & 0xFFU);
}

/* MSX memory-mapper bank-register capture. OUT (0xFC..0xFF),A is an
 * I/O cycle and therefore never asserts ~SLTSL; PE8/~IORQ is its EXTI
 * trigger. Each port selects one 16 KiB bank for a CPU page. The handler
 * only samples the write and never drives the data bus. Enabled only
 * while the Nextor mapper is installed. */
void EXTI9_5_IRQHandler (void) {
    if ((EXTI->INTFR & EXTI_INTENR_MR8) == 0U) return;
    EXTI->INTFR = EXTI_INTENR_MR8;

    uint32_t ctrl = GPIOE->INDR;
    if ((ctrl & CART_IORQ_MASK) != 0U) return;

    /* /IORQ can lead /WR by a short gate delay. Wait briefly for the
     * write strobe, abandoning the edge if /IORQ is released first. */
    uint32_t spin = 4096U;
    while ((ctrl & CART_WR_MASK) != 0U
           && (ctrl & CART_IORQ_MASK) == 0U
           && spin-- != 0U) {
        ctrl = GPIOE->INDR;
    }
    if ((ctrl & CART_WR_MASK) != 0U) return;

    /* OUT (n),A puts the port number n on A7..A0 (low byte) and the
     * accumulator A on A15..A8 (high byte). The bank-select PORT is the
     * low byte; the high byte is the bank VALUE, which must NOT be used
     * to index the page register (doing so misroutes the latch and
     * corrupts the bank registers - the boot hang). */
    const uint8_t port = (uint8_t)(GPIOD->INDR & 0x00FFU);
    if (port < CART_SLOTTED_BANK_PORT_BASE
        || port >= CART_SLOTTED_BANK_PORT_BASE + 4U) return;

    s_slotted_bank_regs[port - CART_SLOTTED_BANK_PORT_BASE] =
        Cart_ReadWriteData ();
    __asm__ volatile ("fence iorw, iorw" ::: "memory");
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

Cart_Mapper Cart_GetMapper (void) { return g_mapper; }

/* ------------------------------------------------------------------ */
/* Cart_SetMapper                                                      */
/* ------------------------------------------------------------------ */

int Cart_SetMapper (Cart_Mapper m) {
    if (m != CART_MAP_NEXTOR) return -1;

    /* Reset bank state so an install does not leak stale offsets. */
    for (unsigned i = 0; i < 16; i++) {
        s_state.bankOffsets[i] = 0U;
    }

    /* Never carry page or sub-slot selections into a fresh install. */
    g_subslot_reg = 0x00U;
    for (unsigned page = 0U; page < 4U; page++) {
        s_slotted_bank_regs[page] = 0U;
    }

    g_mapper = m;

    /* Install the Nextor handler in VTF slot 0. SetVTFIRQ writes
     * VTFADDR[0] and VTFIDR[0] - the VTF dispatcher uses VTFADDR[0]
     * when EXTI0 fires (the IRQ number is matched by VTFIDR[0]). */
    SetVTFIRQ ((uint32_t)Cart_EXTI0_Nextor_Handler, EXTI0_IRQn, 0, ENABLE);

    /* Nextor owns the PE8/~IORQ bank-register decoder (ports FC..FF). */
    EXTI->INTFR = EXTI_INTENR_MR8;
    EXTI->INTENR |= EXTI_INTENR_MR8;
    NVIC_SetPriority (EXTI9_5_IRQn, 0x01);
    NVIC_EnableIRQ (EXTI9_5_IRQn);

    /* Reset the mailbox engine: bank select, command collector, result
     * FIFO - so the kernel starts with a clean engine (no stale LBA /
     * sector buffer from a previous run, no stale state from a
     * half-completed READ/WRITE). Safe to call repeatedly. */
    Nextor_Init ();

    /* Nextor is ASCII16K over nextor_rom[]: 16 KiB banks selected by
     * 0x6000 (page 1) and 0x7000/0x77FF (page 2). Both pages start
     * on bank 0, so the kernel sees ROM[0x0000..0x3FFF] at 0x4000 the
     * moment the mapper is installed - no bank-select write needed
     * before the reset vector is fetched. The bias is stored in the
     * usual "add to the raw cart address" form:
     *   nextor_rom + bankOffsets[0] + 0x4000 == nextor_rom + 0
     *   nextor_rom + bankOffsets[8] + 0x8000 == nextor_rom + 0
     * The mailbox window at 0x7FF0..0x7FF5 has no bias of its own -
     * the handler intercepts it by absolute address before the
     * index is ever computed.
     *
     * Note: the EXTI0 bus interrupt itself is already armed by
     * Init_Cart() with the placeholder handler - this function
     * re-points the VTF entry to the Nextor handler and completes the
     * mapper state, exactly like /firmware's SetMapper swap against an
     * already-live bus. */
    s_state.bankOffsets[0] = 0x0000U - 0x4000U;
    s_state.bankOffsets[8] = 0x0000U - 0x8000U;

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

    /* Control bus PE0(SLTSL) PE1(RD) PE2(WR) PE5(MREQ) PE8(IORQ) as
     * floating inputs. PE3 = WAIT: preserve an explicit diagnostic hold,
     * but normal startup leaves PE3 floating. PE4 = MSX ~RESET stays a
     * floating input - the firmware never drives it. */
    {
        uint32_t control_cfg = 0x44444444U;
        if (s_msx_wait_held != 0U) {
            /* The diagnostic helper cleared the latch before changing the
             * mode; keep the output low while that optional hold is active. */
            control_cfg = (control_cfg & ~(0xFU << 12)) | (0x3U << 12);
        }
        GPIOE->CFGLR = control_cfg;
    }

    /* Data bus PB8..PB15 tri-stated (bus off) */
    GPIOB->CFGHR = CART_BUS_OFF;

    /* LEDFLASH PA0 as push-pull output, 50 MHz slew */
    GPIOA->CFGLR &= ~(0xFu << 0);
    GPIOA->CFGLR |=  (0x3u << 0);

    /* Route PE0 -> EXTI0, FALLING edge only. One IRQ per Z80 bus
     * cycle: the handler serves the byte AND releases the bus inside
     * the same invocation (spins until SLTSL rises), so no rising-
     * edge IRQ is needed.
     *
     * REVERTED to the first working firmware2 port's boot order (and
     * /firmware's): the line is armed HERE with the no-mapper
     * placeholder handler; main() installs Nextor via
     * Cart_SetMapper() a few microseconds later. The later ~WAIT
     * boot-sync variant proved unreliable on the target machine. */
    AFIO->EXTICR[0] = (AFIO->EXTICR[0] & ~(0xFU << 0)) |
                      (AFIO_EXTICR1_EXTI0_PE << 0);
    EXTI->INTENR = (EXTI->INTENR & ~EXTI_INTENR_MR0) | EXTI_INTENR_MR0;
    EXTI->RTENR &= ~EXTI_RTENR_TR0;
    EXTI->FTENR = (EXTI->FTENR & ~EXTI_FTENR_TR0) | EXTI_FTENR_TR0;
    EXTI->INTFR = EXTI_INTENR_MR0;

    /* PE8 (/IORQ) feeds the bank-register decoder for ports FC..FF used
     * by the expanded-slot mapper. Configure the EXTI route now, but keep
     * the line disabled until Cart_SetMapper installs Nextor. */
    GPIOE->CFGHR = (GPIOE->CFGHR & ~0xFU) | 0x4U;
    AFIO->EXTICR[2] = (AFIO->EXTICR[2] & ~0xFU)
                    | (AFIO_EXTICR3_EXTI8_PE << 0);
    EXTI->RTENR &= ~EXTI_RTENR_TR8;
    EXTI->FTENR = (EXTI->FTENR & ~EXTI_FTENR_TR8) | EXTI_FTENR_TR8;
    EXTI->INTENR &= ~EXTI_INTENR_MR8;
    EXTI->INTFR = EXTI_INTENR_MR8;
    NVIC_DisableIRQ (EXTI9_5_IRQn);

    /* Install the no-mapper placeholder handler and enable the EXTI0
     * IRQ - the exact boot order of the first working firmware2 port.
     * SetVTFIRQ writes VTFADDR[0] and VTFIDR[0]; NVIC_EnableIRQ sets
     * the per-IRQ enable bit in PFIC->IENR. Both must be done for VTF
     * dispatch to fire. main() installs Nextor immediately after this
     * returns via Cart_SetMapper(). */
    SetVTFIRQ ((uint32_t)Cart_EXTI0_None_Handler, EXTI0_IRQn, 0, ENABLE);
    NVIC_EnableIRQ (EXTI0_IRQn);
    NVIC_SetPriority (EXTI0_IRQn, 0x00);
    __enable_irq();
}

/* ------------------------------------------------------------------ */
/* MSX ~WAIT line (PE3) - optional diagnostic hold.                    */
/* ------------------------------------------------------------------ */
/* Normal startup never drives ~WAIT. These helpers are retained for
 * the USB layer's PSRAM DMA staging (they bracket a DMA run so it
 * cannot race a Z80 bus cycle); a long hold can stop the MSX, so never
 * wrap normal initialization with them. The release path returns PE3 to
 * a floating input and never drives the shared line high. */

/* Assert cartridge ~WAIT (PE3) for a diagnostic. Idempotent. */
void Cart_HoldMSXWait_Begin (void) {
    RCC_PB2PeriphClockCmd (RCC_PB2Periph_GPIOE, ENABLE);
    /* Drop the output latch LOW before flipping PE3 to an output - any
     * glitch high during the mode change would let the BIOS probe slip
     * past our hold. */
    GPIOE->OUTDR &= ~CART_WAIT_MASK;
    /* 0x3 nibble = 50 MHz push-pull output. The pin stays low because
     * OUTDR bit 3 was cleared above. */
    GPIOE->CFGLR = (GPIOE->CFGLR & ~(0xFU << 12)) | (0x3U << 12);
    s_msx_wait_held = 1U;
}

/* Release cartridge ~WAIT (PE3). Returns PE3 to a floating input,
 * waits for the held bus cycle to actually finish (SLTSL rises), and
 * drives the data bus off so no stale OUTDR value lingers after the
 * cycle. Must be called with global IRQs disabled around the wait
 * loop so the held cycle is served exactly once. */
void Cart_HoldMSXWait_End (void) {
    /* Release ~WAIT by returning PE3 to floating input, NEVER by
     * driving it high (wired-OR bus; see comment above). */
    GPIOE->CFGLR = (GPIOE->CFGLR & ~(0xFU << 12)) | (0x4U << 12);
    /* Wait for the held Z80 cycle to actually finish. The previous
     * handler entry (the one we are releasing) holds SLTSL low while
     * ~WAIT is asserted; once we release ~WAIT the cycle completes
     * in a few cycles. Bounded spin to avoid wedging the firmware if
     * something is wrong (the spin-out path leaves s_msx_wait_held=0
     * so subsequent cart cycles are served normally). */
    uint32_t spin = 200000U;       /* ~2 ms at 200 MHz HCLK */
    while (((GPIOE->INDR & CART_SLTSL_MASK) == 0U) && (--spin)) {
        __asm__ volatile ("nop");
    }
    GPIOB->CFGHR = CART_BUS_OFF;
    s_msx_wait_held = 0U;
}

/* ------------------------------------------------------------------ */
/* Nextor mapper: expanded slot + ASCII16K kernel ROM + mailbox.       */
/* ------------------------------------------------------------------ */
/* Kept in flash: the ROM is flash-resident (zero-wait XIP), so there is
 * nothing to gain by paying the PSRAM cost of a .ramfunc body. */

void Cart_EXTI0_Nextor_Handler (void) {
    /* EXTI0 fires on the falling edge of ~SLTSL. ~RD and ~WR lag ~SLTSL by
     * a gate delay, so sampling them at entry would classify EVERY cycle as
     * a write (both still high) and the read path would never run - that is
     * exactly the rr=0/rw=huge signature seen on hardware. Wait for one of
     * the strobes to settle first, bailing out if ~SLTSL already rose. */
    const uint16_t entry_address = (uint16_t)GPIOD->INDR;
    uint32_t ctrl = GPIOE->INDR;

    if ((ctrl & CART_SLTSL_MASK) != 0U) {
        GPIOB->CFGHR = CART_BUS_OFF;
        EXTI->INTFR = EXTI_INTENR_MR0;
        return;
    }
    while ((ctrl & (CART_RD_MASK | CART_WR_MASK)) ==
           (CART_RD_MASK | CART_WR_MASK)) {
        ctrl = GPIOE->INDR;
        if ((ctrl & CART_SLTSL_MASK) != 0U) {
            GPIOB->CFGHR = CART_BUS_OFF;
            EXTI->INTFR = EXTI_INTENR_MR0;
            return;
        }
    }

    uint16_t address = (uint16_t)GPIOD->INDR;
    /* Preserve a write to the SSR if the address bus changes while the
     * write strobe settles (observed on some MSX bus timings). */
    if ((ctrl & CART_WR_MASK) == 0U
        && entry_address == 0xFFFFU
        && address != 0xFFFFU) {
        address = 0xFFFFU;
    }

    /* Only memory requests belong to the expanded-slot/ROM handler. The
     * mapper's FC..FF bank ports are I/O requests and use EXTI8 instead. */
    if ((ctrl & CART_MREQ_MASK) != 0U) {
        EXTI->INTFR = EXTI_INTENR_MR0;
        GPIOB->CFGHR = CART_BUS_OFF;
        if (s_msx_wait_held == 0U) {
            while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        }
        return;
    }

    if ((ctrl & CART_RD_MASK) == 0U) {
        /* READ cycle: mailbox reads come from the live FIFO/status state;
         * every other address is a normal ROM read from the selected bank. */
        if (Cart_SubslotRead (address, ctrl)) {
            /* Expanded-slot register or sub-slot 1+ driven the bus. */
        } else if ((address & 0xFFF0U) == NEXTOR_MBOX_BASE) {
            /* nextor.c owns FIFO advancement and status bits. This handler
             * only routes the bus cycle and drives the returned byte. */
            const uint8_t v = Nextor_ReadByte (address);
            GPIOB->OUTDR = (GPIOB->OUTDR & ~(0xFFU << 8))
                         | ((uint32_t)v << 8);
            GPIOB->CFGHR = CART_BUS_ON;
        } else {
            /* ASCII16K data flow: select the bank offset for this page and
             * index the embedded ROM. The unsigned wrap is intentional -
             * (bank << 14) - 0x4000 for bank 0 is 0xFFFFC000, and
             * adding address 0x4000 wraps back to exactly 0. */
            const uint32_t bias = (address < 0x8000U)
                                ? g_state->bankOffsets[0]
                                : g_state->bankOffsets[8];
            Cart_DriveByteFromFlash (nextor_rom, bias + (uint32_t)address);
        }
        /* Acknowledge EXTI before waiting for SLTSL where possible, then
         * turn the cartridge data bus off after the Z80 releases the cycle. */
        EXTI->INTFR = EXTI_INTENR_MR0;
        if (s_msx_wait_held == 0U) {
            while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
            GPIOB->CFGHR = CART_BUS_OFF;
        }
        /* During startup the held bus cycle is completed by
         * Cart_HoldMSXWait_End(); keep read data driven until then. */
        return;
    }

    /* WRITE cycle: WR is low and the Z80's data byte is valid on PB8..15.
     * The strobes were already settled by the entry wait above.
     * Mailbox writes feed the command/argument collector; bank writes update
     * the ASCII16K page offsets. Other writes are intentionally ignored. */
    if ((ctrl & CART_WR_MASK) == 0U) {
        const uint8_t w = (uint8_t)(GPIOB->INDR >> 8);
        if (Cart_SubslotWrite (address, ctrl, w)) {
            /* Expanded-slot register or sub-slot 1+ mapper RAM write. */
        } else if ((address & 0xFFF0U) == NEXTOR_MBOX_BASE) {
            /* Mailbox command byte or argument byte. Nextor_WriteByte keeps
             * this path bounded because it runs inside the bus interrupt. */
            Nextor_WriteByte (address, w);
        } else if (address == NEXTOR_BANK_REG_PAGE1) {
            g_state->bankOffsets[0] = ((uint32_t)w << NEXTOR_BANK_SHIFT)
                                    - 0x4000U;
        } else if (address == NEXTOR_BANK_REG_PAGE2 ||
                   address == NEXTOR_BANK_REG_PAGE2_ALT) {
            g_state->bankOffsets[8] = ((uint32_t)w << NEXTOR_BANK_SHIFT)
                                    - 0x8000U;
        }
        /* Other writes are ignored. A bank number past the end of the ROM
         * (w >= 8 for the 128 KiB image) is latched like any other; the
         * read path's off < nextor_rom_len test then floats those
         * addresses to 0xFF instead of indexing out of bounds. */
        EXTI->INTFR = EXTI_INTENR_MR0;
        if (s_msx_wait_held == 0U) {
            while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
            GPIOB->CFGHR = CART_BUS_OFF;
        }
        return;
    }

    /* Neither RD nor WR - spurious; release. */
    EXTI->INTFR = EXTI_INTENR_MR0;
    if (s_msx_wait_held == 0U) {
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
        GPIOB->CFGHR = CART_BUS_OFF;
    }
}

/* ------------------------------------------------------------------ */
/* None handler: bus floats. Z80 reads 0xFF (open-bus pattern).        */
/* ------------------------------------------------------------------ */
/* Placeholder between Init_Cart() and the Cart_SetMapper(NEXTOR) call
 * in main() - a window of a few hundred microseconds while the MSX is
 * normally still in its own power-on reset. This is the first working
 * firmware2 port's design (and /firmware's): every Z80 cycle in the
 * window gets a properly consumed floating-bus read (0xFF).
 * Plain C: the data bus is never driven here, so there is nothing to
 * release and no ordering hazard. */

void Cart_EXTI0_None_Handler (void) {
    EXTI->INTFR = EXTI_INTENR_MR0;
    if (s_msx_wait_held == 0U) {
        /* Wait for the Z80 cycle to finish so the next falling edge is
         * the next cycle (one IRQ per bus cycle, same contract as the
         * Nextor handler). */
        while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) { }
    }
}

#pragma GCC pop_options
