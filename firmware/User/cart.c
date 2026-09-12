#include "cart.h"
#include "psram.h"
#include "scc.h"
#include "ch32v4x7.h"
#include "debug.h"

#pragma GCC push_options
#pragma GCC optimize("Ofast")

/*
 * Cart emulation for RISKYMSX2 (CH32V407V).
 *
 * One PSRAM-backed 8 MiB image window at PSRAM_CART_BASE (0x80000000).
 * The active mapper is selected at runtime; the matching EXTI0 handler
 * is installed in the PFIC VTF slot by Cart_SetMapper().
 *
 * Two classes of mapper:
 *
 *   1. Simple ROM (no bank-switching writes)
 *      ROM16k, ROM32k, ROM48k
 *      -> Hand-scheduled asm handlers (Cart_EXTI0_ROM16k_Handler, etc.)
 *         that mirror the structure of the original Cart_EXTI0_PSRAM_Handler
 *         in this file. The bias is folded into an immediate so the
 *         handler needs zero state from RAM. PSRAM read latency is
 *         hidden by reading the byte BEFORE enabling the data-bus
 *         drivers.
 *
 *   2. Bank-switching mappers (Konami, ASCII, NEO)
 *      -> C handlers (RunKonami, RunKonamiSCC, Run8kASCII, Run16kASCII,
 *         RunNEO8, RunNEO16) that read bank state from `s_state` in
 *         zero-wait-state SRAM, plus three hand-scheduled asm handlers
 *         (Cart_EXTI0_KonamiNOSCC_Handler for KONAMINOSCC,
 *         Cart_EXTI0_ASCII8k_Handler for ASCII8k and
 *         Cart_EXTI0_ASCII16k_Handler for ASCII16k) that serve their
 *         mapper as a direct VTF entry without the dispatcher hop.
 *         Reads hit PSRAM
 *         (~30 cycle latency); the Z80's data sample point still has
 *         comfortable margin because EXTI0 fires on the falling edge
 *         of SLTSL which precedes ~RD by several T-states.
 *
 * SCC: KONAMISCC (Konami-with-SCC, port of the legacy v303 mapper)
 *       banks like KONAMINOSCC but routes the 0x9800..0x98FF window
 *       to the SCC emulator (scc.c / emu2212) - see RunKonamiSCC.
 *
 * Mapper selection lives in `g_mapper`. Cart_SetMapper() writes both
 * `g_mapper` and the VTF slot atomically relative to the Z80 bus
 * (the VTF slot rewrite is a single register write; g_mapper is only
 * read at the start of an invocation, never mid-cycle).
 */

/* ------------------------------------------------------------------ */
/* Cart state, in zero-wait-state SRAM. The bank-switching handlers    */
/* mutate this struct. Field layout chosen to keep hot fields in the   */
/* first 16 bytes (one cache line / one lbu window).                    */
/* ------------------------------------------------------------------ */

struct MSXState {
    uint32_t bankOffsets[16]; /* bank bias = (page * bankSize) - (Z80 - 0x4000) */
} s_state;

/* Active mapper. Read by the EXTI0 dispatcher trampoline (Cart_EXTI0_
 * Dispatch, below) on every IRQ entry. Writes happen only inside
 * Cart_SetMapper() which is called from the main loop, never from the
 * IRQ, so the dispatcher's read is race-free relative to mapper swaps. */
volatile Cart_Mapper g_mapper = CART_MAP_NONE;

/* Pointer to s_state used by the bank-switching handlers. Decoupled
 * from `&s_state` so a future port can relocate the state without
 * touching every handler. */
static struct MSXState *const g_state = &s_state;

/* Active EXTI0 handler trampoline address (PFIC VTF slot 0). Cart_
 * SetMapper() rewrites this. The trampoline reads g_mapper and jumps
 * to the right Run<Mapper>() function - ONE trampoline serves all
 * bank-switching mappers. The simple-ROM mappers get their own VTF
 * entry directly so the hot path is one indirect jump, not two. */
static void Cart_Banked_Dispatch (void) __attribute__((section(".ramfunc"), noinline,
                                                         interrupt("WCH-Interrupt-fast")));

/* Per-mapper C handlers. Each is called from Cart_Banked_Dispatch
 * via a normal jal. They are PLAIN C functions (no interrupt
 * attribute) - the interrupt prologue/epilogue (mret) lives in
 * Cart_Banked_Dispatch, which is the actual VTF entry point.
 * (An asm translation of RunKonamiNOSCC is documented below as a
 * future optimization; the live C body is restored for now.) */
static void RunKonamiNOSCC (void) __attribute__((section(".ramfunc"), noinline));
static void RunKonami  (void) __attribute__((section(".ramfunc"), noinline));
static void RunKonamiSCC (void) __attribute__((section(".ramfunc"), noinline));
static void Run8kASCII (void) __attribute__((section(".ramfunc"), noinline));
static void Run16kASCII(void) __attribute__((section(".ramfunc"), noinline));
static void RunNEO8    (void) __attribute__((section(".ramfunc"), noinline));
static void RunNEO16   (void) __attribute__((section(".ramfunc"), noinline));

/* Hand-scheduled asm handlers. One per mapper that needs the tightest
 * read latency (ROM16/32/48, KONAMINOSCC) - each gets its own VTF slot
 * entry so Cart_SetMapper() picks it directly, with no intermediate
 * dispatch. */
void Cart_EXTI0_KonamiNOSCC_Handler (void) __attribute__((section(".ramfunc"), noinline,
                                                            interrupt("WCH-Interrupt-fast")));
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
};

uint32_t Cart_GetImageBase (void) { return PSRAM_CART_BASE; }
uint32_t Cart_GetImageSize (void) { return PSRAM_CART_SIZE; }
Cart_Mapper Cart_GetMapper (void) { return g_mapper; }

int Cart_SetMapper (Cart_Mapper m) {
    if ((unsigned)m >= CART_MAP_MAX) return -1;
    if (m != CART_MAP_NONE && PSRAM_GetRomMirrorBase() == 0U) return -1;

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
    default:                   return -1;
    }
    SetVTFIRQ (h, EXTI0_IRQn, 0, ENABLE);

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
        s_state.bankOffsets[2] = 0U - 0x4000U;  /* 0x4000..0x5FFF -> PSRAM[0..0x1FFF] */
        s_state.bankOffsets[3] = 0U - 0x6000U;  /* 0x6000..0x7FFF -> PSRAM[0..0x1FFF] */
        s_state.bankOffsets[4] = 0U - 0x8000U;  /* 0x8000..0x9FFF -> PSRAM[0..0x1FFF] */
        s_state.bankOffsets[5] = 0U - 0xA000U;  /* 0xA000..0xBFFF -> PSRAM[0..0x1FFF] */
    } else if (m == CART_MAP_KONAMINOSCC || m == CART_MAP_KONAMISCC) {
        /* Konami-without-SCC variant: same layout as KONAMI but bank
         * writes are accepted at any address in 0x4000..0xBFFF, not
         * just 0x6000/0x8000/0xA000. Same bias math. KONAMISCC shares
         * the identical bank layout AND the legacy v303 initial-bank
         * defaults (bankOffsets[2]=-0x4000, [3]=-0x6000, [4]=-0x8000,
         * [5]=-0xA000 - see Init_Cart's KonamiWithSCC case there),
         * which pre-map pages 1..3 to image offset 0 before any
         * bank-select write. */
        s_state.bankOffsets[2] = 0U - 0x4000U;
        s_state.bankOffsets[3] = 0U - 0x6000U;
        s_state.bankOffsets[4] = 0U - 0x8000U;
        s_state.bankOffsets[5] = 0U - 0xA000U;
        if (m == CART_MAP_KONAMISCC) {
            /* Bring up the SCC emulator core (emu2212) + its DMA->DAC
             * sample pump. Safe to call repeatedly; scc.c ignores a
             * second init while running. */
            SCC_Init();
        }
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
     * compose the 12-bit bank number, no init needed. */

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
     * inputs. PE3 = WAIT (left floating - see Cart_AssertMSXReset if
     * we ever want to drive it). PE4 = MSX ~RESET (floating - the MSX
     * reset circuit controls it). */
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
/* Cart_AssertMSXReset - PE4 pulse                                     */
/* ------------------------------------------------------------------ */

void Cart_AssertMSXReset (uint32_t ms) {
    GPIO_InitTypeDef io = {0};
    io.GPIO_Pin   = GPIO_Pin_4;
    io.GPIO_Mode  = GPIO_Mode_Out_PP;
    io.GPIO_Speed = GPIO_Speed_High;
    GPIO_Init (GPIOE, &io);
    GPIO_SetBits   (GPIOE, GPIO_Pin_4);
    GPIO_ResetBits (GPIOE, GPIO_Pin_4);
    Delay_Ms (ms);
    GPIO_SetBits (GPIOE, GPIO_Pin_4);
    io.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init (GPIOE, &io);
}

/* ------------------------------------------------------------------ */
/* Shared C-side helpers used by the bank-switching mappers.           */
/* ------------------------------------------------------------------ */

/* Drive one byte from PSRAM onto GPIOB[15:8] and turn on the drivers. */
static inline __attribute__((always_inline))
void Cart_DriveByteFromPSRAM (uint32_t addr, uint32_t bias) {
    /* Read the byte with the bus STILL TRI-STATED. PSRAM read takes
     * ~30 cycles; the old handler relied on this exact order to avoid
     * driving a stale OUTDR value during the controller latency. */
    uint8_t b = *(const volatile uint8_t *)(PSRAM_CART_BASE + addr + bias);
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
 * NOTE: this C body is the reference implementation - the asm
 * Cart_EXTI0_KonamiNOSCC_Handler replaces it at runtime when
 * KONAMINOSCC is selected. Kept here as a sanity-check reference.
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
 * Konami mapper with SCC sound chip (port of legacy RunKonamiWithSCC).
 *
 * Bank layout is identical to RunKonamiNOSCC (page 1 = 16 KiB header
 * at image 0, pages 2..3 = user-selectable 8 KiB banks), with two SCC
 * differences on top:
 *
 *   1. The 0x9800..0x98FF window is the SCC-I register file, not ROM:
 *      READS are answered by the emulator (Cart_SCC_ReadByte) and
 *      WRITES there are queued for the SCC emulator core (emu2212)
 *      instead of switching banks - exactly like real Konami-with-SCC
 *      hardware and the legacy v303 firmware.
 *   2. EVERY accepted write (bank select or SCC register) is packed as
 *      ((address << 16) | data) and queued to scc.c, which applies it
 *      to the emulator on the next sample tick. This keeps the write
 *      latch out of the hot path - the handler only stores, never
 *      calls into the emulator.
 *
 * NOTE: like RunKonamiNOSCC, this port uses the corrected bank-bias
 * formula (w << 13) - page_start; the legacy v303 firmware used a
 * buggy (w << 13) - (addr - 0x1000) here. Standard Konami SCC images
 * are written against the corrected semantics.
 */
static void RunKonamiSCC (void) {
    const uint16_t address = (uint16_t)GPIOD->INDR;
    const uint32_t ctrl    = GPIOE->INDR;
    const uint32_t page    = address >> 13;  /* 2..5 valid (0x4000..0xBFFF) */

    if ((ctrl & CART_RD_MASK) == 0U) {
        /* READ. The SCC-I register window is answered by the
         * emulator, not by the ROM image. */
        if (address >= SCC_I_WINDOW_BASE && address <= SCC_I_WINDOW_LAST) {
            int v = Cart_SCC_ReadByte (address);
            GPIOB->OUTDR = ((uint32_t)(uint8_t)v) << 8;
            GPIOB->CFGHR = CART_BUS_ON;
        } else if (page >= 2U && page <= 5U) {
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

    /* WRITE: bank select anywhere in 0x4000..0xBFFF (like NOSCC) plus
     * SCC-I register writes. Every accepted write is queued for the
     * emulator (bank-select writes included, mirroring the legacy
     * handler which queued all of them). */
    EXTI->INTFR = EXTI_INTENR_MR0;
    if (address > 0xB000U) return;  /* ignore writes outside cart range */
    while ((GPIOE->INDR & CART_SLTSL_MASK) == 0U) {
        if ((GPIOE->INDR & CART_WR_MASK) == 0U) {
            const uint8_t w = Cart_ReadWriteData();

            /* Queue for the emulator (SCC_write() ignores addresses
             * outside the SCC-I/base_adr window, so bank-select writes
             * in the queue are harmless - same behaviour as legacy). */
            SCC_QueueWrite (((uint32_t)address << 16) | (uint32_t)w);

            /* Bank-select write (outside the SCC-I window) also
             * updates the mapper state so ROM reads follow it. */
            if (address < SCC_I_WINDOW_BASE || address > SCC_I_WINDOW_LAST) {
                const uint32_t page_start = (uint32_t)(address & 0xE000U);
                g_state->bankOffsets[page] = ((uint32_t)w << 13) - page_start;
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
        "lui   a2, 0x80000                 \n" /* PSRAM base               */
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
 *   if (RD low) {                       // read cycle
 *       if (addr < 0x4000 || addr >= 0xC000) bus off + tail
 *       slot  = (addr >> 13) - 2          // 0..3, page-2 offset
 *       drive PSRAM[addr + bankOffsets[slot]]
 *       INTFR clear; wait SLTSL high; bus off
 *   } else {                            // write cycle
 *       INTFR clear;
 *       if (addr > 0xB000) return;
 *       while (SLTSL low) {
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
        /* SLTSL low but RD high: a write, or ~RD has not fallen yet.
         * The C path samples ~RD after the dispatcher hop (always
         * settled); a single sample here races the SLTSL->RD gate
         * delay and misclassifies early READ cycles as writes (Z80
         * sees 0xFF all cycle - the observed failure). Poll until RD
         * low (read) or WR low (write). */
        "3:                                \n"
        "lw    a6, -2040(t1)               \n"
        "andi  a7, a6, 1                   \n"
        "bnez  a7, 2f                      \n" /* SLTSL rose -> cycle over */
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
        /* ---- READ CYCLE ---- */
        "1:                                \n"
        "srli  a1, a0, 13                  \n" /* a1 = page (0..7)         */
        "addi  a2, a1, -2                  \n" /* a2 = slot = page - 2     */
        "sltiu a7, a2, 4                   \n" /* slot 0..3 ? (page 2..5)  */
        "beqz  a7, 6f                      \n" /* out of range -> tail     */
        "5:                                \n"
        "slli  a1, a2, 2                   \n" /* slot * 4 (a2 still slot) */
        "lui   a2, %%hi(s_state)           \n"
        "add   a2, a2, a1                  \n" /* %hi(s_state) + slot*4    */
        "lw    a3, %%lo(s_state)(a2)       \n" /* a3 = bankOffsets[slot]   */
        "lui   a2, 0x80000                 \n" /* PSRAM base               */
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
        "lui   a2, 0x80000                 \n" /* PSRAM base               */
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
        "lui   a1, 0x7FFFC                 \n" /* bias = 0x80000000 - 0x4000 */
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

/* ROM32k: 32 KiB image at 0x4000..0xBFFF. Same shape as ROM16k (the
 * mirror has no effect on the handler - 0x4000 <= addr < 0xC000 in
 * either case maps to img[0..0x7FFF]). */
void Cart_EXTI0_ROM32k_Handler (void) {
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
        "lui   a1, 0x7FFFC                 \n"
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
        "lui   a1, 0x7FFFC                 \n"
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