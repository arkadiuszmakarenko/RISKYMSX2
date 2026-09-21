/*
 * SCC sound-chip emulation glue for RISKYMSX2 (CH32V407V).
 *
 * Architecture (mirrors v303Firmware/User/scc.c, retargeted):
 *
 *   Z80 write to cart window (KONAMISCC mapper, cart.c)
 *        |  EXTI0 handler, IRQ context
 *        v
 *   SCC_QueueWrite(addr<<16|data)   ->  s_scc_queue[64] SPSC ring
 *        |
 *        |  TIM4 update event
 *        v
 *   TIM4_IRQHandler: drain queue -> SCC_write(); Dual_DAC_Value =
 *        ((SCC_calc() + 0x8000 + 8) >> 4)
 *        |  DMA2_Channel3 (hardwired request: DAC1)
 *        v
 *   DAC->RD12BDHR (dual 12-bit right-aligned; ch1 = sample on PA4)
 *
 * Timer clock: TIM4 is a general-purpose timer clocked from PB1. On
 * the CH32V4x7, TIMxCLK = PCLK1 * (PPRE1 == 1 ? 1 : 2) (clock-tree
 * figure 3-2, "if(PB1 prescaler=1)*1 else *2"). The v303 firmware ran
 * at 144 MHz SYSCLK / PPRE1=DIV2 => PCLK1 72 MHz => TIM4CLK 144 MHz,
 * with a hardcoded period 3368 => 42.7 kHz sample rate. This port
 * derives both the TIM4 clock and the period at runtime from
 * HCLKClock so any clock profile keeps the same ~44.1 kHz-ish rate.
 */

#include "scc.h"
#include "emu2212.h"
#include "ch32v4x7.h"
#include "core_riscv.h"
#include "system_ch32v4x7.h"
#include "debug.h"
#include <string.h>


/* ------------------------------------------------------------------ */
/* Emulator core (verbatim emu2212) instance.                          */
/* ------------------------------------------------------------------ */

/* Static instance: emu2212.c allocates via malloc(); there is no heap
 * in this firmware, so we replicate what SCC_new() does (zeroed struct
 * + clk/rate + refresh) against a static object and call the untouched
 * public API on it. */
static SCC s_scc_core;
static SCC *s_scc = 0;

/* Legacy sample target: 44100 Hz (emu2212's default when rate != 0). */
#define SCC_SAMPLE_RATE   44100U

/* MSX SCC chip clock - the rate SCC_writeReg() divides by for its
 * phase increment (passed as `c` to SCC_new on the legacy firmware). */
#define SCC_CHIP_CLK     3554685U    /* 3.58 MHz / 1024 (legacy value) */

/* ------------------------------------------------------------------ */
/* SPSC queue of packed cart writes (legacy CircularBuffer equivalent).*/
/* ------------------------------------------------------------------ */

#define SCC_QUEUE_SIZE   64U         /* power of two */
#define SCC_QUEUE_MASK   (SCC_QUEUE_SIZE - 1U)

static volatile uint32_t s_scc_queue[SCC_QUEUE_SIZE];
static volatile uint32_t s_q_head;    /* producer (EXTI0 cart IRQ) */
static volatile uint32_t s_q_tail;    /* consumer (TIM4 IRQ)       */

/* IRQ context (EXTI0 cart write) hot path: must run from zero-wait-
 * state SRAM - a flash call here adds ~50-100 cycles of latency inside
 * the SLTSL-low window, past the Z80's data sample point. */
int SCC_QueueWrite (uint32_t word) __attribute__((section(".ramfunc"), noinline));
int SCC_QueueWrite (uint32_t word) {
    uint32_t next = (s_q_head + 1U) & SCC_QUEUE_MASK;
    if (next == s_q_tail) {
        return -1;                    /* full: drop like legacy overflow */
    }
    s_scc_queue[s_q_head] = word;
    s_q_head = next;
    return 0;
}

uint32_t SCC_GetLevel (void) {
    return (s_q_head - s_q_tail) & SCC_QUEUE_MASK;
}

/* (Diagnostics accessor for the DMA-target RAM word lives further
 * down, AFTER the static volatile Dual_DAC_Value definition so the
 * extern machinery isn't needed.) */

/* Drain the queue (main-loop / Cart_SetMapper_Safe context ONLY).
 * See scc.h. */
uint32_t SCC_FlushQueue (void) {
    uint32_t drained = 0;
    /* Snapshot under IRQ-off discipline; the caller (Cart_SetMapper_
     * Safe / cmd_softreset) runs with global IRQ disabled around this
     * call so the TIM4 IRQ can't sneak in mid-drain. */
    while (s_q_head != s_q_tail) {
        const uint32_t word = s_scc_queue[s_q_tail];
        const uint16_t addr = (uint16_t)(word >> 16);
        const uint8_t  data = (uint8_t)word;
        SCC_write (s_scc, addr, data);
        s_q_tail = (s_q_tail + 1U) & SCC_QUEUE_MASK;
        drained++;
        /* Paranoia: 256 entries is more than the queue can hold; bail
         * if the index got out of sync (corrupted head/tail). The
         * caller would otherwise spin forever. */
        if (drained > SCC_QUEUE_SIZE) {
            s_q_tail = s_q_head;
            break;
        }
    }
    return drained;
}

/* ------------------------------------------------------------------ */
/* DAC output latch, DMAed to DAC->RD12BDHR on every timer trigger.    */
/* ------------------------------------------------------------------ */

static volatile uint32_t Dual_DAC_Value;

/* ------------------------------------------------------------------ */
/* Timer period computation                                            */
/* ------------------------------------------------------------------ */

/* Target sample rate. 16-bit ATRLR limits how slow we can tick at a
 * given timer clock; at 175 MHz HCLK the 44.1 kHz period is 3968, so
 * 16 bits are comfortably enough at any supported profile. */
#define SCC_PERIOD_MAX     0xFFFFU

static uint32_t scc_tim4clk (void) {
    /* TIMxCLK = PCLK1 * (PPRE1 == 1 ? 1 : 2). PCLK1 == HCLK on every
     * profile this firmware ships (all clock setups use PPRE1_DIV1),
     * but compute it properly from the register so future profiles
     * stay correct. */
    uint32_t ppre1 = RCC->CFGR0 & RCC_PPRE1;
    uint32_t mult  = (ppre1 == RCC_PPRE1_DIV1) ? 1U : 2U;
    return HCLKClock * mult;
}

static uint32_t scc_sample_period (void) {
    uint32_t clk = scc_tim4clk();
    if (clk == 0U) {
        clk = 72000000U;              /* defensive: uninitialised clock */
    }
    uint32_t period = (clk + (SCC_SAMPLE_RATE / 2U)) / SCC_SAMPLE_RATE;
    if (period < 2U)         period = 2U;
    if (period > SCC_PERIOD_MAX) period = SCC_PERIOD_MAX;
    return period;
}

/* ------------------------------------------------------------------ */
/* IRQ                                                                 */
/* ------------------------------------------------------------------ */

void TIM4_IRQHandler (void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Drain the queue into the emulator, render the next sample. Same
 * structure as v303's DMA2_Channel3_IRQHandler (TC-driven there,
 * update-driven here) minus the DMA-flag bookkeeping. Tone driver
 * also uses TIM4 - if a tone is active we delegate to its
 * TIM4Tick hook instead of running the SCC sample pump. */
void TIM4_IRQHandler (void) {
    if (TIM_GetITStatus (TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit (TIM4, TIM_IT_Update);

        /* Apply all pending Z80-side writes so SCC_calc() sees the
         * emulator state as of now. */
        while (s_q_tail != s_q_head) {
            uint32_t address_data = s_scc_queue[s_q_tail];
            s_q_tail = (s_q_tail + 1U) & SCC_QUEUE_MASK;
            uint32_t address = (address_data >> 16) & 0xFFFFU;
            uint32_t data    = address_data & 0xFFFFU;
            SCC_write (s_scc, address, data);
        }



        /* int16 -> 12-bit unsigned for the DAC: round-and-shift, the
         * identical arithmetic the legacy firmware used. */
        Dual_DAC_Value = ((uint32_t)(SCC_calc (s_scc) + 0x8000 + 8)) >> 4;

    }
}

/* ------------------------------------------------------------------ */
/* Init / DeInit                                                       */
/* ------------------------------------------------------------------ */

/* Legacy SCC_new() equivalent without malloc: zeroed static struct,
 * clk/rate set, refresh computed. SCC_reset/set_type afterwards match
 * the legacy SCC_Init() sequence exactly. */
static void SCC_core_Create (void) {
    memset (&s_scc_core, 0, sizeof (s_scc_core));
    s_scc_core.clk  = SCC_CHIP_CLK;
    s_scc_core.rate = SCC_SAMPLE_RATE;
    s_scc_core.type = SCC_ENHANCED;   /* SCC_new default */
    s_scc = &s_scc_core;
}

/* Konami-with-SCC slot mapping (openMSX::RomKonamiSCC::writeMem):
 *   - SCC enable / SCC+ activation byte at 0x9000..0x97FF
 *     (cart writes 0x3F for SCC-I, 0x80 for SCC+)
 *   - SCC control registers at 0x9800..0x9FFE (offset 0x800..0x8FE)
 *     - 0x9800..0x987F (off 0x800..0x87F): wave table
 *     - 0x9880..0x9889 (off 0x880..0x889): freq
 *     - 0x988A..0x988E (off 0x88A..0x88E): volume
 *     - 0x988F           (off 0x88F)         : ch-enable
 *     - 0x98C0..0x98DF (off 0x8C0..0x8DF): mode/flags
 *
 * emu2212's SCC_reset() already defaults base_adr to 0x9000 - the
 * correct Konami-SCC base. An earlier version of this file
 * re-rebased to 0x9800 (a misread of the slot mapping as
 * "0x9800-based"), which made the activation byte at 0x9000 fall
 * outside the base_adr window (`adr < base_adr` early-out in
 * emu2212::SCC_write line 393), `active` stayed 0, and the SCC
 * stayed silent despite 11000+ writes queued. The fix is to leave
 * base_adr at the emu2212 default of 0x9000 - this function is
 * therefore just a sanity clamp. SCC_SetBase(SCC_I_WINDOW_BASE) keeps
 * the intent explicit at every call site. */
static void SCC_SetBase (uint32_t base_adr) {
    if (s_scc != 0) {
        s_scc->base_adr = base_adr;
    }
}

/* True when SCC_HwInit() has already configured TIM4 / DMA2 / DAC1.
 * SCC_Init() is the public entry point and consults this flag: on the
 * FIRST call it runs the full hardware config; on every subsequent
 * call it skips the hardware touch (which would reconfigure TIM4 /
 * DMA while they're live and is what caused the KONAMISCC cart
 * freeze after switching from the boot-time FLASH mapper) and only
 * resets the emulator + clears the SPSC queue. This makes SCC_Init
 * truly idempotent and safe to call from both main() and
 * Cart_SetMapper(KONAMISCC). */
static uint8_t s_scc_hw_up = 0U;

/* Forward decl - the full hardware init is split out so
 * SCC_Init() can call it only the first time. */
static void SCC_HwInit (void);

int SCC_Init (void) {
#if SCC_DEBUG
    static uint8_t inited = 0;
    if (!inited) {
        inited = 1;
        printf ("SCC: SCC_Init() entry\r\n");
    }
#endif
    /* Emulator core: create once, reset on every init. */
    if (s_scc == 0) {
        SCC_core_Create();
#if SCC_DEBUG
        printf ("SCC: SCC_core_Create done (first call)\r\n");
#endif
    }
    SCC_reset (s_scc);
    SCC_set_quality (s_scc, 0);
    /* SCC+ (val=0x80 at base_adr) is required for Konami-SCC carts
     * (Nemesis, Space Manbow, Metal Gear 2, Gradius 2, ...). See the
     * `active` gate in emu2212.c::SCC_write lines 384-389 - the SCC+
     * activation branch is guarded by `scc->type == SCC_ENHANCED`.
     * Setting SCC_STANDARD here silently dropped the activation byte
     * on Konami-SCC carts and `SCC_calc` returned silence forever
     * (`active=0` -> all channels muted -> 0x800 every sample, the
     * exact symptom we just saw with last_sample=0x800 frozen for
     * 900k+ ticks). SCC_reset() already sets type=SCC_ENHANCED (the
     * SCC_new default) so the SCC_set_type() call is redundant with
     * the right default - kept explicit so the intent is obvious. */
    SCC_set_type (s_scc, SCC_ENHANCED);

    /* Rebase the SCC window to 0x9800 so KONAMI-SCC cart writes hit
     * the activation byte + register file. emu2212 leaves base_adr at
     * 0x9000 after SCC_reset, which is wrong for the KONAMI-SCC slot
     * mapping. Safe to call on every init - SCC_HwInit() guards the
     * rest of the HW so this never runs while DMA / TIM / DAC are
     * live. */
    SCC_SetBase (SCC_I_WINDOW_BASE);

    /* SPSC queue reset (matches legacy initBuffer(&cb)). Safe under
     * the IRQ-off discipline of the swap path. */
    s_q_head = 0U;
    s_q_tail = 0U;
#if SCC_DEBUG
    printf ("SCC: emulator reset (type=ENHANCED, base=0x%04x, "
            "queue cleared, active=%u mode=%u)\r\n",
            SCC_I_WINDOW_BASE,
            (unsigned)s_scc->active,
            (unsigned)s_scc->mode);
#endif

    /* Hardware init runs only the first time. Re-running it on every
     * mapper swap would reconfigure DMA2_Channel3 + TIM4 + DAC1 while
     * they're live (TIM4_IRQHandler can be in the middle of servicing
     * a DMA-driven sample when Cart_SetMapper runs because the loader
     * service loop has global IRQs enabled). The CH32V407's DMA
     * controller does NOT have a "pause + reconfigure" - touching
     * DMA_CFGR while the channel is enabled takes the channel out of
     * service for the duration of the reconfigure, which drops the
     * in-flight sample and (worse) can latch the DMA state machine if
     * the reconfigure crosses an AHB-to-APB1 bridge cycle. */
    if (!s_scc_hw_up) {
#if SCC_DEBUG
        printf ("SCC: SCC_HwInit() entering (first time)\r\n");
#endif
        SCC_HwInit ();
        s_scc_hw_up = 1U;
#if SCC_DEBUG
        printf ("SCC: SCC_HwInit() complete - TIM4=%u Hz, DMA=RD12BDHR, "
                "DAC=Ch1 PA4, IRQ=TIM4@0xC0\r\n",
                (unsigned)SCC_SAMPLE_RATE);
#endif
    } else {
#if SCC_DEBUG
        printf ("SCC: SCC_Init() idempotent path (HW already up)\r\n");
#endif
    }

    /* Re-arm the sample pump IRQ in case SCC_DeInit() left it
     * disabled (we enter here on mapper return from non-SCC map).
     * EnableIRQ is idempotent at the NVIC level. */
    NVIC_SetPriority (TIM4_IRQn, 0xC0);
    NVIC_EnableIRQ (TIM4_IRQn);
    TIM_ITConfig (TIM4, TIM_IT_Update, ENABLE);

    return 0;
}

/* One-shot hardware bring-up. Called only by SCC_Init() the first
 * time it runs - NEVER while DMA2 / TIM4 / DAC1 are live. The
 * mirror image, SCC_DeInit(), pairs with this for clean shutdown on
 * mapper leave. */
static void SCC_HwInit (void) {
    /* Clocks: DMA2 lives on the HB bus, DAC/TIM4 on PB1. */
    RCC_HBPeriphClockCmd (RCC_HBPeriph_DMA2, ENABLE);
    RCC_PB1PeriphClockCmd (RCC_PB1Periph_DAC | RCC_PB1Periph_TIM4, ENABLE);

    /* Tear down any leftover state from a previous driver pass (e.g.
     * Tone_Stop -> SCC_Init re-enters this path). Without the
     * disable/clear sequence below, the DMA controller can latch a
     * stale source address (s_table[] instead of Dual_DAC_Value) or
     * a half-configured TIM4 period. */
    DMA_Cmd (DMA2_Channel3, DISABLE);
    DAC_DMACmd (DAC_Channel_1, DISABLE);
    DAC_Cmd (DAC_Channel_1, DISABLE);
    TIM_Cmd (TIM4, DISABLE);
    TIM_ITConfig (TIM4, TIM_IT_Update, DISABLE);

    /* DAC1 output pin: PA4 (SCC_OUT on this board). Analog in - no
     * driver, no pull. (v303 wired SCC_OUT to a different pin; this is
     * the only GPIO change the port needs.) */
    GPIOA->CFGLR = (GPIOA->CFGLR & ~(0xFU << 16)) | (0x0U << 16);

    /* DAC channel 1: triggered by TIM4 TRGO (update), no wave gen,
     * output buffer on - identical to the legacy gpio.c setup. CRITICAL:
     * DAC_Cmd AND DAC_DMACmd are both required. DAC_Cmd enables the
     * analog channel; DAC_DMACmd lets the DMA controller's writes to
     * DAC->RD12BDHR actually reach the channel's holding register.
     * Without DAC_DMACmd the DMA transfers complete silently and the
     * analog output sits at whatever the last CPU-write set it to. */
    DAC_InitTypeDef dac_init = {0};
    dac_init.DAC_Trigger         = DAC_Trigger_T4_TRGO;
    dac_init.DAC_WaveGeneration  = DAC_WaveGeneration_None;
    dac_init.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    dac_init.DAC_OutputBuffer    = DAC_OutputBuffer_Enable;
    DAC_Init (DAC_Channel_1, &dac_init);
    DAC_Cmd (DAC_Channel_1, ENABLE);
    DAC_DMACmd (DAC_Channel_1, ENABLE);
    DAC_SetChannel1Data (DAC_Align_12b_R, 0x00);

    /* DMA2 channel 3: single-word circular transfer Dual_DAC_Value ->
     * DAC dual 12-bit right-aligned holding register. Word-sized write
     * hits DAC1 (low half) and DAC2 (high half) together; DAC2 is not
     * enabled so only DAC1/PA4 sees the samples - same as the legacy
     * firmware's config. */
    DMA_InitTypeDef dma_init = {0};
    DMA_StructInit (&dma_init);
    dma_init.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->RD12BDHR);
    dma_init.DMA_MemoryBaseAddr     = (uint32_t)&Dual_DAC_Value;
    dma_init.DMA_DIR                = DMA_DIR_PeripheralDST;
    dma_init.DMA_BufferSize         = 1;
    dma_init.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma_init.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma_init.DMA_MemoryDataSize     = DMA_MemoryDataSize_Word;
    dma_init.DMA_Mode               = DMA_Mode_Circular;
    dma_init.DMA_Priority           = DMA_Priority_VeryHigh;
    dma_init.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init (DMA2_Channel3, &dma_init);
    DMA_Cmd (DMA2_Channel3, ENABLE);

    /* TIM4: update event -> TRGO -> DAC trigger, period derived from
     * the live HCLK (see module header comment). */
    TIM_TimeBaseInitTypeDef tim_init = {0};
    tim_init.TIM_Prescaler         = 0;
    tim_init.TIM_Period            = (uint16_t)(scc_sample_period() - 1U);
    tim_init.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim_init.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInit (TIM4, &tim_init);

    TIM_SelectOutputTrigger (TIM4, TIM_TRGOSource_Update);
    TIM_Cmd (TIM4, ENABLE);

    /* Sample-pump IRQ: TIM4 update. Priority 0xC0 keeps it below the
     * cart EXTI0 (0x00) and the CLI USART1 (0x40), matching the legacy
     * DMA-IRQ priority. */
    TIM_ITConfig (TIM4, TIM_IT_Update, ENABLE);
    NVIC_SetPriority (TIM4_IRQn, 0xC0);
    NVIC_EnableIRQ (TIM4_IRQn);
}

void SCC_DeInit (void) {
    /* Symmetric teardown - turn off everything SCC_HwInit enabled so
     * a subsequent driver (e.g. Tone_Init) can reconfigure from a
     * known-clean state. Each step is its own /CRIT: DMA first (so
     * the DMA controller stops fetching from Dual_DAC_Value), then
     * DAC DMA path, then DAC channel itself, then TIM4 IRQ + clock. */
    DMA_Cmd (DMA2_Channel3, DISABLE);
    DAC_DMACmd (DAC_Channel_1, DISABLE);
    DAC_Cmd (DAC_Channel_1, DISABLE);

    TIM_Cmd (TIM4, DISABLE);
    TIM_ITConfig (TIM4, TIM_IT_Update, DISABLE);
    NVIC_DisableIRQ (TIM4_IRQn);

    /* Park the DAC at mid-scale so the pin does not sit at 0 V. */
    DAC_SetChannel1Data (DAC_Align_12b_R, 0x800U);

    /* CRITICAL: clear the HW-up flag so the next SCC_Init() forces a
     * full SCC_HwInit() pass. Without this, Tone_Stop() ->
     * SCC_Init() skips the HW init (the "idempotent fast path") and
     * leaves the DMA controller still pointing at the now-stale
     * s_table[] source address instead of Dual_DAC_Value. The SCC
     * sample pump would then loop old waveform-table bytes through
     * the DAC instead of freshly-computed SCC_calc() values. */
    s_scc_hw_up = 0U;
}

/* ------------------------------------------------------------------ */
/* MSX-side SCC-I register read path (KONAMISCC mapper).               */
/* ------------------------------------------------------------------ */

int Cart_SCC_ReadByte (uint16_t address) {
    if (s_scc == 0U) {
        return -1;                    /* emulator not initialised */
    }
    if (address < SCC_I_WINDOW_BASE || address > SCC_I_WINDOW_LAST) {
        return -1;                    /* not the SCC-I window */
    }

    uint32_t adr = (uint32_t)address - SCC_I_WINDOW_BASE;

    /* First window byte is the SCC-I presence/status byte: 0x3F in
     * standard mode, 0x80 in SCC+ mode - emu2212 models this via
     * SCC_read() on absolute address 0x9800 (base_adr). */
    uint32_t v = SCC_read (s_scc, (uint32_t)address);

    /* emu2212 SCC_read() returns 0 for the status byte unless active;
     * feed the window read straight through - SCC_read handles the
     * 0x9800 status byte and the 0x9811..0x98FF register file exactly
     * like the legacy read path. */
    (void)adr;
    return (int)(v & 0xFFU);
}