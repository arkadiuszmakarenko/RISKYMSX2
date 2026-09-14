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
 * update-driven here) minus the DMA-flag bookkeeping. */
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

int SCC_Init (void) {
    /* Emulator core: create once, reset + re-refresh on every init. */
    if (s_scc == 0) {
        SCC_core_Create();
    }
    SCC_reset (s_scc);
    SCC_set_quality (s_scc, 0);
    SCC_set_type (s_scc, SCC_STANDARD);

    /* Legacy scc.c called initBuffer(&cb) here; the SPSC queue is
     * reset the same way. */
    s_q_head = 0U;
    s_q_tail = 0U;

    /* Clocks: DMA2 lives on the HB bus, DAC/TIM4 on PB1. */
    RCC_HBPeriphClockCmd (RCC_HBPeriph_DMA2, ENABLE);
    RCC_PB1PeriphClockCmd (RCC_PB1Periph_DAC | RCC_PB1Periph_TIM4, ENABLE);

    /* DAC1 output pin: PA4 (SCC_OUT on this board). Analog in - no
     * driver, no pull. (v303 wired SCC_OUT to a different pin; this is
     * the only GPIO change the port needs.) */
    GPIOA->CFGLR = (GPIOA->CFGLR & ~(0xFU << 16)) | (0x0U << 16);

    /* DAC channel 1: triggered by TIM4 TRGO (update), no wave gen,
     * output buffer on - identical to the legacy gpio.c setup. */
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

    return 0;
}

void SCC_DeInit (void) {
    TIM_Cmd (TIM4, DISABLE);
    TIM_ITConfig (TIM4, TIM_IT_Update, DISABLE);
    NVIC_DisableIRQ (TIM4_IRQn);

    DMA_Cmd (DMA2_Channel3, DISABLE);
    DAC_DMACmd (DAC_Channel_1, DISABLE);

    /* Park the DAC at mid-scale so the pin does not sit at 0 V. */
    DAC_SetChannel1Data (DAC_Align_12b_R, 0x800U);
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