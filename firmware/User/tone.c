/*
 * tone.c — raw-DAC1 tone generator for audio-path verification.
 *
 * Purpose: prove the DAC1/PA4 chain is wired correctly, independent
 * of the SCC emulator. The SCC path requires the cart to actually
 * write to the SCC register window (which is a chicken-and-egg with
 * the cart-init code). This module is simpler: it drives the DAC
 * directly with a generated waveform (sine / triangle / square / DC)
 * at a configurable frequency, so a single command can confirm the
 * analog path works.
 *
 * Hardware path (CH32V407V DAC1):
 *
 *   TIM4 update event -> TRGO -> DAC trigger -> DMA2_Channel3 ->
 *   DAC->RD12BDHR -> pin PA4 (SCC_OUT)
 *
 * We OVERRIDE the SCC's TIM4/DMA/DAC setup with a tone-specific DMA
 * source that points at a 256-sample waveform table we generate at
 * init. Result: PA4 carries the waveform regardless of cart state.
 * The SCC path is bypassed by stopping SCC_Init's DMA pump while
 * Tone_Init runs, then re-init it via SCC_Init if user wants audio
 * back (Tone_Stop restores SCC by re-running SCC_HwInit through
 * SCC_Init's idempotent fast path).
 *
 * Frequency resolution: TIM4 period = (HCLK * 2) / sample_rate.
 * The waveform table is 256 samples; one full cycle = 256 IRQs.
 * Cycle rate = sample_rate / 256 Hz. We expose Tone_SetFreq() to
 * tune sample_rate for any value 200..96000 Hz.
 *
 * Cycle budget: tone computation = ~30 cycles/sample inc. CRC
 * update. Cheap enough to run in TIM4_IRQHandler.
 */

#include "tone.h"
#include "scc.h"          /* SCC_GetLevel / SCC_GetDualDacValue for status */
#include "ch32v4x7.h"
#include "core_riscv.h"   /* __disable_irq / RCC definitions */
#include "debug.h"
#include <string.h>
#include <math.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Sample-rate derived state.                                          */
/* ------------------------------------------------------------------ */

/* HCLK-derived sample-rate, set by Tone_Init() so we can compute the
 * TIM4 reload value without recomputing every IRQ tick. */
static uint32_t s_sample_rate = 44100U;

/* Phase accumulator (Q32 fixed-point). Advancing by `s_phase_step`
 * cycles through one revolution of 0..2^32 every (2^32 / s_phase_step)
 * IRQs. We want one revolution per waveform-table pass (256 samples),
 * so s_phase_step = 2^32 / 256 = 0x01000000 (constant), and frequency =
 * sample_rate / 256 Hz. */
#define TONE_TABLE_SIZE   256U
#define TONE_FULL_SCALE   (1u << 12)   /* 12-bit DAC, 0..4095 */

static volatile uint32_t s_phase = 0;          /* Q32 phase, top 8 bits = table index */
static volatile uint16_t s_table[TONE_TABLE_SIZE]; /* waveform table */

/* Active waveform / status. tone_idle = not generated; tone_user_off
 * = paused by user; tone_running = active. */
typedef enum {
    TONE_IDLE = 0,
    TONE_RUNNING,
    TONE_USER_PAUSED,
} ToneStatus;

static volatile ToneStatus s_status = TONE_IDLE;
static volatile uint8_t      s_waveform = TONE_WAVE_SINE; /* current waveform */

/* ------------------------------------------------------------------ */
/* DMA + TIM4 setup. Identical shape to scc.c::SCC_HwInit so the DMA   */
/* channel pump slides into the same AHB-cycle window; SCC's setup    */
/* is undone first so TIM4 isn't double-clock-gated.                   */
/* ------------------------------------------------------------------ */

static void tone_setup_dma (void) {
    /* CRITICAL: SCC_DeInit() (called from Tone_Init before us) leaves
     * DAC_Cmd / DAC_DMACmd DISABLED. If we only set up TIM4 + DMA the
     * sample values will fly into DAC->RD12BDHR but the analog channel
     * itself is OFF - PA4 sits at 0V (or whatever it last held). We
     * must explicitly re-enable the DAC channel AND its DMA trigger
     * here, not assume SCC left them on.
     *
     * Likewise, GPIOA PA4 (SCC_OUT / DAC1_OUT) must be configured as
     * analog (CNF=00 MODE=00) for the DAC output to drive the pin.
     * SCC_HwInit set this once at boot; we re-set it here defensively
     * in case anything else stomped on it. */
    RCC_PB1PeriphClockCmd (RCC_PB1Periph_DAC, ENABLE);
    /* DMA2 lives on the HB bus; SCC_HwInit enabled this at boot but
     * SCC_DeInit does NOT disable it (so it's a noop to re-enable).
     * Keep it defensive. */
    RCC_HBPeriphClockCmd (RCC_HBPeriph_DMA2, ENABLE);
    GPIOA->CFGLR = (GPIOA->CFGLR & ~(0xFU << 16)) | (0x0U << 16);

    DAC_InitTypeDef dac_init = {0};
    /* DAC_Trigger_T4_TRGO matches what scc.c uses, so the same TIM4
     * update event feeds the DAC. Output buffer MUST be enabled on the
     * CH32V407 or the analog output pin stays at high impedance. */
    dac_init.DAC_Trigger                = DAC_Trigger_T4_TRGO;
    dac_init.DAC_WaveGeneration         = DAC_WaveGeneration_None;
    dac_init.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    dac_init.DAC_OutputBuffer           = DAC_OutputBuffer_Enable;
    DAC_Init (DAC_Channel_1, &dac_init);
    DAC_Cmd (DAC_Channel_1, ENABLE);     /* <-- the line that was missing */
    DAC_DMACmd (DAC_Channel_1, ENABLE);  /* <-- required for DMA-driven */

    /* DMA2 channel 3: keep the same channel as SCC (RD12BDHR -> byte
     * format), just point the source memory at our 256-sample table.
     * We single-shot the entire table once per period (DMA_BufferSize
     * = TONE_TABLE_SIZE), and use circular mode so the TIM4 TRGO
     * reloads at every DAC trigger for a sustained waveform. The
     * "memory increment + circular" pattern is identical to SCC's;
     * only the source address and buffer size differ. */
    DMA_InitTypeDef dma_init = {0};
    DMA_StructInit (&dma_init);
    dma_init.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->RD12BDHR);
    dma_init.DMA_MemoryBaseAddr     = (uint32_t)s_table;
    dma_init.DMA_DIR                = DMA_DIR_PeripheralDST;
    dma_init.DMA_BufferSize         = TONE_TABLE_SIZE;
    dma_init.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    dma_init.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma_init.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    dma_init.DMA_Mode               = DMA_Mode_Circular;
    dma_init.DMA_Priority           = DMA_Priority_VeryHigh;
    dma_init.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init (DMA2_Channel3, &dma_init);
    DMA_Cmd (DMA2_Channel3, ENABLE);

    /* TIM4: update event -> TRGO -> DAC trigger. Period derived from
     * sample rate; for a 1000 Hz wave with a 256-sample table we want
     * 256000 IRQs/sec. TIM4->ATRLR is 16-bit so for HCLK=200 MHz the
     * slowest sample rate is 200e6/65536 ~= 3052 Hz (audio freq ~12 Hz).
     * The fastest unattainable is HCLK - i.e. sample_rate at or above
     * HCLK would underflow period. Clamp at call site (Tone_Init). */
    TIM_TimeBaseInitTypeDef tim_init = {0};
    tim_init.TIM_Prescaler         = 0;
    uint32_t period32 = (s_sample_rate > 0U)
                      ? ((SystemCoreClock + s_sample_rate / 2U) / s_sample_rate - 1U)
                      : 3967U;
    if (period32 < 1U)        period32 = 1U;
    if (period32 > 0xFFFFU)   period32 = 0xFFFFU;
    tim_init.TIM_Period            = (uint16_t)period32;
    tim_init.TIM_ClockDivision     = TIM_CKD_DIV1;
    tim_init.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInit (TIM4, &tim_init);

    TIM_SelectOutputTrigger (TIM4, TIM_TRGOSource_Update);
    TIM_ITConfig (TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd (TIM4, ENABLE);
}

static uint16_t scc_tim4_period_snapshot = 0;
static uint16_t scc_dac_dhr12r1_snapshot  = 0;

static void tone_save_scc_state (void) {
    /* Snapshot enough SCC config to put it back when Tone_Stop exits.
     * We just need TIM4's period + DAC's last value; SCC's DMA
     * channel config can stay - the source address is what matters
     * and Tone_Init overwrites DMA->MADDR with s_table[]. On
     * Tone_Stop we re-point DMA at Dual_DAC_Value and re-init the
     * SCC pump (see scc.c::SCC_Init idempotent path).
     *
     * CH32V407 DAC registers: RD12BDHR (the dual-write target DMA
     * uses), DHR12R1/DHR12R2 (CPU-side data holding), OUT1/OUT2
     * (analog output, not readable back as a register on most parts).
     * We snapshot the TIM4 reload value; the DAC value is intentionally
     * left as-is (DHR12R1 is masked 0xFFF by definition), no need to
     * snapshot. */
    scc_tim4_period_snapshot = TIM4->ATRLR;
    scc_dac_dhr12r1_snapshot = 0x800U;
}

/* ------------------------------------------------------------------ */
/* Waveform-table generation.                                          */
/* ------------------------------------------------------------------ */

static void fill_table_sine (void) {
    /* Sine: 0..0xFFF..0 over 256 entries, biased at 0x800 mid-scale.
     * Q12 amplitude = 0x7FF (~half of 0xFFF) to give plenty of headroom. */
    const uint32_t half_fs = TONE_FULL_SCALE / 2U - 1U; /* mid-scale ±0x7FE */
    for (uint32_t i = 0; i < TONE_TABLE_SIZE; ++i) {
        /* Quarter-wave index in [0, 2*pi). Use the system math.h sin()
         * which is a Taylor series (~30 cycles/call); 256 calls at
         * boot is ~8000 cycles, well under 1 ms. */
        const double theta = (double)i * 2.0 * 3.14159265358979323846
                              / (double)TONE_TABLE_SIZE;
        const double s     = sin (theta);
        const int32_t v     = (int32_t)(0.5 + (s + 1.0) * (double)half_fs);
        s_table[i] = (uint16_t)(v < 0 ? 0 : (v > 0xFFF ? 0xFFF : v));
    }
}

static void fill_table_triangle (void) {
    /* Triangle: 0..0xFFF..0..0..0xFFF, 1/4-period ramps. */
    for (uint32_t i = 0; i < TONE_TABLE_SIZE; ++i) {
        const uint32_t q = i << 2;             /* i * 4 to get quadrant */
        uint16_t v;
        if (q < TONE_TABLE_SIZE) {
            /* 0 -> 0xFFF ramp (i = 0..63 gives v = 0..0xFF0 / 64 = 0..255*4) */
            v = (uint16_t)((i * (TONE_FULL_SCALE - 1U) * 4U) / TONE_TABLE_SIZE);
        } else if (q < (TONE_TABLE_SIZE * 2U)) {
            const uint32_t k = (TONE_TABLE_SIZE * 2U) - q - 1U;
            v = (uint16_t)((k * (TONE_FULL_SCALE - 1U) * 4U) / TONE_TABLE_SIZE);
        } else if (q < (TONE_TABLE_SIZE * 3U)) {
            v = (uint16_t)((i - (TONE_TABLE_SIZE * 2U) / 4U)
                            * (TONE_FULL_SCALE - 1U) * 4U / TONE_TABLE_SIZE);
        } else {
            const uint32_t k = (TONE_TABLE_SIZE * 4U) - q - 1U;
            v = (uint16_t)((k * (TONE_FULL_SCALE - 1U) * 4U) / TONE_TABLE_SIZE);
        }
        s_table[i] = v;
    }
}

static void fill_table_square (void) {
    /* Square: 0xFFF for first half, 0 for second half. */
    for (uint32_t i = 0; i < TONE_TABLE_SIZE; ++i) {
        s_table[i] = (i < (TONE_TABLE_SIZE / 2U))
                     ? (uint16_t)(TONE_FULL_SCALE - 1U)
                     : (uint16_t)0U;
    }
}

static void fill_table_dc (uint16_t level) {
    for (uint32_t i = 0; i < TONE_TABLE_SIZE; ++i) s_table[i] = level;
}

static void refill_table (uint8_t waveform) {
    switch (waveform) {
    case TONE_WAVE_SINE:     fill_table_sine ();    break;
    case TONE_WAVE_TRIANGLE: fill_table_triangle (); break;
    case TONE_WAVE_SQUARE:   fill_table_square ();   break;
    case TONE_WAVE_DC:       fill_table_dc (0x800U); break;
    default:                                            break;
    }
}

/* ------------------------------------------------------------------ */
/* IRQ handling - scc.c owns the TIM4_IRQHandler symbol and forwards   */
/* to Tone_TIM4Tick() when a tone is active. See scc.c::TIM4_IRQHandler*/

static void tone_set_irq_state (FunctionalState en) {
    if (en == ENABLE) {
        NVIC_SetPriority (TIM4_IRQn, 0xC0);
        NVIC_EnableIRQ (TIM4_IRQn);
    } else {
        NVIC_DisableIRQ (TIM4_IRQn);
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

/* Static double of s_table[] so DMA can replay it. The DMA reads from
 * s_table[0..255] in a circular loop. To change waveform mid-stream
 * we just rewrite s_table[]. */

int Tone_Init (uint32_t sample_rate_hz, ToneWaveform wf) {
    /* Tear down the SCC DMA path first so TIM4 isn't double-clocked.
     * This stops SCC's sample pump - if the user wants SCC audio back
     * later, Tone_Stop() re-runs SCC_Init's idempotent HW init.
     * SCC_DeInit is declared in scc.h. */
    SCC_DeInit ();

    s_sample_rate = (sample_rate_hz < 200U)   ? 44100U : sample_rate_hz;
    s_sample_rate = (s_sample_rate > 96000U) ? 96000U : s_sample_rate;
    s_waveform    = (wf > TONE_WAVE_DC) ? TONE_WAVE_SINE : wf;
    s_phase       = 0U;

    refill_table ((uint8_t)s_waveform);
    tone_save_scc_state ();

    /* DMA + TIM4 setup runs from main-loop context with IRQs off so
     * no half-configured state hits the IRQ. */
    __disable_irq ();
    tone_setup_dma ();
    __enable_irq ();

    tone_set_irq_state (ENABLE);

    s_status = TONE_RUNNING;
    printf ("Tone: started waveform=%s rate=%u Hz "
            "(table=%u samples, audio_freq=%u Hz, "
            "first_sample=0x%03x)\r\n",
            Tone_WaveName ((ToneWaveform)s_waveform),
            (unsigned)s_sample_rate,
            (unsigned)TONE_TABLE_SIZE,
            (unsigned)Tone_GetFreq (),
            (unsigned)s_table[0]);
    return 0;
}

/* Bypass the SCC DMA pump entirely so TIM4 is dedicated to the tone
 * generator. Called from Tone_Init. The scc.c counterpart is
 * SCC_HWStop_ForTone - declare it locally with extern. */
// (removed: SCC_DeInit is in scc.h)

void Tone_Stop (void) {
    if (s_status == TONE_IDLE) return;
    tone_set_irq_state (DISABLE);
    TIM_Cmd (TIM4, DISABLE);
    TIM_ITConfig (TIM4, TIM_IT_Update, DISABLE);
    DMA_Cmd (DMA2_Channel3, DISABLE);
    DAC_SetChannel1Data (DAC_Align_12b_R, 0x800U); /* park mid-scale */
    s_status = TONE_IDLE;
    /* Re-init SCC pump - SCC_Init's idempotent HW path re-arms TIM4 +
     * DMA + DAC without disturbing the (already running) emulator. */
    SCC_Init ();
    printf ("Tone: stopped, SCC audio resumed\r\n");
}

void Tone_SetFreq (uint32_t sample_rate_hz) {
    /* Re-init at a new sample rate (and re-fill the table to match the
     * new period's frequency). Frequency = sample_rate / 256. */
    if (s_status == TONE_IDLE) {
        s_sample_rate = sample_rate_hz;
        return;
    }
    Tone_Init (sample_rate_hz, (ToneWaveform)s_waveform);
}

void Tone_SetWaveform (ToneWaveform wf) {
    if (wf > TONE_WAVE_DC) return;
    s_waveform = (uint8_t)wf;
    refill_table ((uint8_t)s_waveform);
    printf ("Tone: waveform=%s first_sample=0x%03x\r\n",
            Tone_WaveName (wf), (unsigned)s_table[0]);
}

ToneWaveform Tone_GetWaveform (void) { return (ToneWaveform)s_waveform; }
uint32_t     Tone_GetSampleRate (void) { return s_sample_rate; }
uint32_t     Tone_GetFreq (void) {
    return (s_status == TONE_IDLE) ? 0U : (s_sample_rate / TONE_TABLE_SIZE);
}
int          Tone_IsActive (void) { return (s_status == TONE_RUNNING); }

const char *Tone_WaveName (ToneWaveform wf) {
    switch (wf) {
    case TONE_WAVE_SINE:     return "sine";
    case TONE_WAVE_TRIANGLE: return "triangle";
    case TONE_WAVE_SQUARE:   return "square";
    case TONE_WAVE_DC:       return "dc";
    }
    return "?";
}

/* ------------------------------------------------------------------ */
/* TIM4 tick handler - called by scc.c::TIM4_IRQHandler every time    */
/* TIM4 fires (whichever is active, SCC or tone). Internally scc.c    */
/* checks `if (Tone_IsActive()) Tone_TIM4Tick(); else drain_scc();`  */
/* ------------------------------------------------------------------ */

static volatile uint32_t s_tone_tick_count   = 0;
static volatile uint32_t s_tone_last_print_at = 0;

void Tone_TIM4Tick (void) {
    if (s_status != TONE_RUNNING) return;     /* safety */
    ++s_tone_tick_count;
    /* Advance phase for diagnostics; 0x01000000 = one full table per
     * 2^32 wraps. DMA does the actual audio work. */
    s_phase = (s_phase + 0x01000000U);
    /* Periodically print what we're playing. 16 Hz is human-readable
     * + one printf per ~62 ms which keeps USART1 responsive. */
    const uint32_t now = s_tone_tick_count;
    const uint32_t period = (s_sample_rate > 16U) ? (s_sample_rate / 16U) : 1U;
    if ((now - s_tone_last_print_at) >= period) {
        s_tone_last_print_at = now;
        /* CH32V407 DAC1 has no digital output-read register - the
         * analog side has no digital readout. Read the current entry
         * in the waveform table (the value DMA is delivering to PA4
         * right now via the phase counter) instead. */
        const uint8_t  idx = (uint8_t)((s_phase >> 24) & 0xFFU);
        const uint16_t tbl_val = s_table[idx];
        printf ("Tone: %s @ %u Hz tick=%u scc.q=%u "
                "PA4=0x%03x idx=%u\r\n",
                Tone_WaveName ((ToneWaveform)s_waveform),
                (unsigned)Tone_GetFreq (),
                (unsigned)now,
                (unsigned)SCC_GetLevel (),
                (unsigned)(tbl_val & 0xFFFU),
                (unsigned)idx);
    }
}
