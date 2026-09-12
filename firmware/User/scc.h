/*
 * SCC sound-chip emulation glue for RISKYMSX2 (CH32V407V).
 *
 * Ports the legacy v303Firmware scc.c glue to the new board:
 *
 *   - The emulator core (emu2212.c/h, Mitsutaka Okazaki's SCC emulator)
 *     is used VERBATIM from v303Firmware - byte-identical copy.
 *
 *   - Cart writes are latched by the EXTI0 cart handler (KONAMISCC
 *     mapper, cart.c) into a lock-free SPSC ring buffer; this module's
 *     timer IRQ drains the queue into SCC_write() and renders the next
 *     sample, mirroring the legacy DMA2_Channel3_IRQHandler design.
 *
 *   - Clock-rate change vs v303: the legacy chip ran TIM4 off a
 *     fixed 144 MHz timer clock with a hardcoded period of 3368
 *     (=> ~42.7 kHz). Here the period is derived at init time from
 *     HCLKClock (SystemCoreClockUpdate() must have run), so the same
 *     ~44.1 kHz-ish sample rate is kept across clock profiles.
 *
 *   - DAC: CH32V407 DAC1 output pin is PA4 = SCC_OUT on this board
 *     (the v303 board wired SCC_OUT elsewhere; GPIO config added here
 *     accordingly). DMA2_Channel3 is hardwired to DAC1 on the CH32V4x7
 *     DMA request map (same channel as the v303 chip used).
 *
 * Public API:
 *   SCC_Init()        bring up emulator + DAC + DMA + TIM (idempotent)
 *   SCC_QueueWrite()  IRQ-safe: push one (addr<<16 | data) cart write
 *   SCC_GetLevel()    current fill level of the queue (diagnostics)
 *   SCC_DeInit()      stop the pump (mapper switch away from KONAMISCC)
 */

#ifndef __SCC_H
#define __SCC_H

#include <stdint.h>

/* MSX Z80-side register-window constants (SCC-I / Konami-with-SCC). */
#define SCC_I_WINDOW_BASE   0x9800U   /* first SCC-I register address  */
#define SCC_I_WINDOW_LAST   0x98FFU   /* last  SCC-I register address  */

/* Bring up the SCC emulator, DAC1/PA4, DMA2 ch3 and the sample timer.
 * Idempotent: a second call while running is a no-op (returns 0).
 * Returns 0 on success, -1 on failure (clock not initialised etc.). */
int  SCC_Init (void);

/* Stop the sample pump + DMA (used when switching mapper away).
 * The emulator keeps its register state; re-init restarts cleanly. */
void SCC_DeInit (void);

/* Queue one cartridge write for the emulator. `word` is the packed
 * ((address << 16) | data) form produced by the cart write handler.
 * IRQ-safe (single-producer/single-consumer). Returns 0 on success,
 * -1 if the queue is full (write dropped - same as legacy overflow). */
int  SCC_QueueWrite (uint32_t word);

/* Number of queued-but-not-yet-applied writes (CLI diagnostics). */
uint32_t SCC_GetLevel (void);

#endif /* __SCC_H */