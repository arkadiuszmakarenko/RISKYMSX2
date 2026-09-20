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

/* MSX Z80-side register-window constants (Konami-with-SCC).
 *
 * Konami-with-SCC slot mapping (openMSX::RomKonamiSCC::writeMem + the
 * SCC chip memory layout emu2212 reimplements):
 *   0x9000..0x97FF  SCC enable (write 0x3F) AND SCC+ activation
 *                   (write 0x80 at 0x9000). emu2212 treats BOTH as
 *                   "offset 0" relative to base_adr=0x9000.
 *   0x9800..0x9FFE  SCC control registers (offset 0x800..0x8FE).
 *                   - 0x9800..0x987F (off 0x800..0x87F): wave table
 *                   - 0x9880..0x9889 (off 0x880..0x889): freq
 *                   - 0x988A..0x988E (off 0x88A..0x88E): volume
 *                   - 0x988F           (off 0x88F)         : ch-enable
 *                   - 0x98C0..0x98DF (off 0x8C0..0x8DF): mode/flags
 *                   0x9FFE/0x9FFF are the SCC mode-switch pair - the
 *                   cart writes (val & 0x20) to it to toggle
 *                   base_adr between 0x9000 and 0xB000. The cart
 *                   never touches 0x9800..0x9FFE for activation.
 *
 * Therefore SCC_I_WINDOW_BASE MUST be 0x9000. Setting it to 0x9800
 * (an earlier misread) made the activation byte write at 0x9000 fall
 * outside the emulator's base_adr window (`adr < base_adr` early-out
 * in emu2212::SCC_write line 393), `active` stayed 0 forever, and
 * the SCC stayed silent despite 11000+ writes landing in the cart
 * IRQ queue. See the openMSX::RomKonamiSCC reference impl.
 */
#define SCC_I_WINDOW_BASE   0x9000U   /* SCC enable + activation byte */
#define SCC_I_WINDOW_LAST   0x9FFEU   /* last  SCC register address  */

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

/* Snapshot the last sample value computed by the SCC pump (the 32-bit
 * word that DMA2_Channel3 transfers to DAC->RD12BDHR each tick).
 * Updated from TIM4_IRQHandler at ~44 kHz; reads from main loop see
 * either the in-flight or the most recently completed sample,
 * depending on IRQ racing. Used by the boot-time DAC probe to verify
 * the SCC sample pump is running - 0x800 means "mid-scale / silent",
 * anything else means the emulator produced a sample. */
uint32_t SCC_GetDualDacValue (void);

/* Drain every pending write through the SCC emulator NOW (in calling
 * context, NOT from the TIM4 IRQ). Used when the active mapper is
 * about to leave KONAMISCC - we want any cart writes already queued
 * by the Z80 to apply to the emulator's register state BEFORE the
 * TIM4 IRQ runs another batch with stale context. Returns the number
 * of writes drained (0 if the queue was empty). Must NOT be called
 * from IRQ context (it calls SCC_write which is not IRQ-safe). */
uint32_t SCC_FlushQueue (void);

#endif /* __SCC_H */