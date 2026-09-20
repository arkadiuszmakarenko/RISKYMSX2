/*
 * tone.h - raw-DAC1 tone generator for audio-path verification.
 *
 * Drives DAC1/PA4 directly with a generated waveform at a configurable
 * frequency, independent of the SCC emulator. Used to verify the
 * DAC1 -> PA4 chain works before debugging the SCC path.
 *
 * Takes over TIM4 + DMA2_Channel3 + DAC1 (the same peripherals the
 * SCC sample pump uses). Calling Tone_Init() stops the SCC pump;
 * calling Tone_Stop() restores it via SCC_Init's idempotent path.
 *
 * CLI exposure is recommended ("TONE" command) - see firmware/
 * User/cli.c for the parse.
 */

#ifndef __TONE_H
#define __TONE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TONE_WAVE_SINE     = 0,
    TONE_WAVE_TRIANGLE = 1,
    TONE_WAVE_SQUARE   = 2,
    TONE_WAVE_DC       = 3,
} ToneWaveform;

/* Waveform table size - exposed so the CLI can show the divisor. */
#define TONE_TABLE_SIZE   256U

/* Set to 1 to start the tone at boot; 0 to leave SCC audio running
 * by default (tone is opt-in via the CLI `TONE START` command).
 * Default OFF because the tone driver takes over TIM4 + DMA + DAC1
 * from the SCC sample pump - useful as a self-test but not wanted in
 * the normal game-playing flow. */
#ifndef TONE_DEFAULT_ON
#define TONE_DEFAULT_ON   0
#endif

/* Bring up the tone generator. Replaces SCC's TIM4/DMA/DAC pump.
 * sample_rate_hz: TIM4 rate (256 samples/cycle, audio freq = rate/256).
 *                200..96000 Hz. Clamped to fit.
 * wf:            initial waveform. */
int Tone_Init (uint32_t sample_rate_hz, ToneWaveform wf);

/* Tear down the tone generator; re-init SCC's sample pump via
 * SCC_Init (idempotent path). Pin PA4 is parked at mid-scale
 * (0x800) before handing back so we don't generate a step. */
void Tone_Stop (void);

/* Hot-swap the waveform without changing the sample rate. */
void Tone_SetWaveform (ToneWaveform wf);

/* Hot-swap the sample rate (and thus the audio frequency). Re-fills
 * the table. No-op if the tone is not currently running. */
void Tone_SetFreq (uint32_t sample_rate_hz);

ToneWaveform Tone_GetWaveform (void);
uint32_t     Tone_GetSampleRate (void);
uint32_t     Tone_GetFreq (void);
int          Tone_IsActive (void);

/* Forward from TIM4_IRQHandler (in scc.c) into the tone engine while
 * a tone is active. Cheap: ~30 cycles per call (phase add + print
 * throttle). */
void Tone_TIM4Tick (void);

const char *Tone_WaveName (ToneWaveform wf);

#ifdef __cplusplus
}
#endif

#endif /* __TONE_H */
