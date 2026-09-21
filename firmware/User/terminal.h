/********************************** (C) COPYRIGHT *******************************
 * File Name          : terminal.h
 * Description        : Minimal MSX terminal menu for the RISKYMSX2.
 *
 *                      Cart-loaded terminal UI (port of v303 MSXTerminal.c,
 *                      simplified to a single file-list + ROM-load +
 *                      mapper-select workflow).
 *
 *                      The MSX-side program (MSXSoftware/RomLoader/asm/
 *                      terminal.asm) drives the screen and forwards
 *                      keystrokes on the 0x7FFD mailbox byte. The firmware
 *                      hosts the menu logic via the FIFO at 0x7FFF (read
 *                      by the MSX) and the control byte at 0x7FFE.
 *
 *                      Call Terminal_Service() from the main loop only
 *                      while the TERMINAL mapper is active (or always,
 *                      if you want the menu available in parallel with
 *                      the FLASH loader). The mailbox is IRQ-safe: EXTI0
 *                      fills it, main-loop drains it.
 *********************************************************************************/

#ifndef __TERMINAL_H
#define __TERMINAL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Terminal mailbox (lives in cart.c so the EXTI0 handler can touch it
 * directly - kept in zero-wait-state SRAM via the existing volatile
 * qualifiers). */
typedef struct {
    /* keyboard FIFO: MSX pushes a byte at 0x7FFD; terminal.c drains. */
    volatile uint8_t  kbd_buf[16];
    volatile uint8_t  kbd_head;
    volatile uint8_t  kbd_tail;
    volatile uint8_t  kbd_n;
    /* output FIFO: terminal.c pushes; MSX pops at 0x7FFF. Sized to
     * hold an entire file-list refresh plus title, screen-clear,
     * sprite-position and a comfortable safety margin (~800 bytes
     * for a 20-file menu), so the menu pushes in one Terminal_Service
     * pass without overflowing and the MSX never sees a partial
     * refresh.
     *
     * out_head / out_tail MUST be uint16_t: the ring wraps at 2048,
     * and a uint8_t index would alias after 256 bytes - which was
     * exactly the "only 6 files render, second page repeats one
     * file" bug. */
    volatile uint8_t  out_buf[2048];
    volatile uint16_t out_head;
    volatile uint16_t out_tail;
    volatile uint16_t out_n;
    /* control byte: latched on every 0x7FFE write. terminal.c reads
     * and clears. Set to 0xFF to mean "no control yet" (since valid
     * control values are 0x00/0x03/0x04 - all small, so 0xFF never
     * collides). */
    volatile uint8_t  control;
} TerminalMailbox;

extern TerminalMailbox g_term_mbox;

/* Reset the terminal mailbox (call when installing the TERMINAL mapper
 * to drop any stale FIFO contents from a previous session). */
void Terminal_Reset (void);

/* Main-loop service: pumps menu characters into g_term_mbox.out_buf,
 * pops keystrokes from g_term_mbox.kbd_buf, and acts on the control
 * byte. Never blocks longer than a single FATFS directory scan; call
 * every loop pass while the TERMINAL mapper is active. */
void Terminal_Service (void);

/* Boot-path boot of the user cart image. Called when the user picks a
 * ROM file and a mapper (or when the user held GRPH at boot). Applies
 * the chosen mapper and triggers the MSX-side soft reset by writing
 * 0x03 into the output FIFO (the MSX terminal jumps to the UGLY_PATCH
 * path on that byte). */
void Terminal_BootCart (uint8_t mapper_idx, const char *filename);

#ifdef __cplusplus
}
#endif

#endif /* __TERMINAL_H */