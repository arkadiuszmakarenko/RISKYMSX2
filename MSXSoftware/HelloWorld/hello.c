/*
 * hello.c - Hello World for MSX, built as a ROM cartridge with SDCC.
 *
 * Uses direct BIOS calls via inline assembly (MSX BIOS entry points):
 *   CHGMOD  (0x005F) - set screen mode
 *   CHPUT   (0x00A2) - print character
 *   CHGET   (0x009F) - wait for a key
 * Works on any MSX1 or higher. The cartridge auto-starts at power-on.
 */

#include <stdint.h>

#define SCREEN1_WIDTH 32          /* SCREEN 1 (32x24 text) fits "HELLO WORLD" nicely */
#define SCREEN0_WIDTH 40          /* SCREEN 0 (40x24 text) */

/* ------------------------------------------------------------------ */
/* Minimal BIOS wrappers using inline Z80 assembly                     */
/* ------------------------------------------------------------------ */

/* Set screen mode: 0 = SCREEN 0 (40x24), 1 = SCREEN 1 (32x24) */
void set_screen(uint8_t mode) __z88dk_fastcall
{
	mode; /* passed in L */
	__asm
		push	ix
		push	iy
		ex	af, af'
		ld	a, l
		ld	iy, #0xFCC0		; EXPTBL & 0xFF00: main-ROM slot
		ld	ix, #0x005F		; CHGMOD
		call	0x001C			; CALSLT
		ex	af, af'
		pop	iy
		pop	ix
	__endasm;
}

/* Print one character to the screen */
void putchar_msx(char c) __z88dk_fastcall
{
	c; /* passed in L */
	__asm
		push	ix
		push	iy
		ex	af, af'
		ld	a, l
		ld	iy, #0xFCC0
		ld	ix, #0x00A2		; CHPUT
		call	0x001C			; CALSLT
		ex	af, af'
		pop	iy
		pop	ix
	__endasm;
}

/* Wait for and return a key press (returned in L by CHGET) */
char wait_key(void) __naked
{
	__asm
		ld	iy, #0xFCC0
		ld	ix, #0x009F		; CHGET
		call	0x001C			; CALSLT
		ld	l, a			; CHGET returns keycode in A;
						; put it in L as C return value
		ret
	__endasm;
}

/* Print a C string */
void print(const char *s)
{
	while (*s)
		putchar_msx(*s++);
}

/* ------------------------------------------------------------------ */
/* main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
	set_screen(SCREEN1_WIDTH == 32 ? 1 : 0);

	print("HELLO WORLD!\r\n\r\n");
	print("FROM RISKYMSX2 CARTRIDGE\r\n\r\n");
	print("PRESS ANY KEY TO EXIT...");

	wait_key();

	set_screen(0);          /* back to SCREEN 0 for BASIC */

	return 0;               /* RET to BIOS, boot continues into BASIC */
}
