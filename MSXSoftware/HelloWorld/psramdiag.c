/*
 * psramdiag.c - PSRAM / cartridge-bus measurement ROM v2 (RISKYMSX2).
 *
 * Purpose: give the HP 1660C logic analyzer REPRODUCIBLE, per-phase
 * address-tagged bus patterns so the PSRAM serve path can be measured
 * and diffed against the SRAM baseline. Run the same cart image with
 * the firmware cart source set to SRAM and to PSRAM, capture each
 * phase on the analyzer, and compare cycle-for-cycle.
 *
 *   PH1  4000x fixed-address data read @ 0x5000 (M1=0 cycles, all
 *        identical). LA: trigger SLTSL fall + MREQ=0 + RD=0 + M1=0
 *        with A[15:8]=0x50, A[7:0]=0x00. Measure SLTSL-fall ->
 *        D0-D7 valid and the served-byte window.
 *   PH2  LDIR 256 bytes 0x5000 -> 0xD000. Tightest legal back-to-back
 *        cart read sequence (one cart read + one cart M1 fetch per
 *        LDIR iteration). Exposes read-to-read (tRC) recovery problems
 *        in the PSRAM controller.
 *   PH3  2048x isolated reads @ 0x5040 through a 4-byte stub running
 *        entirely in RAM (ld a,(hl) / djnz / ret). The ONLY cart bus
 *        activity is the data read itself, ~15 T apart - the cleanest
 *        possible view of a single serve. LA qualifier: A=0x5040.
 *   PH4  Walks 0x5000..0x50FF and verifies the diagstub walking
 *        pattern (byte[i] = i ^ 0xA5). Catches PSRAM data corruption
 *        independent of timing. Prints OK or the first mismatch.
 *   PH5  512x alternating main-ROM BIOS calls (GTSTCK via CALSLT) and
 *        cart reads @ 0x5080. Stresses slot turnaround: SLTSL goes
 *        high for the BIOS call and low for the read, twice per
 *        iteration. LA qualifier: A=0x5080.
 *   L    After all phases: parks in PH1 forever (leave the LA
 *        triggering on it).
 *
 * Input model: ANY key (press + release) advances to the next
 * phase; the ROM blocks indefinitely at each step so the operator
 * can arm the 1660C first. Keys are read DIRECTLY from the keyboard
 * matrix via SNSMAT (BIOS 0x0141), which scans the PSG hardware and
 * does NOT depend on the BIOS keyboard interrupt handler -
 * deliberate robustness for the emulated-cart bring-up, where
 * CHGET would hang if the ISR was never entered (crt0 keeps DI).
 * (CHGET also caused a real v2 bug: SDCC __naked emits no ret, so
 * the v2 menu fell through into gtstck() and appeared frozen.)
 *
 * Build: make psramdiag.bin (see Makefile; requires sdcc).
 * Embed: make embed  (regenerates firmware/User/hello_rom.c via
 *                     tools/rom2c.py, so the ROM is served from BOTH
 *                     the SRAM window and the PSRAM mirror at boot).
 */

#include <stdint.h>

/* Fixed cart-bus addresses used by the phases (must match diagstub.s).
 * All fall inside the 256-byte walking pattern at 0x5000. */
#define PATTERN_ADDR   0x5000U   /* walking pattern (256 bytes)   */
#define PH1_ADDR       0x5000U   /* PH1 fixed-read address       */
#define PH3_ADDR       0x5040U   /* PH3 isolated-read address    */
#define PH5_ADDR       0x5080U   /* PH5 turnaround read address  */
#define PATTERN_SIZE   256U
#define LDIR_DST       0xD000U   /* PH2 destination (MSX RAM)    */
#define STUB_RAM       0xD100U   /* PH3 stub target (MSX RAM)    */

/* Iteration counts (sized for the 1660C capture depth). */
#define PH1_ITER       4000U
#define PH3_PASSES     8U        /* x256 reads per pass = 2048    */
#define PH5_ITER       512U

/* ------------------------------------------------------------------ */
/* BIOS wrappers - same CALSLT shape as the proven hello.c code.       */
/* ------------------------------------------------------------------ */

/* CHGMOD 0x005F: set screen mode (arg in L -> A). */
static void chgmod(uint8_t mode) __z88dk_fastcall
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

/* CHPUT 0x00A2: print one character (arg in L -> A). */
static void chput(char c) __z88dk_fastcall
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

/* SNSMAT 0x0141: reads one keyboard-matrix row DIRECTLY through the
 * PSG (row in A, matrix byte back in A; bit=0 => key down). Unlike
 * CHGET/CHSNS this does NOT depend on the BIOS keyboard interrupt
 * handler, so it works with interrupts disabled - deliberately:
 * the BIOS enters cart INIT with IF=0 and we keep it that way (see
 * crt0.s), so no ISR ever runs while phases are being measured.
 *
 * NOTE: functions marked __naked get NO compiler-generated ret - the
 * explicit ret below is mandatory (its absence is what made the v2
 * menu fall through into the next function and appear dead).
 * Interrupts stay DI throughout (no ei on the exit paths). */
static uint8_t key_scan_all(void) __naked
{
	__asm
		push	ix
		push	iy
		di			; no ISR interference while we
					; hold the PSG address latch
		ld	b, #10			; rows 0..9
		ld	c, #0			; current row
scan_row:
		push	bc
		ld	a, c			; row number
		ld	iy, #0xFCC0
		ld	ix, #0x0141		; SNSMAT
		call	0x001C			; CALSLT -> A = row matrix
		pop	bc
		inc	a			; 0xFF (row idle) -> 0 sets Z
		jr	z, row_idle
		pop	iy
		pop	ix
		ld	l, #1			; some key is down
		ret
row_idle:
		inc	c
		djnz	scan_row
		pop	iy
		pop	ix
		ld	l, #0			; nothing down
		ret
	__endasm;
}

/* Busy-wait ~1 ms per unit (HL), interrupt-independent (no halt).
 * Naked + explicit ret: the asm clobbers bc, and SDCC cannot see
 * register usage inside a plain __asm block, so the function must
 * not share a frame with compiler-managed locals.
 * Loop body: dec bc / ld a,b / or c / jr nz = 26 T ~ 7.27 us @
 * 3.58 MHz; 138 iterations ~ 1 ms. */
static void delay_ms(uint16_t ms) __z88dk_fastcall __naked
{
	__asm
dly_outer:
		ld	bc, #138
dly_inner:
		dec	bc
		ld	a, b
		or	c
		jr	nz, dly_inner
		dec	hl
		ld	a, h
		or	l
		jr	nz, dly_outer
		ret
	__endasm;
}

/* Wait for ANY key, debounce, then wait for release. Returns only
 * when the key is released - so a single press = exactly one phase
 * advance (hold-to-repeat is impossible). Waits FOREVER: the whole
 * point is to give the operator time to arm the 1660C before each
 * phase; there is no auto-advance. */
static void wait_key(void)
{
	for (;;) {
		if (key_scan_all()) {
			delay_ms(30);		/* debounce */
			while (key_scan_all())	/* wait for release */
				delay_ms(20);
			return;
		}
		delay_ms(20);
	}
}

/* GTSTCK 0x00D5: read joystick state (main-ROM slot cycle, no side
 * effects). PH5 uses it as the "BIOS call between cart reads". */
static void gtstck(uint8_t stick) __z88dk_fastcall
{
	stick; /* passed in L */
	__asm
		push	ix
		push	iy
		ex	af, af'
		ld	a, l
		ld	iy, #0xFCC0
		ld	ix, #0x00D5		; GTSTCK
		call	0x001C			; CALSLT
		ex	af, af'
		pop	iy
		pop	ix
	__endasm;
}

static void print(const char *s)
{
	while (*s)
		chput(*s++);
}

static void print_hex(uint8_t b)
{
	static const char hex[] = "0123456789ABCDEF";
	chput(hex[(b >> 4) & 0x0F]);
	chput(hex[b & 0x0F]);
}

/* ------------------------------------------------------------------ */
/* PH1: fixed-address data-read loop                                   */
/* ------------------------------------------------------------------ */

/* 4000 identical MREQ+RD cycles reading 0x5000 (no M1). The loop
 * itself executes from the cart, so its opcode fetches are cart M1
 * cycles; qualify the LA trigger with M1=0 to see only the data
 * reads - every one of them is the same cycle, so traces can be
 * averaged and diffed between SRAM and PSRAM sources. */
static void phase1_fixed_reads(void) __naked
{
	__asm
		ld	bc, #0x0FA0		; PH1_ITER = 4000
		ld	hl, #0x5000		; PH1_ADDR
ph1_loop:
		ld	a, (hl)			; the cart data read
		dec	bc
		ld	a, b
		or	c
		jp	nz, ph1_loop
		ret
	__endasm;
}

/* ------------------------------------------------------------------ */
/* PH2: LDIR block copy from the cart                                  */
/* ------------------------------------------------------------------ */

/* LDIR re-fetches itself each iteration (PC does not advance), giving
 * the tightest legal sequence of cart cycles: M1 fetch @ LDIR opcode +
 * data read @ (HL) per iteration, ~21 T apart. Any missed serve shows
 * up immediately; PSRAM tRC recovery problems appear as WAIT or wrong
 * data. Source = walking pattern @ 0x5000, dest = MSX RAM @ 0xD000. */
static void phase2_ldir_copy(void) __naked
{
	__asm
		ld	hl, #0x5000		; src: walking pattern in cart
		ld	de, #0xD000		; dst: MSX RAM
		ld	bc, #0x0100		; PATTERN_SIZE
		ldir
		ret
	__endasm;
}

/* ------------------------------------------------------------------ */
/* PH3: isolated cart reads via a RAM-resident stub                    */
/* ------------------------------------------------------------------ */

/* Builds a 4-byte stub in RAM at 0xD100:
 *     0xD100: 7E        ld a,(hl)    ; THE cart read
 *     0xD101: 10 FD     djnz 0xD100 ; loop 256x
 *     0xD103: C9        ret
 * and calls it 8 times with HL = 0x5040. The inner loop runs entirely
 * in RAM, so the ONLY cart bus cycles are the isolated data reads,
 * one every ~20 T (ld a,(hl) = 7 T + djnz = 13 T). This is the
 * cleanest measurement of the serve: SLTSL fall -> D0-D7 valid ->
 * SLTSL rise -> bus released. */
static void phase3_isolated_reads(void) __naked
{
	__asm
		; build the stub at 0xD100 (ld a,(hl) / djnz -3 / ret)
		ld	hl, #0xD100
		ld	(hl), #0x7E
		inc	hl
		ld	(hl), #0x10
		inc	hl
		ld	(hl), #0xFD
		inc	hl
		ld	(hl), #0xC9
		; 8 passes x 256 isolated reads @ 0x5040
		ld	c, #0x08		; PH3_PASSES
ph3_outer:
		ld	b, #0x00		; djnz counts 256 reads
		ld	hl, #0x5040		; PH3_ADDR
		call	0xD100			; run the RAM stub
		dec	c
		jr	nz, ph3_outer
		ret
	__endasm;
}

/* ------------------------------------------------------------------ */
/* PH4: walking-pattern integrity check                                */
/* ------------------------------------------------------------------ */

/* Reads 0x5000..0x50FF and verifies byte[i] == i ^ 0xA5 (the content
 * placed by diagstub.s). Catches data corruption on the PSRAM source
 * even when every timing measurement looks fine. */
static void phase4_integrity(void)
{
	uint16_t off;
	uint8_t  bad = 0;

	print("PH4 integrity 0x5000-0x50FF ... ");
	for (off = 0; off < PATTERN_SIZE; off++) {
		uint8_t exp = (uint8_t)(off ^ 0xA5U);
		uint8_t got = *(volatile uint8_t *)(uint16_t)(PATTERN_ADDR + off);
		if (got != exp) {
			bad = 1;
			print("\r\nBAD@");
			print_hex((uint8_t)(off >> 8));
			print_hex((uint8_t)off);
			print(" got=");
			print_hex(got);
			print(" exp=");
			print_hex(exp);
			print("\r\n");
		}
	}
	if (!bad)
		print("OK\r\n");
}

/* ------------------------------------------------------------------ */
/* PH5: BIOS <-> cart slot-turnaround torture                          */
/* ------------------------------------------------------------------ */

/* Each iteration does a main-ROM BIOS call (GTSTCK via CALSLT: SLTSL
 * high) followed by a cart read @ 0x5080 (SLTSL low). If the cart
 * handler misses or mis-serves cycles around slot switches - EXTI0
 * pending from a late previous cycle, PSRAM controller busy, etc. -
 * the MSX hangs or reads garbage here. LA: trigger on SLTSL fall,
 * qualifier A=0x5080, capture several cycles of alternation. */
static void phase5_turnaround(void)
{
	uint16_t i;
	uint8_t  dummy;

	print("PH5 BIOS/rd @0x5080 x512 ... ");
	for (i = 0; i < PH5_ITER; i++) {
		gtstck(2);	/* main-ROM slot cycle (SLTSL high)   */
		dummy = *(volatile uint8_t *)(uint16_t)PH5_ADDR;
		(void)dummy;	/* cart cycle (SLTSL low)             */
	}
	print("done\r\n");
}

/* ------------------------------------------------------------------ */
/* menu                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
	uint8_t phase;

	chgmod(0);	/* SCREEN 0, 40 columns */

	print("\r\nPSRAMDIAG v2 - RISKYMSX2\r\n\r\n");
	print("ANY key = run next phase.\r\n");
	print("(interrupt-free key scan via SNSMAT,\r\n");
	print(" works even if the BIOS keyboard ISR\r\n");
	print(" never runs - deliberately robust\r\n");
	print(" on the emulated-cart bring-up)\r\n\r\n");
	print("PH1 fixed rd 0x5000 x4000\r\n");
	print("PH2 LDIR 0x5000->0xD000\r\n");
	print("PH3 isolated rd 0x5040 x2048\r\n");
	print("PH4 pattern integrity 0x5000-0x50FF\r\n");
	print("PH5 BIOS+rd 0x5080 x512\r\n\r\n");
	print("Set SRC SRAM/PSRAM on UART, reset,\r\n");
	print("arm the 1660C, then press a key.\r\n\r\n");

	for (phase = 1; ; phase++) {
		wait_key();	/* blocks until any key is pressed+released */

		switch (phase) {
		case 1:
			print("PH1 fixed rd @0x5000 x4000\r\n");
			phase1_fixed_reads();
			print("PH1 done\r\n");
			break;
		case 2:
			print("PH2 LDIR 0x5000->0xD000\r\n");
			phase2_ldir_copy();
			print("PH2 done\r\n");
			break;
		case 3:
			print("PH3 isolated rd @0x5040\r\n");
			phase3_isolated_reads();
			print("PH3 done\r\n");
			break;
		case 4:
			phase4_integrity();
			break;
		case 5:
			phase5_turnaround();
			break;
		default:
			/* all phases done: park in PH1 forever so the LA can
			 * keep triggering; power-cycle to restart. */
			print("\r\nALL DONE - PARKED in PH1 loop\r\n");
			print("(LA: trigger on A=0x5000, M1=0)\r\n\r\n");
			for (;;) {
				phase1_fixed_reads();
			}
			/* not reached */
			break;
		}
	}
	/* not reached */
	return 0;
}