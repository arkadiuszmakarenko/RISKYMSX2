/*
 * romloader.c - RISKYMSX2 ROM LOADER for MSX (SDCC / z80 target).
 *
 * Runs ENTIRELY from MSX internal RAM (crt0 copies it there at boot)
 * and talks to the CH32V407 firmware over the cartridge bus using a
 * small mailbox protocol.  The user picks a .ROM file, the firmware
 * streams it into PSRAM, the user picks a mapper, then the firmware
 * resets the MSX - after which the firmware serves the loaded ROM
 * from PSRAM with the chosen mapper.  The loader itself is gone.
 *
 * ------------------------------------------------------------------ *
 * MAILBOX PROTOCOL (cart window 0x7FF0..0x7FFF, active while the
 * firmware's LOADER mapper is installed):
 *
 *   The loader executes from MSX RAM, so every cart access is a
 *   deliberate mailbox cycle.  The firmware's loader handler decodes
 *   the low bits of the address (0x7FF0 + off):
 *
 *     READ  off=0  - status byte:
 *                     bit7 = ready (firmware alive)
 *                     bit6 = command finished (result available)
 *                     bit5 = reserved
 *     READ  off=1  - pop one result byte (FIFO)
 *     READ  off>=2 - 0xFF (reserved)
 *     WRITE off=0  - push one command byte (cmd first, then args)
 *     WRITE off>=1 - reserved
 *
 *   Command byte stream (each byte is one write to off=0):
 *     CMD_DIR_OPEN   (0x00)  no args
 *         -> result: count-lo, count-hi (number of .ROM files)
 *     CMD_DIR_READ   (0x01)  no args
 *         -> result: 11 bytes, the i-th filename in 8.3 space-padded
 *            form ("NAME    ROM"), exactly as the firmware listed it.
 *            Call count times.
 *     CMD_DIR_CLOSE  (0x02)  no args
 *     CMD_LOAD_ROM   (0x03)  args: 11-byte filename (8.3 form)
 *         -> result: 4-byte length, big-endian (bytes actually copied
 *            into the PSRAM cart window at offset 0)
 *     CMD_SET_MAPPER (0x04)  args: mapper index (Cart_Mapper enum,
 *            1..10: ROM16k..KONAMISCC; 0=NONE is refused)
 *         -> result: 1 byte, 0 = OK, nonzero = refused
 *     CMD_RESET      (0x05)  no args
 *         Firmware asserts the MSX reset line.  No result; the MSX
 *         restarts and the BIOS re-reads the cart header - by then
 *         the firmware serves the loaded ROM with the new mapper.
 *
 *   The firmware side of this protocol lives in the LOADER mapper
 *   handler (Cart_EXTI0_Loader_Handler in cart.c, which services the
 *   mailbox range from SRAM and serves the loader ROM image for every
 *   other read) and the command engine (loader.c, drained from the
 *   firmware main loop).
 * ------------------------------------------------------------------ *
 *
 * UI interaction model (keyboard-layout independent - ANY key works):
 *   TAP  (press + release quickly)  = advance the cursor
 *   HOLD (press >= ~400 ms)          = choose the highlighted line
 * The SNSMAT PSG matrix scan needs no BIOS key ISR; crt0 keeps
 * interrupts disabled throughout (INIT is entered with IF=0).
 */

#include <stdint.h>

/* ---- mailbox ------------------------------------------------------ */
#define MBOX_BASE   0x7FF0U
#define MBOX_STATUS 0U   /* read: status          */
#define MBOX_DATA   1U   /* read: result pop     */
#define MBOX_CMD    0U   /* write: cmd/arg push  */

#define ST_READY    0x80U
#define ST_DONE    0x40U

#define CMD_DIR_OPEN    0x00U
#define CMD_DIR_READ    0x01U
#define CMD_DIR_CLOSE   0x02U
#define CMD_LOAD_ROM    0x03U
#define CMD_SET_MAPPER  0x04U
#define CMD_RESET       0x05U

/* mapper indices - MUST match Cart_Mapper in firmware/User/cart.h */
#define MAP_NONE        0U
#define MAP_ROM16K      1U
#define MAP_ROM32K      2U
#define MAP_ROM48K      3U
#define MAP_KONAMI      4U
#define MAP_KONAMINOSCC 5U
#define MAP_ASCII8K     6U
#define MAP_ASCII16K    7U
#define MAP_NEO8        8U
#define MAP_NEO16       9U
#define MAP_KONAMISCC   10U

/* ---- BIOS wrappers (CALSLT into the main ROM) --------------------- */

/* SNSMAT 0x0141: one keyboard-matrix row via the PSG.  row in A,
 * matrix byte back in A (bit=0 => key down).  No ISR dependency.
 * CALSLT returns the value in A; move it to L (SDCC's z80 return
 * register for 8-bit values). */
static uint8_t snsmat(uint8_t row) __z88dk_fastcall __naked
{
	row; /* in L */
	__asm
		push	ix
		push	iy
		ld	a, l
		ld	iy, #0xFCC0
		ld	ix, #0x0141		; SNSMAT
		call	0x001C			; CALSLT
		ld	l, a			; return value in L
		pop	iy
		pop	ix
		ret
	__endasm;
}

/* CHPUT 0x00A2: print one char (arg in L -> A). */
static void chput(char c) __z88dk_fastcall
{
	c; /* in L */
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

/* CHGMOD 0x005F: set screen mode (arg in L -> A). */
static void chgmod(uint8_t mode) __z88dk_fastcall
{
	mode; /* in L */
	__asm
		push	ix
		push	iy
		ex	af, af'
		ld	a, l
		ld	iy, #0xFCC0
		ld	ix, #0x005F		; CHGMOD
		call	0x001C			; CALSLT
		ex	af, af'
		pop	iy
		pop	ix
	__endasm;
}

/* ---- mailbox access ------------------------------------------------
 * The cart window is memory-mapped; SDCC must emit a real ld a,(nn)
 * on every access, so go through a volatile pointer. */
static volatile uint8_t *const mbox = (volatile uint8_t *)MBOX_BASE;

static uint8_t mbox_status(void)     { return mbox[MBOX_STATUS]; }
static uint8_t mbox_pop(void)        { return mbox[MBOX_DATA];   }
static void    mbox_push(uint8_t v)  { mbox[MBOX_CMD] = v;       }

/* Wait until the firmware reports the current command finished. */
static void mbox_wait_done(void)
{
	while ((mbox_status() & ST_DONE) == 0U) {
	}
}

/* Post a command + argument bytes (args may be NULL when argc==0),
 * then wait for completion. */
static void mbox_cmd(uint8_t cmd, const uint8_t *args, uint8_t argc)
{
	mbox_push(cmd);
	for (uint8_t i = 0; i < argc; i++)
		mbox_push(args[i]);
	mbox_wait_done();
}

/* ---- console helpers ---------------------------------------------- */

static void print(const char *s)
{
	while (*s)
		chput(*s++);
}

static void print_dec(uint16_t v)
{
	char buf[6];
	uint8_t i = 0;
	if (v == 0) {
		chput('0');
		return;
	}
	while (v) {
		buf[i++] = (char)('0' + (v % 10U));
		v /= 10U;
	}
	while (i)
		chput(buf[--i]);
}

static void cls(void)
{
	chput(0x0C);		/* CHPUT ctrl-L = clear screen */
}

/* ---- key input: SNSMAT scan, TAP / HOLD detection ------------------ */

/* Scan all 10 rows; return 1 if any key is down. */
static uint8_t any_key_down(void)
{
	for (uint8_t row = 0; row < 10; row++) {
		if (snsmat(row) != 0xFFU)
			return 1;
	}
	return 0;
}

/* Crude millisecond delay (SNSMAT read ~ 100 us; 10 rows ~ 1 ms). */
static void delay_ms(uint16_t ms)
{
	while (ms--)
		(void)any_key_down();
}

#define HOLD_MS 400U   /* press held this long = "choose" */

/* Wait for a key event: returns 0 for TAP, 1 for HOLD.
 *
 * While the key is held we keep polling the matrix (which also
 * provides the delay ticks); after HOLD_MS of continuous press we
 * return HOLD immediately (no need to wait for release).  For a TAP
 * we wait for full release, then debounce. */
static uint8_t wait_key_event(void)
{
	/* wait for a press */
	while (!any_key_down()) { }

	/* count how long it stays down */
	uint16_t held = 0;
	while (any_key_down()) {
		held++;
		if (held >= HOLD_MS)
			return 1U;		/* HOLD */
	}
	/* released early enough - debounce release, report TAP */
	delay_ms(30);
	return 0U;
}

/* ---- file list ------------------------------------------------------ */

#define MAX_FILES 128U

/* 8.3 names, 11 bytes each ("NAME    ROM"), NUL-terminated for print */
static char s_files[MAX_FILES][12];
static uint16_t s_count;

static void fetch_file_list(void)
{
	mbox_cmd(CMD_DIR_OPEN, 0, 0);
	uint8_t cnt_lo = mbox_pop();
	uint8_t cnt_hi = mbox_pop();
	s_count = (uint16_t)cnt_lo | ((uint16_t)cnt_hi << 8);
	if (s_count > MAX_FILES)
		s_count = MAX_FILES;

	for (uint16_t i = 0; i < s_count; i++) {
		mbox_cmd(CMD_DIR_READ, 0, 0);
		for (uint8_t j = 0; j < 11; j++)
			s_files[i][j] = (char)mbox_pop();
		s_files[i][11] = '\0';
	}

	mbox_cmd(CMD_DIR_CLOSE, 0, 0);
}

/* ---- selector: generic single-choice list ---------------------------
 * Draws up to 10 visible lines + a one-line hint.  TAP moves, HOLD
 * chooses.  Returns the chosen index. */
static uint16_t choose(const char *title, const char *const *names,
                       uint16_t count, uint16_t start)
{
	uint16_t sel = start < count ? start : 0;

	for (;;) {
		cls();
		print(title);
		print("\r\n\r\n");

		/* 10-line scrolling window centred on sel */
		uint16_t top = sel >= 5U ? (uint16_t)(sel - 5U) : 0U;
		uint16_t bot = top + 10U;
		if (bot > count)
			bot = count;
		for (uint16_t i = top; i < bot; i++) {
			print(i == sel ? ">" : " ");
			print(" ");
			print(names[i]);
			print("\r\n");
		}
		print("\r\nTAP=NEXT  HOLD=SELECT\r\n");

		if (wait_key_event() == 1U)
			return sel;		/* HOLD on the highlighted line */
		sel++;
		if (sel >= count)
			sel = 0;
	}
}

/* ---- main ----------------------------------------------------------- */

int main(void)
{
	chgmod(0);		/* SCREEN 0, 40 columns */

	cls();
	print("RISKYMSX2 ROM LOADER\r\n\r\n");
	print("Reading USB directory...\r\n");

	fetch_file_list();

	if (s_count == 0U) {
		cls();
		print("NO .ROM FILES ON USB STICK\r\n\r\n");
		print("Plug a stick with .ROM files,\r\n");
		print("power-cycle the MSX.\r\n");
		for (;;)
			(void)wait_key_event();
	}

	/* ---- screen 1: pick the file ---- */
	static const char *file_ptrs[MAX_FILES];
	for (uint16_t i = 0; i < s_count; i++)
		file_ptrs[i] = s_files[i];
	uint16_t sel = choose("SELECT ROM FILE:", file_ptrs, s_count, 0);

	/* ---- load it into PSRAM ---- */
	cls();
	print("LOADING ");
	print(s_files[sel]);
	print("...\r\n");
	{
		uint8_t args[11];
		for (uint8_t i = 0; i < 11; i++)
			args[i] = (uint8_t)s_files[sel][i];
		mbox_cmd(CMD_LOAD_ROM, args, 11);
		/* result: 4-byte length, big-endian */
		uint8_t len[4];
		for (uint8_t i = 0; i < 4; i++)
			len[i] = mbox_pop();
		uint32_t total = ((uint32_t)len[0] << 24)
		               | ((uint32_t)len[1] << 16)
		               | ((uint32_t)len[2] << 8)
		               |  (uint32_t)len[3];
		print("LOADED ");
		print_dec((uint16_t)((total + 1023U) / 1024U));
		print(" KiB\r\n\r\n");
	}

	/* ---- screen 2: pick the mapper ---- */
	static const char *const mapper_names[] = {
		"ROM16K",
		"ROM32K",
		"ROM48K",
		"KONAMI",
		"KONAMINOSCC",
		"ASCII8K",
		"ASCII16K",
		"NEO8",
		"NEO16",
		"KONAMISCC",
	};
	uint16_t msel = 1U;	/* default suggestion: ROM32K */
	uint8_t chosen = (uint8_t)choose("SELECT MAPPER:",
	                                 mapper_names, 10, msel);

	/* ---- apply the mapper ---- */
	cls();
	print("MAPPER: ");
	print(mapper_names[chosen]);
	print("\r\n");
	{
		uint8_t args[1];
		args[0] = (uint8_t)(chosen + 1U);	/* Cart_Mapper index */
		mbox_cmd(CMD_SET_MAPPER, args, 1);
		uint8_t rc = mbox_pop();
		if (rc != 0U) {
			print("REFUSED! (PSRAM not ready?)\r\n");
			for (;;)
				(void)wait_key_event();
		}
	}

	/* ---- reset the MSX; firmware serves the new ROM on boot ---- */
	print("RESET...");
	mbox_push(CMD_RESET);

	/* If the firmware is not there, the write above silently no-ops;
	 * park here (interrupts are off, so HALT would need an ei). */
	for (;;) { }
}