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
 *     CMD_SOFTRESET  (0x07)  no args
 *         Software-only reboot. The firmware swaps the mapper WITHOUT
 *         driving ~RESET (read-only on most MSX2+ machines; driving
 *         it externally can damage the mainboard). The MSX then
 *         executes a small Z80 slingshot from MSX RAM at 0xF000
 *         (installed by soft_reset_install() at boot). The slingshot
 *         delays ~120 ms before `jp 0x0000` so the firmware's
 *         Cart_SetMapper_Safe completes BEFORE BIOS INIT re-probes
 *         the cart bus.
 *
 *     0x05 was formerly CMD_RESET (cart-driven ~RESET pulse). That
 *     command is GONE because driving ~RESET externally can damage
 *     some MSX designs. The slot is intentionally left empty rather
 *     than reused, so old firmware images on a stick don't silently
 *     invoke a half-defined command.
 *
 *   The firmware side of this protocol lives in the LOADER mapper
 *   handler (Cart_EXTI0_Loader_Handler in cart.c, which services the
 *   mailbox range from SRAM and serves the loader ROM image for every
 *   other read) and the command engine (loader.c, drained from the
 *   firmware main loop).
 * ------------------------------------------------------------------ *
 *
 * UI interaction model:
 *   UP / DOWN arrows = move the cursor (with wrap-around)
 *   SPACE            = choose the highlighted line
 *   Any other key    = ignored
 *
 * crt0 keeps interrupts disabled throughout (INIT is entered with
 * IF=0), so the BIOS keyboard ISR cannot run. We poll the PSG
 * keyboard matrix directly via SNSMAT 0x0141 - that does not need
 * an ISR.
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
/* 0x05 was CMD_RESET (cart-driven ~RESET pulse). REMOVED - driving
 * ~RESET externally can damage some MSX designs. Use CMD_SOFTRESET. */
#define CMD_BOOT_NEXTOR 0x06U   /* reserved (NEXTOR_PLAN §D2b) */
#define CMD_SOFTRESET   0x07U   /* RAM-resident slingshot reboot.
                                  * Use this when the MSX's ~RESET is
                                  * read-only on the cart edge: the
                                  * firmware swaps the mapper without
                                  * driving ~RESET, waits ~50 ms for
                                  * the cart bus / PSRAM to settle,
                                  * then signals DONE. The MSX-side
                                  * slingshot (at MSX RAM 0xE000)
                                  * then reads the cart header via RDSLT
                                  * and invokes the cart's INIT via
                                  * CALSLT - bypasses the BIOS cold-boot
                                  * probe entirely (which on most MSX2+
                                  * BIOSes does NOT re-probe the cart
                                  * even with WBOOT=0). */

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
	chput(0x0C);		/* CHPUT ctrl-L = clear screen + home cursor */
}

/* ---- key input: SNSMAT scan, arrow / space navigation --------------
 *
 * crt0 keeps interrupts disabled (the BIOS keyboard ISR does not get
 * along with the cart EXTI0 emulation, and INIT is entered with
 * IF=0 anyway), so we poll the PSG keyboard matrix directly via
 * SNSMAT 0x0141.  No ISR dependency.
 *
 * MSX keyboard matrix (international layout), row 8 = cursor keys:
 *
 *     bit 7 = RIGHT arrow
 *     bit 6 = DOWN  arrow
 *     bit 5 = UP    arrow
 *     bit 4 = LEFT  arrow
 *     bit 0 = SPACE
 *
 * (Row 6 is F1/F2/F3/CODE/CAPS/GRAPH/CTRL/SHIFT - NOT the arrows.
 * Row 7 is RET/SELECT/BS/STOP/TAB/ESC/F5/F4.)
 */

enum {
    KEY_NONE   = 0,
    KEY_DOWN   = 1,
    KEY_UP     = 2,
    KEY_SELECT = 3   /* SPACE */
};

/* Bit masks for row 8 (active-low: bit=0 => key down). */
#define R8_RIGHT  0x80U
#define R8_DOWN   0x40U
#define R8_UP     0x20U
#define R8_LEFT   0x10U
#define R8_SPACE  0x01U

/* Crude delay - SNSMAT read takes ~100 us per row; 10 iterations
 * of snsmat(0) is roughly 1 ms. */
static void delay_ms(uint16_t ms)
{
    while (ms--)
        (void)snsmat(0);
}

/* Wait for a DOWN / UP / SPACE press on row 8. Returns KEY_DOWN,
 * KEY_UP, KEY_SELECT, or KEY_NONE for any other key. */
static uint8_t wait_arrow(void)
{
    /* Wait for the first press on row 8. */
    while ((snsmat(8) & (R8_DOWN | R8_UP | R8_SPACE)) ==
           (R8_DOWN | R8_UP | R8_SPACE)) { }

    /* 10 ms debounce. */
    delay_ms(10);
    uint8_t row = snsmat(8);
    if ((row & (R8_DOWN | R8_UP | R8_SPACE)) ==
        (R8_DOWN | R8_UP | R8_SPACE)) {
        return KEY_NONE;
    }

    uint8_t key = KEY_NONE;
    if ((row & R8_SPACE) == 0U) key = KEY_SELECT;
    else if ((row & R8_DOWN) == 0U && (row & R8_UP) == 0U) key = KEY_DOWN;
    else if ((row & R8_DOWN) == 0U) key = KEY_DOWN;
    else if ((row & R8_UP)   == 0U) key = KEY_UP;

    /* Wait for release. */
    while ((snsmat(8) & (R8_DOWN | R8_UP | R8_SPACE)) !=
           (R8_DOWN | R8_UP | R8_SPACE)) { }
    delay_ms(10);
    return key;
}

/* Wait until any key on any row is down. */
static void wait_any_key(void)
{
    while (1) {
        for (uint8_t row = 0; row < 10; row++) {
            if (snsmat(row) != 0xFFU) return;
        }
    }
}

/* ---- file list ------------------------------------------------------ */

#define MAX_FILES 128U

#define LOADER_NAME_LEN 12
static char s_files[MAX_FILES][LOADER_NAME_LEN + 1];
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
		for (uint8_t j = 0; j < LOADER_NAME_LEN; j++)
			s_files[i][j] = (char)mbox_pop();
		s_files[i][LOADER_NAME_LEN] = '\0';
	}

	mbox_cmd(CMD_DIR_CLOSE, 0, 0);
}

/* ---- selector: generic single-choice list ---------------------------
 * DOWN / UP arrows move the cursor (with wrap-around). SPACE selects.
 * Full cls() + redraw per keypress. Simple and reliable. */
static uint16_t choose(const char *title, const char *const *names,
                       uint16_t count, uint16_t start)
{
    if (count == 0U) return 0U;
    uint16_t sel = start < count ? start : 0;

    for (;;) {
        cls();
        print(title);
        print("\r\n\r\n");

        /* 10-line scrolling window. Keep the window FULL (10 lines)
         * whenever possible: clamp top so top+10 <= count, rather
         * than letting bot get capped and shrinking the window.
         * The old code did `top = sel-5; bot = top+10; if (bot >
         * count) bot = count;` which for count=10, sel=6..9
         * produced top=1..4, bot=10 -> only 6..9 entries visible
         * (the list visibly shrank while navigating down). */
        uint16_t top, bot;
        if (count <= 10U) {
            top = 0U;
            bot = count;
        } else if (sel < 5U) {
            top = 0U;
            bot = 10U;
        } else if (sel >= (uint16_t)(count - 5U)) {
            top = (uint16_t)(count - 10U);
            bot = count;
        } else {
            top = (uint16_t)(sel - 5U);
            bot = (uint16_t)(sel + 5U);
        }
        for (uint16_t i = top; i < bot; i++) {
            print(i == sel ? "> " : "  ");
            print(names[i]);
            print("\r\n");
        }
        print("\r\nUP/DOWN=navigate  SPACE=select\r\n");

        uint8_t k;
        do {
            k = wait_arrow();
        } while (k == KEY_NONE);

        if (k == KEY_SELECT) {
            return sel;
        }

        if (k == KEY_DOWN) {
            sel++;
            if (sel >= count) sel = 0U;
        } else if (k == KEY_UP) {
            if (sel == 0U) sel = (uint16_t)(count - 1U);
            else sel--;
        }
    }
}

/* ---- soft reset slingshot --------------------------------------------
 *
 * CMD_SOFTRESET reboots the MSX WITHOUT driving ~RESET (necessary on
 * MSX machines whose cart-edge ~RESET is read-only). The firmware
 * still swaps the mapper; we still need to let that swap finish
 * before BIOS INIT re-probes the cart. We can't just `jp 0x0000`
 * from the loader because at that point:
 *   1. The mapper has not been swapped yet (firmware's cmd_softreset
 *      runs after we send the mailbox command).
 *   2. Even after the swap, we cannot rely on the cart bus serving the
 *      code we want to execute - the new mapper is being installed
 *     ~concurrently, and our `jp 0x0000` would fetch a byte off the
 *      cart bus while the swap is in flight.
 *
 * Solution: copy a Z80 slingshot to MSX RAM at 0xE000, then
 * `jp 0xE000` into it.  Everything from that jp onwards runs from MSX
 * RAM, so the cart swap cannot yank executing code out from under
 * the Z80.
 *
 *  *** WBOOT CLEAR + jp 0x0000 - DOES NOT WORK on user's MSX ***
 * An earlier version of the slingshot only wrote 0 to the MSX
 * system variable WBOOT at RAM 0xF3B0 (forcing a BIOS cold-boot
 * re-probe) and then did `jp 0x0000`. Symptom on the user's MSX
 * (no LA activity at 0x4000 after the slingshot ran):
 *   - the BIOS at 0x0000 did NOT re-probe the cart even with
 *     WBOOT cleared. Most MSX2+ BIOSes have RST 0 enter a
 *     path that DOES NOT do cold-boot slot expansion on its
 *     own - they trust the SLTTBL in system RAM, which was
 *     populated during the initial firmware-boot probe and
 *     shows "no cart in this slot" because the FLASH mapper
 *     was active then but the BIOS state has long since
 *     moved to BASIC.
 *   - Without a true hardware reset, the BIOS just returns to
 *     BASIC warm-start. Cart stays invisible.
 *
 *  *** Direct cart INIT invocation (current implementation) ***
 * Instead of asking the BIOS to re-probe, the slingshot invokes
 * the cart's INIT routine directly via the MSX BIOS CALSLT
 * (0x001C) entry point. Steps:
 *   1. Read cart header bytes 0x4000-0x4003 from slot 1 via
 *      RDSLT (0x000C). Check 0x4000='A', 0x4001='B'. If not,
 *      try slot 2 the same way.
 *   2. Read INIT_LO = 0x4002, INIT_HI = 0x4003. Form the cart's
 *      INIT absolute address = INIT_HI:INIT_LO (little-endian
 *      word stored in the cart header).
 *   3. Set IX = INIT address, IY high byte = slot number,
 *      EI (so the cart's INIT can use interrupts),
 *      call CALSLT. The cart's INIT runs and takes over.
 *
 * This bypasses the BIOS entirely: the cart's INIT is exactly
 * what the BIOS would have called anyway. As a last resort,
 * if neither slot 1 nor slot 2 has "AB" at 0x4000, the
 * slingshot falls back to `jp 0x0000` (letting the BIOS
 * warm-start run - not useful but harmless).
 *
 * Slingshot is 130 bytes. Install target moved from 0xF000
 * to 0xE000 (page 3, well below the BIOS system area at
 * 0xF380, above SDCC's _DATA at 0xC000).
 */

/* Hand-encoded Z80 (NOT a function - the byte stream is what we want
 * to copy verbatim into MSX RAM). Assembled with sdasz80 from
 * /tmp/slingshot.s; bytes copied here verbatim. 130 bytes total. */
static const unsigned char soft_reset_code[130] = {
    /* 0x00: prologue (12 bytes) */
    0xF3,                   /* di                              */
    0x3E, 0x00,             /* ld a, 0                         */
    0xED, 0x47,             /* ld i, a                         */
    0x31, 0xFE, 0xFF,       /* ld sp, 0xFFFE                   */
    0x21, 0xB0, 0xF3,       /* ld hl, 0xF3B0 (WBOOT, harm=ok) */
    0x36, 0x00,             /* ld (hl), 0                      */
    /* 0x0D: try slot 1 - read 0x4000 */
    0x26, 0x40,             /* ld h, 0x40                      */
    0x2E, 0x00,             /* ld l, 0x00                      */
    0x3E, 0x01,             /* ld a, 0x01 (slot 1)             */
    0xCD, 0x0C, 0x00,       /* call 0x000C (RDSLT)             */
    0xFE, 0x41,             /* cp 'A'                          */
    0x20, 0x2C,             /* jr nz, +0x2C -> try_slot_2      */
    /* 0x1A: read 0x4001 */
    0x26, 0x40,
    0x2E, 0x01,
    0x3E, 0x01,
    0xCD, 0x0C, 0x00,
    0xFE, 0x42,             /* cp 'B'                          */
    0x20, 0x1F,             /* jr nz, +0x1F -> try_slot_2      */
    /* 0x27: read 0x4002 (INIT_LO) -> E */
    0x26, 0x40,
    0x2E, 0x02,
    0x3E, 0x01,
    0xCD, 0x0C, 0x00,
    0x5F,                   /* ld e, a                         */
    /* 0x31: read 0x4003 (INIT_HI) -> D */
    0x26, 0x40,
    0x2E, 0x03,
    0x3E, 0x01,
    0xCD, 0x0C, 0x00,
    0x57,                   /* ld d, a                         */
    /* 0x3B: IX=DE; IY=slot<<8; EI; CALSLT */
    0xD5,                   /* push de                         */
    0xDD, 0xE1,             /* pop ix                          */
    0xFD, 0x21, 0x00, 0x01, /* ld iy, 0x0100 (slot 1)         */
    0xFB,                   /* ei                              */
    0xCD, 0x1C, 0x00,       /* call 0x001C (CALSLT)            */
    /* 0x46: try slot 2 - read 0x4000 */
    0x26, 0x40,
    0x2E, 0x00,
    0x3E, 0x02,             /* ld a, 0x02 (slot 2)             */
    0xCD, 0x0C, 0x00,
    0xFE, 0x41,
    0x20, 0x2C,             /* jr nz, +0x2C -> fail           */
    /* 0x53: read 0x4001 */
    0x26, 0x40,
    0x2E, 0x01,
    0x3E, 0x02,
    0xCD, 0x0C, 0x00,
    0xFE, 0x42,
    0x20, 0x1F,             /* jr nz, +0x1F -> fail           */
    /* 0x60: read 0x4002 (INIT_LO) -> E */
    0x26, 0x40,
    0x2E, 0x02,
    0x3E, 0x02,
    0xCD, 0x0C, 0x00,
    0x5F,
    /* 0x6A: read 0x4003 (INIT_HI) -> D */
    0x26, 0x40,
    0x2E, 0x03,
    0x3E, 0x02,
    0xCD, 0x0C, 0x00,
    0x57,
    /* 0x74: IX=DE; IY=slot<<8; EI; CALSLT */
    0xD5,
    0xDD, 0xE1,
    0xFD, 0x21, 0x00, 0x02, /* ld iy, 0x0200 (slot 2)         */
    0xFB,
    0xCD, 0x1C, 0x00,
    /* 0x7F: fall back to jp 0x0000 */
    0xC3, 0x00, 0x00
};

/* Copy the slingshot from ROM-resident soft_reset_code[] into MSX RAM
 * at 0xE000. SDCC places const arrays in _CODE (0x4020+), so we have
 * to byte-copy them - the cart bus is the only way to read them, and
 * the cart handler is what we're trying to *swap out*, so any code
 * that touches the cart from this point on is unsafe. The copy goes
 * via 'lxi h,<src>; lxi d,0E000h; mvi c,N; ld a,(hl); ldi; dcr c;
 * jnz' etc.; easier to emit the loop inline in __asm and have SDCC
 * not touch it.
 *
 * The slingshot size (130 bytes) is a compile-time constant kept in
 * sync with soft_reset_code[]. If you edit the array, edit
 * `ld c, #130` here too.
 *
 * We also DI at the top - the function itself returns (no caller
 * resumes us).
 */
static void soft_reset_install(void) __naked
{
	__asm
		;; di - mirrors crt0; belt and braces
		di
		;; source = soft_reset_code (resolved by SDCC at link time)
		ld	hl, #_soft_reset_code
		;; dest   = 0xE000
		ld	de, #0xE000
		;; count  = 130 (keep in sync with soft_reset_code[])
		ld	c, #130
		;; copy loop: ld a,(hl); ld (de),a; inc hl; inc de; dec c; jr nz,-5
	loop$:
		ld	a, (hl)
		ld	(de), a
		inc	hl
		inc	de
		dec	c
		jr	nz, loop$
		;; Function returns - the slingshot at 0xE000 is what
		;; soft_reset() enters, not this function. We MUST return
		;; to the C caller (main()), which continues to set up the
		;; menu etc. until the user picks a file/mapper.
		ret
	__endasm;
}

/* Enter the slingshot at 0xE000. Naked: emits exactly `jp 0xE000`,
 * nothing else. After this jp, the Z80 executes ONLY code from MSX
 * RAM - never the cart - until the slingshot's CALSLT invokes the
 * cart's INIT routine (which then runs from the new cart mapper).
 * Call ONLY after CMD_SOFTRESET has been issued and the firmware
 * has had at least a few ms to start its mapper swap
 * (cmd_softreset's Cart_SetMapper_Safe takes ~2 ms; the slingshot
 * itself runs in <1 ms total because it just reads the cart header
 * via RDSLT and CALSLTs into the INIT - no delay loop needed).
 *
 * The settling delay (~50 ms) for the cart bus + PSRAM to settle
 * after the firmware's mapper swap is in firmware/User/loader.c
 * ::cmd_softreset, BEFORE the firmware marks the mailbox done.
 * The MSX-side mbox_wait_done() blocks during it. If we need
 * more delay in the future, increase it there rather than adding
 * a delay loop to the slingshot (the slingshot is already at its
 * size budget for 0xE000). */
static void soft_reset(void) __naked
{
	__asm
		jp	0xE000
	__endasm;
}

/* ---- main ----------------------------------------------------------- */

int main(void)
{
	/* Install the soft-reset slingshot to MSX RAM at 0xE000 BEFORE
	 * any other work. The slingshot must be in place by the time
	 * soft_reset() is called at the end of main(); installing it
	 * here means a stray early reset (firmware crash, USB
	 * disconnect during dir scan, etc.) still has a working
	 * slingshot to fall back on. */
	soft_reset_install();

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
			wait_any_key();
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
		uint8_t args[LOADER_NAME_LEN];
		for (uint8_t i = 0; i < LOADER_NAME_LEN; i++)
			args[i] = (uint8_t)s_files[sel][i];
		mbox_cmd(CMD_LOAD_ROM, args, LOADER_NAME_LEN);
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
				wait_any_key();
		}
	}

	/* ---- reboot the MSX; firmware serves the new ROM on boot ----
	 *
	 * CMD_SOFTRESET is the ONLY reboot path. There is no CMD_RESET:
	 * driving MSX ~RESET externally can damage the mainboard on
	 * MSXs whose cart-edge reset line is read-only.
	 *
	 * Sequence:
	 *   1. mbox_cmd waits for the firmware to finish
	 *      Cart_SetMapper_Safe (~2 ms). The firmware-side
	 *      cmd_softreset returns IMMEDIATELY after the swap - no
	 *      firmware-side delay, so the firmware's main loop stays
	 *      free to service CLI + USB.
	 *   2. delay_ms(50) gives the PSRAM + cart bus time to settle
	 *      after the mapper swap. Without this, some games fail to
	 *      boot on the user's MSX: the cart's INIT runs but reads
	 *      garbage from 0x4000 because the PSRAM controller hasn't
	 *      settled after the firmware's large DMA write (LOAD_ROM)
	 *      followed by the mapper swap.
	 *   3. print("BOOT...") confirms we got this far.
	 *   4. soft_reset() emits `jp 0xE000` and the Z80 enters the
	 *      slingshot. The slingshot reads the cart header via RDSLT
	 *      and invokes the cart's INIT via CALSLT - bypasses the
	 *      MSX BIOS cold-boot probe entirely (which on most MSX2+
	 *      BIOSes does NOT re-probe the cart even with WBOOT=0).
	 *
	 * If the firmware is missing/broken (mailbox write no-ops),
	 * mbox_wait_done never sees ST_DONE and we spin forever in
	 * the wait loop - the user sees "BOOT..." but the screen
	 * doesn't change. Press reset on the MSX to recover (the
	 * loader menu comes back because CART_MAP_FLASH was never
	 * touched). */
	print("BOOT...");
	mbox_cmd(CMD_SOFTRESET, 0, 0);
	delay_ms(50);
	soft_reset();

	/* soft_reset() never returns; the slingshot jp's to 0x0000 which
	 * never comes back to main(). The for(;;) is defensive in case
	 * the firmware's reply signals CMD_SOFTRESET handled the FLASH
	 * fallback path (s_pending_mapper was NONE) and the slingshot's
	 * jp 0x0000 re-booted into the loader menu - we'd see this loop
	 * only if soft_reset() somehow returned without jp'ing, which
	 * the naked inline asm cannot do. */
	for (;;) { }
}