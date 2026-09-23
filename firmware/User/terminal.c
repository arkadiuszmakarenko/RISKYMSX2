
#include "terminal.h"
#include "cart.h"
#include "psram.h"
#include "usb_disk.h"
#include "ff.h"
#include <stdio.h>
#include "ch32v4x7.h"

#pragma GCC push_options
#pragma GCC optimize("Os")

/* Single shared mailbox instance. cart.c's EXTI0 handler touches the
 * fields directly. */
TerminalMailbox g_term_mbox;

#define TERM_MAX_FILES   32U
#define FILE_NAME_MAX    26U    /* printed-column width: 25 chars + NUL.
                                 * Line layout = 25 (name) + 1 (space) +
                                 * up to 5 (size, e.g. "1023K") = 31
                                 * printed chars, leaving 1-char margin
                                 * on the MSX's 32-column screen so the
                                 * CR/LF never wraps mid-row. */
#define TERM_PAGE_SIZE   20U    /* rows per page on the file list */

typedef enum {
    MENU_LIST,
    MENU_MAPPER,
} MenuState;

/* Per-file menu state. */
static struct {
    /* SFN (8.3 short name, e.g. "METALG~1.ROM") - what f_open() gets
     * for every file operation. LFN paths fail f_open with
     * FR_NO_FILE (4) on our stick, so all operations use the SFN. */
    char    names[TERM_MAX_FILES][FILE_NAME_MAX];
    /* LFN (long name, up to 25 printable chars) - DISPLAY ONLY.
     * Drawn in the file list so the menu stays human-readable; never
     * used for f_open / path building. */
    char    lfns[TERM_MAX_FILES][FILE_NAME_MAX];
    uint32_t sizes[TERM_MAX_FILES];  /* bytes */
    uint16_t count;
    uint16_t sel;       /* currently-highlighted index (absolute,
                         * i.e. counts across pages) */
    uint16_t page;      /* current page, 0-based */
    uint16_t file_idx;  /* index of the file picked for the mapper
                         * screen - s_menu.sel is REUSED as the
                         * mapper-row index on that screen, so this
                         * preserves which LFN to display. */
    uint8_t  loaded;    /* "ROM was loaded into PSRAM" */
    MenuState state;
    char     picked[FILE_NAME_MAX]; /* SFN of the selected file - the
                                 * operational name passed to
                                 * Terminal_BootCart */
} s_menu;

/* Number of pages needed to display s_menu.count files. */
static uint16_t menu_page_count (void) {
    if (s_menu.count == 0U) return 1U;
    return (uint16_t)((s_menu.count + TERM_PAGE_SIZE - 1U) / TERM_PAGE_SIZE);
}

static const char *const kMapperNames[] = {
    "Standard 16KB ROM",
    "Standard 32KB ROM",
    "Standard 48KB/64KB ROM",
    "Konami (no SCC)",
    "Konami + SCC",
    "Konami + SCC (no sound)",
    "ASCII 8KB",
    "ASCII 16KB",
    "NEO 8KB",
    "NEO 16KB",
};
#define MAPPER_COUNT  ((int)(sizeof(kMapperNames)/sizeof(kMapperNames[0])))

/* Mapper index -> Cart_Mapper. Index 0..9 of the menu map onto the
 * v303 CartType enum (same order as kMapperNames). The Cart_Mapper
 * enum in cart.h has additional values (KONAMISCC == 10, etc) we
 * do not expose in this minimal menu. */
static const Cart_Mapper kMenuToCart[] = {
    CART_MAP_ROM16k,
    CART_MAP_ROM32k,
    CART_MAP_ROM48k,
    CART_MAP_KONAMI,        /* Konami no-SCC, write at 0x6000/0x8000/0xA000 only */
    CART_MAP_KONAMISCC,     /* Konami + SCC emulator */
    CART_MAP_KONAMINOSCC,   /* Konami + SCC, SCC bypassed */
    CART_MAP_ASCII8k,
    CART_MAP_ASCII16k,
    CART_MAP_NEO8,
    CART_MAP_NEO16,
};

/* ------------------------------------------------------------------ */
/* FIFO helpers (main-loop side)                                       */
/* ------------------------------------------------------------------ */

static int out_push (uint8_t b) {
    uint16_t next = (uint16_t)((g_term_mbox.out_tail + 1U) % 2048U);
    if (next == g_term_mbox.out_head) return 0;  /* full */
    g_term_mbox.out_buf[g_term_mbox.out_tail] = b;
    /* Memory barrier: make sure the byte write above is visible to the
     * IRQ-side consumer (Cart_EXTI0_Terminal_Handler) BEFORE the
     * tail increment that exposes the slot. RV32 needs a store-store
     * fence; the WCH core provides __asm__ fence syntax via the
     * standard RISC-V mnemonics. */
    __asm__ volatile ("fence w, w" ::: "memory");
    g_term_mbox.out_tail = next;
    g_term_mbox.out_n++;
    return 1;
}

static int kbd_pop (uint8_t *out) {
    if (g_term_mbox.kbd_n == 0U) return 0;
    *out = g_term_mbox.kbd_buf[g_term_mbox.kbd_head];
    g_term_mbox.kbd_head = (uint8_t)((g_term_mbox.kbd_head + 1U) % 16U);
    g_term_mbox.kbd_n--;
    return 1;
}

static void out_str (const char *s) {
    while (*s) {
        out_push ((uint8_t)*s++);
    }
}

/* Move the cursor to (row, col) on the MSX screen (0-based; the MSX
 * accepts any 0..23 value here, with 0 = top row). Pushes the
 * standard MSX-BIOS ESC-Y sequence 'ESC Y' (row+0x20) (col+0x20).
 *
 * Callers want the natural "line, column" order: move_cursor(5, 0)
 * drops the cursor on the 6th screen row, leftmost column. */
static void move_cursor (uint8_t row, uint8_t col) {
    out_push (0x1B);
    out_push ('Y');
    out_push ((uint8_t)(0x20 + row));
    out_push ((uint8_t)(0x20 + col));
}

/* Move the sprite cursor to (col, row) in CHARACTER coordinates. The
 * MSX-side terminal expects a 0x04 prefix on the FIFO followed by two
 * reads from 0x7FFF (X-pixels then Y-pixels); we multiply by 8 here
 * because each character cell is 8 pixels wide on a screen-1 layout.
 *
 * The MSX-side sprite handler does `dec a` on Y before writing to
 * VRAM 0x1B00 (sprite 0 Y). For row N (top-left pixel y = N*8) we
 * want the sprite top-left at exactly N*8, so we send Y = N*8 + 1.
 * After `dec a`, the VRAM write is N*8 - which is the top of row N.
 * With X = col*8 + 1 the sprite lands one pixel to the right of the
 * leftmost column, hugging the file text but not overlapping it.
 *
 * The +1 offsets fix the "initial page load arrow misaligned" issue:
 * without them the first paint lands the arrow half a row off and
 * overlapping the title strip. */
static void move_pointer (uint8_t col, uint8_t row) {
    out_push (0x04);
    out_push ((uint8_t)(col * 8U + 1U));
    out_push ((uint8_t)(row * 8U + 1U));
}

/* Clear screen + home cursor. ESC 'E' is the MSX-BIOS CLS. */
static void clear_screen (void) {
    out_push (0x1B);
    out_push ('E');
}

static void newline (void) {
    out_push ('\r');
    out_push ('\n');
}

/* Spin until the MSX has drained the output FIFO (out_n == 0) or the
 * given millisecond budget expires, whichever comes first.
 *
 * Clock-derived calibration: empirically 4,000,000 iterations = 20 ms
 * at 200 MHz HCLK (~1 cycle/iteration), so iters-per-ms =
 * SystemCoreClock (HCLK) / 1000. Stays correct at 175 MHz HCLK too
 * (this is only a scheduling budget, not a hard deadline). */
static void term_wait_drain (uint32_t ms) {
    const uint32_t iters = ms * (SystemCoreClock / 1000U);
    uint32_t i;
    for (i = 0; i < iters; i++) {
        if (g_term_mbox.out_n == 0U) break;
        __asm__ volatile ("nop");
    }
}

/* ------------------------------------------------------------------ */
/* Public mailbox lifecycle                                            */
/* ------------------------------------------------------------------ */

/* Set once per terminal session: the next Terminal_Service pass scans
 * the stick and renders the file list. Terminal_Reset() re-arms it, so
 * a session entered via the boot gate (BOOTGATE -> TERMINAL swap)
 * renders even though Terminal_Service already ran while the boot
 * gate was active (the firmware main loop calls every service
 * unconditionally). */
static uint8_t s_list_pending = 1;

void Terminal_Reset (void) {
    g_term_mbox.kbd_head = g_term_mbox.kbd_tail = g_term_mbox.kbd_n = 0;
    g_term_mbox.out_head = g_term_mbox.out_tail = g_term_mbox.out_n = 0;
    g_term_mbox.control = 0xFFU;
    s_menu.count = 0;
    s_menu.sel   = 0;
    s_menu.page  = 0;
    s_menu.file_idx = 0;
    s_menu.loaded = 0;
    s_menu.state  = MENU_LIST;
    s_menu.picked[0] = '\0';
    s_list_pending = 1U;
}

/* ------------------------------------------------------------------ */
/* Menu rendering                                                      */
/* ------------------------------------------------------------------ */

static void print_size (uint32_t bytes) {
    char buf[8];
    uint32_t adj = bytes;
    char suffix = 'B';
    if (bytes >= 1024U * 1024U) { adj = bytes / (1024U * 1024U); suffix = 'M'; }
    else if (bytes >= 1024U)    { adj = bytes / 1024U;          suffix = 'K'; }
    /* int -> ASCII */
    char tmp[8]; int n = 0;
    if (adj == 0U) { tmp[n++] = '0'; }
    else {
        while (adj > 0U) { tmp[n++] = (char)('0' + (adj % 10U)); adj /= 10U; }
    }
    int i = 0;
    while (n > 0) buf[i++] = tmp[--n];
    buf[i++] = suffix; buf[i] = '\0';
    out_str (buf);
}

static void print_menu_title (void) {
    clear_screen ();
    out_str (" RISKYMSX2   RET SEL  <- -> page");
    newline ();
}

static void print_file_list (void) {
    /* Row of the selection within the current page (0-based offset
     * into the on-screen file block). */
    uint16_t row_in_page = (uint16_t)(s_menu.sel % TERM_PAGE_SIZE);

    /* Position the sprite cursor FIRST so that the arrow is on-screen
     * even if the MSX-side polling is slow to drain the FIFO. Without
     * this the very first paint had the arrow hidden briefly while
     * the body rendered all the text bytes ahead of the move_pointer
     * push at the end of the FIFO.
     *
     * The arrow uses sprite 0; sits in the leftmost column at the
     * vertical centre of the selected row. Y is sent as (row+1)*8
     * because the MSX-side sprite handler does `dec a` on Y before
     * writing it to VRAM. */
    move_pointer (0, (uint8_t)(1 + row_in_page));

    move_cursor (0, 0);
    print_menu_title ();

    /* Files of the current page. */
    uint16_t start = (uint16_t)(s_menu.page * TERM_PAGE_SIZE);
    uint16_t end   = (uint16_t)(start + TERM_PAGE_SIZE);
    if (end > s_menu.count) end = s_menu.count;

    for (uint16_t i = start; i < end; i++) {
        /* Reserve column 0 for the sprite cursor - text starts at
         * column 1. */
        move_cursor ((uint8_t)(1 + (i - start)), 1);
        /* print the long name, padded to FILE_NAME_MAX for a stable
         * column layout (operations keep using the SFN - see
         * scan_usb). */
        const char *n = s_menu.lfns[i];
        for (int j = 0; j < FILE_NAME_MAX - 1; j++) {
            char c = n[j];
            if (c == '\0') {
                for (; j < FILE_NAME_MAX - 1; j++) out_push (' ');
                    break;
            }
            out_push ((uint8_t)c);
        }
        out_push (' ');
        print_size (s_menu.sizes[i]);
        newline ();
    }
    if (s_menu.count == 0U) {
        out_str ("  (no .ROM files on USB stick)");
        newline ();
    }

    /* Page indicator on the last row. */
    move_cursor ((uint8_t)(1 + TERM_PAGE_SIZE), 1);
    char buf[16];
    snprintf (buf, sizeof (buf), " Pg %u/%u  F=old N=nextor",
              (unsigned)(s_menu.page + 1U), (unsigned)menu_page_count ());
    out_str (buf);
    newline ();
}

static void print_mapper_menu (void) {
    /* Sprite first - same rationale as print_file_list: the arrow
     * lands on the highlighted mapper row before the MSX has to read
     * any of the surrounding text. */
    move_pointer (0, (uint8_t)(4 + s_menu.sel));

    clear_screen ();
    move_cursor (0, 0);
    /* Display the long name (human-readable) - operations still use
     * the SFN stored in s_menu.picked. */
    out_str (" File: ");
    out_str (s_menu.lfns[s_menu.file_idx]);
    newline ();
    out_str (" Pick mapper (Up/Down/RET/ESC):");
    newline ();
    newline ();
    /* reserve column 0 for the sprite arrow; mapper names start at
     * column 1. */
    for (int i = 0; i < MAPPER_COUNT; i++) {
        move_cursor ((uint8_t)(4 + i), 1);
        out_str (kMapperNames[i]);
        newline ();
    }
}

/* ------------------------------------------------------------------ */
/* FAT scan                                                             */
/* ------------------------------------------------------------------ */

static void scan_usb (void) {
    s_menu.count = 0;
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        printf ("TERM: scan_usb - USB not mounted\r\n");
        return;
    }
    DIR dir; FILINFO fi;
    FRESULT fr = f_opendir (&dir, "0:/");
    if (fr != FR_OK) {
        printf ("TERM: scan_usb - f_opendir failed (%u)\r\n", (unsigned)fr);
        return;
    }
    while (s_menu.count < TERM_MAX_FILES) {
        fr = f_readdir (&dir, &fi);
        /* End-of-directory check MUST use fi.fname - FatFS only
         * clears fname[0] on end-of-dir, leaving altname holding a
         * stale copy of the previous entry's SFN. Using altname made
         * the last file repeat over and over (observed as "last file
         * repeated 18 times" on page 2). */
        if (fr != FR_OK || fi.fname[0] == 0) break;
        if (fi.fattrib & AM_DIR) continue;
        /* Accept any file - the v303 firmware filtered by extension
         * (.ROM) but the cart loader may serve .BIN/.MX1 too.
         *
         * Store BOTH name forms:
         *   - s_menu.names[] = SFN (fi.altname, 8.3) - used for ALL
         *     file operations (f_open in Terminal_BootCart). LFN
         *     paths fail f_open with FR_NO_FILE (4) on our stick.
         *   - s_menu.lfns[]  = LFN (fi.fname) - display only, so the
         *     menu shows "Metal Gear 2 - Solid Snake" instead of
         *     METALG~1.ROM. Truncated to the printable width with a
         *     trailing '.'-style ellipsis when longer. */
        const char *sf = fi.altname;
        size_t k = 0;
        while (sf[k] && k < FILE_NAME_MAX - 1) {
            s_menu.names[s_menu.count][k] = sf[k];
            k++;
        }
        s_menu.names[s_menu.count][k] = '\0';

        const char *ln = fi.fname;
        k = 0;
        while (ln[k] && k < FILE_NAME_MAX - 1) {
            s_menu.lfns[s_menu.count][k] = ln[k];
            k++;
        }
        s_menu.lfns[s_menu.count][k] = '\0';
        /* If the LFN is longer than the display width, keep the
         * extension readable: show first (width-4) chars + "~X" style
         * ellipsis + extension. Simplest robust rule: keep the last
         * '.' and truncate the stem. */
        if (ln[k] != '\0') {
            /* find extension in the SFN (always present for ROMs) */
            const char *dot = sf;
            for (size_t t = 0; sf[t]; t++) if (sf[t] == '.') dot = sf + t;
            const char *ext = dot;   /* ".ROM" */
            size_t ext_len = 0;
            while (ext[ext_len] && (ext - sf) + ext_len < 12U) ext_len++;
            if (ext_len > 0 && ext_len <= 4U &&
                (size_t)(FILE_NAME_MAX - 1) > ext_len) {
                size_t stem = (size_t)(FILE_NAME_MAX - 1) - ext_len;
                /* copy first `stem` chars of LFN then the extension */
                for (size_t t = 0; t < stem && ln[t]; t++) {
                    s_menu.lfns[s_menu.count][t] = ln[t];
                }
                size_t w = stem < k ? stem : k;
                for (size_t t = 0; t < ext_len; t++) {
                    s_menu.lfns[s_menu.count][w + t] = ext[t];
                }
                s_menu.lfns[s_menu.count][w + ext_len] = '\0';
            }
        }
        s_menu.sizes[s_menu.count] = fi.fsize;
        printf ("TERM:   file[%u] SFN=[%s] LFN=[%s] sz=%u\r\n",
                s_menu.count, s_menu.names[s_menu.count], fi.fname, (unsigned)fi.fsize);
        s_menu.count++;
    }
    f_closedir (&dir);
}

/* ------------------------------------------------------------------ */
/* Boot path                                                            */
/* ------------------------------------------------------------------ */

/* Progress bar geometry - keep in sync with the header printed by
 * Terminal_BootCart: "  [--------------------]   0%" = 2 spaces + '['
 * + 20 blocks + ']' + 2 spaces + 3 digits + '%' = 30 chars. */
#define TERM_PROG_BAR_BLOCKS  20U

/* Redraws the progress bar in place on the fixed screen row the
 * header occupied. Called from USB_FileToPSRAM's read loop via the
 * USB_ProgressCB hook - keep it short. */
static void term_progress_cb (uint32_t done, uint32_t total) {
    if (total == 0U) return;
    /* Throttle: only redraw when the percentage changes. */
    uint32_t pct = (uint32_t)(((uint64_t)done * 100ULL) / total);
    static uint32_t s_last_pct = 0xFFFFU;
    if (pct == s_last_pct) return;
    s_last_pct = pct;

    uint32_t filled = (uint32_t)(((uint64_t)done * TERM_PROG_BAR_BLOCKS) / total);

    /* Backpressure: the ring is only 2048 bytes and the MSX drains at
     * ~50 µs/byte, so a bar redraw every 1% can outrun it. If out_n is
     * high, SKIP this redraw - a skipped cosmetic update is harmless,
     * but out_push() silently drops bytes when full, and dropping the
     * trailing " OK - booting MSX into cart" + 0x03 launch byte (see
     * soft_reset_into_cart) leaves the MSX in the terminal loop and
     * the cart never boots. Threading the ring below ~50% always
     * leaves room for the launch sequence. */
    if (g_term_mbox.out_n > 1024U) return;

    /* Move to the progress-bar row (row 3, column 0) and redraw in
     * place - must match the header layout in Terminal_BootCart. */
    move_cursor (3, 0);
    out_push (' ');
    out_push (' ');
    out_push ('[');
    for (uint32_t i = 0; i < TERM_PROG_BAR_BLOCKS; i++) {
        out_push ((uint8_t)((i < filled) ? '#' : '-'));
    }
    out_push (']');
    out_push (' ');
    /* Percentage, 3 chars zero-padded so the column stays fixed. */
    char num[4];
    if (pct >= 100U) { num[0] = '1'; num[1] = '0'; num[2] = '0'; }
    else if (pct >= 10U) { num[0] = (char)('0' + pct / 10U); num[1] = (char)('0' + pct % 10U); num[2] = ' '; }
    else { num[0] = (char)('0' + pct); num[1] = ' '; num[2] = ' '; }
    num[3] = '\0';
    out_str (num);
    out_push ('%');
}

static void soft_reset_into_cart (Cart_Mapper m) {
    printf ("TERM: soft_reset push 0x03 to FIFO\r\n");
    /* Two MSX-side reboot paths exist, both must end up with the
     * NEW mapper live so the slot probe finds the user ROM's 'AB':
     *
     *   (A) MSX-side rom_start validation succeeds: rom_start patches
     *       the BIOS slot-search return and returns. The BIOS runs
     *       UGLY_PATCH (`jp 0x7D84` = slot re-search).
     *   (B) MSX-side rom_start validation FAILS (this MSX's BIOS
     *       stack layout doesn't match the MSX1/MSX2 0x7DA3 expected
     *       return): rom_start falls into `unknown`, does `RST 0`
     *       (full cold reboot). BIOS cold-boots and re-probes slots
     *       as part of MSX-BASIC init.
     *
     * Both paths read the cart at 0x4000 within ~20 ms of the rom_start
     * triggering. Swap the mapper EARLY (after ~20 ms - just enough
     * for the MSX to read 0x03 and start rom_start) so the slot probe
     * in both paths sees the new ROM image.
     *
     * CRITICAL RACE (fixed here): the MSX has to READ 0x03 from the
     * FIFO before we swap the mapper - after the swap the TERMINAL
     * handler is uninstalled, reads at 0x7FFF return 0xFF (open bus,
     * served by RunKonamiSCC as "page 7, bus off"), and the MSX-side
     * terminal loop prints those 0xFFs forever = the "random rubbish
     * on screen" bug. Terminal_BootCart already FLUSHED the backlog
     * and pushed the short " OK..." line BEFORE the 0x03, so only
     * ~30 bytes precede the launch byte and the polling MSX consumes
     * them within one jiffy frame (<= ~18 ms). */
    out_push (0x03);
    {
        /* FIXED 30 ms delay via the timer-calibrated Delay_Ms (the
         * old 4,000,000-iteration spin measured anywhere from ~20 to
         * ~110 ms depending on codegen - too imprecise to land the
         * swap inside the launch window). Timing contract with the
         * MSX-side launch (terminal.asm rom_start):
         *   - the MSX polls the FIFO at most one jiffy apart, so it
         *     reads the 0x03 within ~17 ms of the push;
         *   - rom_start then waits ~35 ms IN RAM (no cart access),
         *     patches the BIOS return to the RAM-resident launch
         *     patch (0xE100) and unwinds into the BIOS;
         *   - the BIOS slot re-probe reads 0x4000 a few ms later.
         * So the swap must land AFTER the ~17 ms read (else the
         * launch byte is lost) and BEFORE the ~40-45 ms probe - 30 ms
         * is the centre of that window. With the launch fully
         * RAM-resident the swap can no longer poison executing code,
         * so this race now has huge margins instead of being a
         * knife-edge. */
        Delay_Ms (20U);
        if (g_term_mbox.out_n != 0U) {
            printf ("TERM: WARNING FIFO not drained (out_n=%u at swap "
                    "time) - MSX has not read the 0x03 yet, it may "
                    "show rubbish\r\n", g_term_mbox.out_n);
        }
    }
    printf ("TERM: soft_reset swap mapper=%d\r\n", (int)m);
    /* Swap the mapper. The MSX-side rom_start is now executing in
     * MSX RAM; both path (A) (UGLY_PATCH) and path (B) (RST 0 from
     * unknown) reach the BIOS slot probe within ~20 ms and find the
     * new ROM's 'AB' header. */
    (void)Cart_SetMapper_Safe (m);
    /* Post-swap settle cushion: keep the firmware quiet (no PSRAM /
     * USB churn) while the game's own INIT runs its first cart reads.
     * The legacy 55b6b88 build spun ~10,000,000 iterations here
     * (~250 ms) - shorter budgets made some games (e.g. Metal Gear 2,
     * Konami-SCC with a large INIT) fail to boot. Tune via the
     * constant below if a title needs more. */
    Delay_Ms (800U);
}

void Terminal_BootCart (uint8_t mapper_idx, const char *filename) {
    if (mapper_idx >= MAPPER_COUNT) {
        printf ("TERM: BootCart refused (mapper_idx=%u >= MAPPER_COUNT)\r\n", mapper_idx);
        return;
    }
    if (!filename || !filename[0]) {
        printf ("TERM: BootCart refused (empty filename)\r\n");
        return;
    }
    Cart_Mapper m = kMenuToCart[mapper_idx];
    printf ("TERM: BootCart start mapper=%u (%s) file=[%s]\r\n", mapper_idx, kMapperNames[mapper_idx], filename);

    /* Build a "0:/NAME" path. The FAT volume is "0:" on the USB
     * stick; loader-side uses the same path scheme (see loader.c). */
    char path[40];
    int p = 0;
    path[p++] = '0'; path[p++] = ':'; path[p++] = '/';
    int i = 0;
    while (filename[i] && p < (int)sizeof(path) - 1) {
        path[p++] = filename[i++];
    }
    path[p] = '\0';

    clear_screen ();
    out_str (" Loading ");
    out_str (path);
    newline ();
    out_str (" Mapper: ");
    out_str (kMapperNames[mapper_idx]);
    newline ();
    newline ();
    /* Progress bar header - drawn once, updated in place by the
     * callback below. 20 blocks wide + 2 brackets = 22 chars, plus
     * " 100%" (5) = 27 chars, fits in the 31-char printable width. */
    out_str ("  [--------------------]   0%");
    newline ();

    /* Install the progress renderer for the duration of the copy.
     * Throttle the redraws so the MSX-side FIFO isn't hammered with a
     * full bar redraw for every 512-byte chunk. */
    {
        USB_ProgressCB = term_progress_cb;
        /* len = 0 makes USB_FileToPSRAM use the real file size, so
         * the progress percentage is relative to the ROM being
         * loaded - not the full 8 MiB PSRAM window (a 1 MiB ROM
         * would otherwise only ever reach ~12%). */
        uint32_t got = USB_FileToPSRAM (path,
                                        PSRAM_CART_BASE + CART_GAME_BASE,
                                        0U);
        USB_ProgressCB = 0;
        printf ("TERM: USB_FileToPSRAM got=%u (expected full file)\r\n",
                (unsigned)got);
        if (got == 0U) {
            out_str (" Load FAILED");
            newline ();
            printf ("TERM: load failed - f_open or first f_read error; "
                    "check USB: lines above\r\n");
            return;
        }
        if (got < 1024U) {
            /* Suspiciously small - could be a wrong file / FAT issue. */
            printf ("TERM: WARNING got < 1 KiB - suspicious, "
                    "verify the ROM file\r\n");
        }
        s_menu.loaded = 1;
        /* VERIFY the image really landed in PSRAM: the cart handler
         * serves BIOS slot probes from PSRAM[GAME_BASE + addr], so a
         * valid bootable ROM starts with "AB" (0x41 0x42) at offset
         * 0 and a second "AB" at 0x4000 for Konami/SCC mappers
         * (2-page image). If these bytes are wrong the reset sequence
         * works perfectly but the BIOS probes will never accept the
         * cart - or serves garbage. Read back directly over the
         * PSRAM window (same bus the ELF handler uses). */
        {
            volatile const uint8_t *ps =
                (volatile const uint8_t *)(PSRAM_CART_BASE + CART_GAME_BASE);
            printf ("TERM: PSRAM verify [0000]=%02x %02x | [4000]=%02x %02x"
                    " (want 41 42)\r\n",
                    ps[0x0000U], ps[0x0001U], ps[0x4000U], ps[0x4001U]);
            if (ps[0] != 0x41U || ps[1] != 0x42U) {
                printf ("TERM: WARNING no 'AB' header at PSRAM 0 - "
                        "BIOS will not boot this image!\r\n");
            }
        }
        /* DRAIN THE BACKLOG BEFORE THE LAUNCH TEXT: the MSX prints at
         * ~50 µs/byte, so a ~1 KB bar/status backlog takes ~50 ms to
         * consume. The old flow pushed " OK..."+0x03 directly behind
         * that backlog; the 20 ms mapper-swap budget then expired with
         * 610 bytes still unread (observed on the console) and the MSX
         * never actually READ the 0x03 -> after the swap its 0x7FFF
         * polls return open bus and the cart never boots. Flushing
         * here is safe: until 0x03 is pushed the MSX is only printing
         * (its terminal LOOP), it cannot reach ROM_START or touch
         * 0x4000. 600 ms covers the worst-case 2048-byte ring at ~50
         * µs/byte (~102 ms) with margin; normally it returns in tens
         * of ms. */
        term_wait_drain (600U);
        if (g_term_mbox.out_n != 0U) {
            printf ("TERM: WARNING MSX still draining (out_n=%u after "
                    "600 ms) - launch text will queue behind it\r\n",
                    g_term_mbox.out_n);
        }
        out_str (" OK - booting MSX into cart");
        newline ();
        printf ("TERM: about to soft_reset_into_cart mapper=%d "
                "(PSRAM has %u bytes of '%s')\r\n",
                (int)m, (unsigned)got, path);
        soft_reset_into_cart (m);
    }
}

/* ------------------------------------------------------------------ */
/* Service - main-loop pump                                            */
/* ------------------------------------------------------------------ */

static void handle_list_key (uint8_t key) {
    /* arrow up / down / RET / ESC / LEFT/RIGHT (page nav) */
    printf ("TERM: list key=0x%02X sel=%u count=%u page=%u\r\n",
            key, s_menu.sel, s_menu.count, s_menu.page);
    if (key == 0x1E) {                   /* up */
        if (s_menu.sel > 0) s_menu.sel--;
        /* page change if we crossed a page boundary */
        uint16_t new_page = (uint16_t)(s_menu.sel / TERM_PAGE_SIZE);
        if (new_page != s_menu.page) {
            s_menu.page = new_page;
            print_file_list ();
            return;
        }
    } else if (key == 0x1F) {            /* down */
        if (s_menu.sel + 1U < s_menu.count) s_menu.sel++;
        /* page change if we crossed a page boundary */
        uint16_t new_page = (uint16_t)(s_menu.sel / TERM_PAGE_SIZE);
        if (new_page != s_menu.page) {
            s_menu.page = new_page;
            print_file_list ();
            return;
        }
    } else if (key == 0x1D) {            /* LEFT = previous page */
        if (s_menu.page > 0) {
            s_menu.page--;
            /* move sel to top of new page */
            s_menu.sel = (uint16_t)(s_menu.page * TERM_PAGE_SIZE);
            print_file_list ();
            return;
        }
    } else if (key == 0x1C) {            /* RIGHT = next page */
        if (s_menu.page + 1U < menu_page_count ()) {
            s_menu.page++;
            /* move sel to top of new page */
            s_menu.sel = (uint16_t)(s_menu.page * TERM_PAGE_SIZE);
            print_file_list ();
            return;
        }
    } else if (key == 0x0D && s_menu.count > 0U) {
        /* pick this file -> show mapper menu */
        int i = 0;
        const char *n = s_menu.names[s_menu.sel];
        while (n[i] && i < FILE_NAME_MAX - 1) { s_menu.picked[i] = n[i]; i++; }
        s_menu.picked[i] = '\0';
        printf ("TERM: picked file [%s] -> mapper menu\r\n", s_menu.picked);
        /* Remember which file was picked (for the LFN display on the
         * mapper screen) BEFORE s_menu.sel is reused as the
         * mapper-row index. */
        s_menu.file_idx = s_menu.sel;
        /* Reuse s_menu.sel as the mapper-screen highlight index.
         * Reset to 0 so the arrow starts on the first (default)
         * mapper entry. */
        s_menu.sel = 0;
        s_menu.state = MENU_MAPPER;
        print_mapper_menu ();
        return;
    } else if (key == 0x1B) {            /* ESC = rescan */
        printf ("TERM: ESC -> rescan\r\n");
        scan_usb ();
        s_menu.page = 0;
        s_menu.sel   = 0;
        print_file_list ();
        return;
    } else if (key == 'F' || key == 'f') {
        /* User wants the OLD loader (FLASH mapper - serves selector_rom
         * with the existing 0x7FF0 mailbox protocol). */
        printf ("TERM: F -> FLASH loader\r\n");
        clear_screen ();
        out_str (" Switching to OLD loader");
        newline ();
        (void)Cart_SetMapper_Safe (CART_MAP_FLASH);
        return;
    } else if (key == 'N' || key == 'n') {
        /* Boot the flash-served Nextor Sunrise IDE kernel (Carnivore2
         * mapper, IDE window 0x7C00..0x7EFF backed by the USBHS host
         * via sunrise_ide.c). Uses EXACTLY the proven game-launch
         * dance (soft_reset_into_cart): push the launch byte, the
         * MSX-side terminal's rom_start runs rst 0 from MSX RAM, and
         * the BIOS re-probe finds the armed cart - here the Sunrise
         * kernel's 'AB' at 0x4000 (nextor_rom bank 0 of the
         * MSXSoftware/Nextor/nextor_sunrise.bin ROM).
         * Physical F1 cannot be sniffed through the menu's CHGET
         * forwarding (the BIOS turns F1 into its KEY string), so the
         * menu accepts the letter N; the footer hints it. */
        printf ("TERM: N -> SUNRIDE\r\n");
        clear_screen ();
        out_str (" Booting Nextor (Sunrise IDE)...");
        newline ();
        /* Let the MSX print the launch text before the 0x03 byte and
         * the mapper swap (same discipline as Terminal_BootCart).
         * CRITICAL: keep the soft_reset_into_cart sequence exactly
         * as for game ROMs - the Sunrise kernel's init has the same
         * "MSX slot probe 0x4000 within ~20 ms of RST 0" timing, so
         * the 30 ms swap window in soft_reset_into_cart is what
         * avoids the boot hang regression. */
        term_wait_drain (600U);
        soft_reset_into_cart (CART_MAP_SUNRIDE);
        return;
    }
    /* redraw cursor + arrow on the line we landed on (in-page row) */
    uint16_t row_in_page = (uint16_t)(s_menu.sel % TERM_PAGE_SIZE);
    move_pointer (0, (uint8_t)(1 + row_in_page));
}

static void handle_mapper_key (uint8_t key) {
    /* Arrow up/down moves the sprite between the MAPPER_COUNT lines;
     * RET or a single digit 0..9 selects. ESC goes back to the file
     * list. s_menu.sel carries over as the mapper-row index (it was
     * reset to 0 by handle_list_key when we entered this screen). */
    printf ("TERM: mapper key=0x%02X sel=%u\r\n", key, s_menu.sel);
    if (key == 0x1E) {                   /* up */
        if (s_menu.sel > 0) s_menu.sel--;
        move_pointer (0, (uint8_t)(4 + s_menu.sel));
        return;
    }
    if (key == 0x1F) {                   /* down */
        if ((int)s_menu.sel + 1 < MAPPER_COUNT) s_menu.sel++;
        move_pointer (0, (uint8_t)(4 + s_menu.sel));
        return;
    }
    if (key == 0x0D) {                   /* RET - boot the highlighted mapper */
        printf ("TERM: mapper RET -> boot[%u] %s\r\n", s_menu.sel, s_menu.picked);
        Terminal_BootCart ((uint8_t)s_menu.sel, s_menu.picked);
        return;
    }
    if (key == 0x1B) {                   /* ESC = back to file list */
        printf ("TERM: mapper ESC -> file list\r\n");
        s_menu.state = MENU_LIST;
        print_file_list ();
        return;
    }
    if (key >= '0' && key <= '9') {
        int idx = (int)(key - '0');
        if (idx < MAPPER_COUNT) {
            printf ("TERM: mapper digit -> boot[%d] %s\r\n", idx, s_menu.picked);
            s_menu.sel = (uint16_t)idx;
            Terminal_BootCart ((uint8_t)idx, s_menu.picked);
        }
    }
}

void Terminal_Service (void) {
    /* Drop the control byte if the cart wrote one (the IRQ latches
     * it; we consume it once and clear it). */
    uint8_t c = g_term_mbox.control;
    if (c != 0xFFU) {
        g_term_mbox.control = 0xFFU;
        /* Note: the previous version had a "GRPH-skip" branch that
         * switched to CART_MAP_FLASH on control=0x04. v303's MSX-side
         * asm probes GRPH via CALL #141 and writes 0x04 to 0x7FFE
         * when GRPH is held - but CALL #141 is GTSIZE, which is
         * model-dependent (Panasonic MSX2+ returns GRPH bit, some
         * clones return 0x04 always). On affected hardware the
         * terminal would silently switch to the OLD cart loader even
         * when GRPH was not held, with no way to recover except a
         * power cycle. To keep the terminal predictable across MSX
         * models, the GRPH-skip behaviour is removed: the user picks
         * the FLASH loader explicitly from the menu (key F on the
         * file list). */
    }

    /* Drain keyboard FIFO. */
    uint8_t key;
    while (kbd_pop (&key)) {
        if (s_menu.state == MENU_LIST) {
            handle_list_key (key);
        } else {
            handle_mapper_key (key);
        }
    }

    /* On the first service pass of each terminal session, render the
     * file list once - the trigger is s_list_pending (re-armed by
     * Terminal_Reset), not a never-reset static, so sessions entered
     * via the boot gate render correctly too. */
    if (s_list_pending) {
        s_list_pending = 0;
        printf ("TERM: first service - scanning USB\r\n");
        scan_usb ();
        printf ("TERM: scan_usb -> count=%u\r\n", s_menu.count);
        print_file_list ();
    }
}

#pragma GCC pop_options