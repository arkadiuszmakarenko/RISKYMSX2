
#include "terminal.h"
#include "cart.h"
#include "psram.h"
#include "usb_disk.h"
#include "nextor.h"
#include "raw_disk.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>
#include "ch32v4x7.h"

/* Declared here rather than by including Debug/debug.h: that header also
 * drags in the USART bring-up declarations, and terminal.c has no use for
 * them (it writes the MSX screen, not the UART). Same declaration
 * nextor.c uses. */
extern void Delay_Ms (uint32_t n);

#pragma GCC push_options
#pragma GCC optimize("Os")

/* Single shared mailbox instance. cart.c's EXTI0 handler touches the
 * fields directly. */
TerminalMailbox g_term_mbox;

/* FatFs logical drive 0 normally auto-detects a partition. During USB
 * preparation the formatter temporarily pins it to partitions 1..4 so
 * each primary FAT16 volume can be formatted in turn. */
PARTITION VolToPart[FF_VOLUMES] = {{0U, 0U}};

/* Keep a useful multi-page directory listing instead of truncating it after
 * two pages. Each entry costs about 57 bytes across the parallel name/size
 * arrays, so 512 entries use roughly 29 KiB of SRAM and allow 26 pages. */
#define TERM_MAX_FILES   512U
#define FILE_NAME_MAX    26U    /* printed-column width: 25 chars + NUL.
                                 * Line layout = 25 (name) + 1 (space) +
                                 * up to 5 (size, e.g. "1023K") = 31
                                 * printed chars, leaving 1-char margin
                                 * on the MSX's 32-column screen so the
                                 * CR/LF never wraps mid-row. */
#define TERM_PAGE_SIZE   20U    /* rows per page on the file list */

/* Terminal menu build tag, printed once per session on the UART.
 *
 * Why a tag at all: the on-screen footers are the user-facing version
 * marker, and they are 32-column lines that get edited often enough
 * to be ambiguous ("is this row 21 the new one or the old one?").
 * One log line at session start settles it, and it costs one string.
 * Bump this whenever the key map changes. */
#define TERM_MENU_VERSION "v8 (USB manager)"

/* Maximum depth of the current-directory path the menu can hold. The
 * full path assembled at boot time (e.g. "0:/A/B/C/file.ROM") is built
 * out of this same buffer, so 64 bytes is enough for ~10 levels of
 * 8.3-style nesting. f_opendir on this stick refused to open a path
 * deeper than about 80 bytes in practice - keep the budget well below. */
#define TERM_PATH_MAX   64U

/* Display column budgets. The MSX screen is 32 columns and column 0
 * is reserved for the sprite arrow, so the printable width is 31.
 * "25 name + 1 sp + 5 size" = 31, which is what FILE_NAME_MAX already
 * encodes. Directory rows use the full 25 name columns, followed by
 * a two-space gap and the "DIR" marker. */
#define TERM_NAME_COLS  (FILE_NAME_MAX - 1U)   /* 25 */

typedef enum {
    MENU_LIST,
    MENU_MAPPER,
} MenuState;

/* Per-file menu state. */
static struct {
    /* SFN (8.3 short name, e.g. "METALG~1.ROM") - what f_open() gets
     * for every file operation. LFN paths fail f_open with
     * FR_NO_FILE (4) on our stick, so all operations use the SFN.
     *
     * For directory rows this holds the SFN of the directory itself
     * (fi.altname, e.g. "SUB") or the literal string ".." for the
     * parent-entry row. */
    char    names[TERM_MAX_FILES][FILE_NAME_MAX];
    /* LFN (long name, up to 25 printable chars) - DISPLAY ONLY.
     * Drawn in the file list so the menu stays human-readable; never
     * used for f_open / path building.
     *
     * For directory rows the LFN is the same as the SFN unless the
     * volume was formatted with LFN support, in which case the
     * human-readable form is shown. */
    char    lfns[TERM_MAX_FILES][FILE_NAME_MAX];
    uint32_t sizes[TERM_MAX_FILES];  /* bytes - 0 for directory rows */
    uint16_t count;
    uint16_t sel;       /* currently-highlighted index (absolute,
                         * i.e. counts across pages) */
    uint16_t page;      /* current page, 0-based */
    uint16_t file_idx;  /* index of the file picked for the mapper
                         * screen - s_menu.sel is REUSED as the
                         * mapper-row index on that screen, so this
                         * preserves which LFN to display. */
    /* Per-row type flag. 0 = file, 1 = directory (including the
     * synthetic ".. " row at index 0 of every non-root list). The
     * file list reuses the same array for files and dirs, so the
     * type travels with the row rather than with a separate path. */
    uint8_t  is_dir[TERM_MAX_FILES];
    /* Current directory relative to the volume root. The empty
     * string means the root. A trailing '/' is always present when
     * the string is non-empty, so the boot path is built as
     * "0:/" + s_menu.cwd + s_menu.picked without any case-by-case
     * separator logic. Bounded to TERM_PATH_MAX-1 chars. */
    char     cwd[TERM_PATH_MAX];
    uint8_t  loaded;    /* "ROM was loaded into PSRAM" */
    MenuState state;
    char     picked[FILE_NAME_MAX]; /* SFN of the selected file - the
                                 * operational name passed to
                                 * Terminal_BootCart */
} s_menu;

/* ------------------------------------------------------------------ */
/* Menu rendering                                                      */
/* ------------------------------------------------------------------ */
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
    uint16_t tail = g_term_mbox.out_tail;
    uint16_t next = (uint16_t)((tail + 1U) % TERM_OUT_FIFO_DEPTH);
    if (next == g_term_mbox.out_head) return 0;  /* full */
    g_term_mbox.out_buf[tail] = b;
    /* Memory barrier: make sure the byte write above is visible to the
     * IRQ-side consumer (Cart_EXTI0_Terminal_Handler) BEFORE the
     * tail increment that exposes the slot. RV32 needs a store-store
     * fence; the WCH core provides __asm__ fence syntax via the
     * standard RISC-V mnemonics. */
    __asm__ volatile ("fence w, w" ::: "memory");
    g_term_mbox.out_tail = next;
    return 1;
}

/* Occupancy, derived from the SPSC indices rather than a shared counter
 * (see terminal.h). The main loop may observe one byte more than is
 * actually queued if the EXTI0 consumer advances out_head mid-read;
 * every caller treats this as a scheduling hint, so the conservative
 * over-estimate is harmless. */
uint16_t Terminal_OutLevel (void) {
    return (uint16_t)((g_term_mbox.out_tail - g_term_mbox.out_head)
                      & (TERM_OUT_FIFO_DEPTH - 1U));
}

static int kbd_pop (uint8_t *out) {
    if (g_term_mbox.kbd_head == g_term_mbox.kbd_tail) return 0;
    *out = g_term_mbox.kbd_buf[g_term_mbox.kbd_head];
    g_term_mbox.kbd_head = (uint8_t)((g_term_mbox.kbd_head + 1U)
                                     % TERM_KBD_FIFO_DEPTH);
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

/* Spin until the MSX has drained the output FIFO (level == 0) or the
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
        if (Terminal_OutLevel () == 0U) break;
        __asm__ volatile ("nop");
    }
}

/* ------------------------------------------------------------------ */
/* Public mailbox lifecycle                                            */
/* ------------------------------------------------------------------ */

/* Set once per terminal session: the next Terminal_Service pass scans
 * the stick and renders the file list. Terminal_Reset() re-arms it, so
 * a session entered via the boot gate (the MSX-side init reports GRAPH
 * held) renders on its first post-gate pass even though Terminal_Service
 * already ran while the gate was still waiting for that report. */
static uint8_t s_list_pending = 1;

/* Boot gate. At power-on the MSX-side terminal ROM's init probes GRAPH
 * and writes its decision to the control mailbox before it starts
 * polling:
 *   0x00 -> GRAPH held     -> render the terminal menu (this pass).
 *   0x01 -> GRAPH not held -> boot Nextor instead of rendering.
 * Until that byte arrives the main loop must NOT render the menu (else
 * a no-GRAPH power-on flashes the menu for ~40 ms before Nextor boots),
 * so the first render is gated on s_bootgate_done. A timeout keyed off
 * rdcycle is the safety net: if the byte never arrives (e.g. a stale
 * MSX-side ROM that writes nothing) the menu still renders, exactly as
 * the pre-gate firmware did. */
static uint8_t  s_bootgate_done = 0U;
static uint32_t s_bootgate_t0   = 0U;

/* Core-cycle counter, used only for the boot-gate safety timeout. The
 * threshold below is deliberately coarse (it only has to be "a few
 * seconds"), so no Delay_Ms-based calibration is needed: the core clock
 * is a few hundred MHz (400 MHz on this part), and even a 2x error still
 * gives a multi-second fallback well under the ~10 s rdcycle wrap.
 * 2.4e9 cycles ~= 6 s at 400 MHz. Unsigned subtraction makes wrap
 * harmless. */
#define TERM_BOOTGATE_TIMEOUT_CYC 2400000000U

static inline uint32_t term_rdcycle (void) {
    uint32_t c;
    __asm__ volatile ("rdcycle %0" : "=r" (c));
    return c;
}

void Terminal_Reset (void) {
    g_term_mbox.kbd_head = g_term_mbox.kbd_tail = 0;
    g_term_mbox.out_head = g_term_mbox.out_tail = 0;
    g_term_mbox.control = 0xFFU;
    s_menu.count = 0;
    s_menu.sel   = 0;
    s_menu.page  = 0;
    s_menu.file_idx = 0;
    s_menu.loaded = 0;
    s_menu.state  = MENU_LIST;
    s_menu.picked[0] = '\0';
    /* Start at the volume root. A subsequent scan_usb() will list its
     * contents. */
    s_menu.cwd[0] = '\0';
    s_list_pending = 1U;
    s_bootgate_done = 0U;
    s_bootgate_t0   = 0U;
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

/* Progress bar geometry - keep in sync with the header printed by
 * Terminal_BootCart: "  [--------------------]   0%" = 2 spaces + '['
 * + 20 blocks + ']' + 2 spaces + 3 digits + '%' = 30 chars. Shared with
 * the Nextor screen's create progress, which is the same 20-block bar
 * at a different row. */
#define TERM_PROG_BAR_BLOCKS  20U

/* Draw the progress bar in place at (row, col) for a percentage. Both
 * long-running UI paths redraw the same 29-char bar in place rather
 * than scrolling, so a callback must be cheap: 29 out_push calls and
 * nothing else. Callers do their own throttling and backpressure
 * (see term_progress_cb and term_nextor_progress) - this is the
 * drawing primitive, not the policy. */
static void draw_bar (uint8_t row, uint8_t col, uint32_t pct) {
    uint32_t filled;
    char     num[4];
    uint8_t  i;

    if (pct > 100U) pct = 100U;
    filled = (pct * TERM_PROG_BAR_BLOCKS) / 100U;

    move_cursor (row, col);
    out_push (' ');
    out_push (' ');
    out_push ('[');
    for (i = 0U; i < TERM_PROG_BAR_BLOCKS; i++) {
        out_push ((uint8_t)((i < filled) ? '#' : '-'));
    }
    out_push (']');
    out_push (' ');
    /* Percentage, 3 chars zero-padded so the column stays fixed. */
    if (pct >= 100U) { num[0] = '1'; num[1] = '0'; num[2] = '0'; }
    else if (pct >= 10U) { num[0] = (char)('0' + pct / 10U); num[1] = (char)('0' + pct % 10U); num[2] = ' '; }
    else { num[0] = (char)('0' + pct); num[1] = ' '; num[2] = ' '; }
    num[3] = '\0';
    out_str (num);
    out_push ('%');
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

    /* Keep all 20 list rows available on every level. The previous
     * cwd header consumed a row but paging and pointer positioning
     * still assumed 20 visible entries, causing the last row to
     * overlap the footer in subdirectories. */
    const uint8_t body_offset = 1U;
    uint16_t start = (uint16_t)(s_menu.page * TERM_PAGE_SIZE);
    uint16_t end   = (uint16_t)(start + TERM_PAGE_SIZE);
    if (end > s_menu.count) end = s_menu.count;

    for (uint16_t i = start; i < end; i++) {
        /* Reserve column 0 for the sprite cursor - text starts at
         * column 1. */
        move_cursor ((uint8_t)(body_offset + (i - start)), 1);
        /* Two column layouts share the same 25-char name budget:
         *
         *   file:       "<name padded to 25> <size>"
         *   directory:  "<name padded to 25>  DIR"
         *
         * The synthetic "<DIR> .." row uses the same 8 chars at the
         * front as "<DIR> ..", padded to 25 - unambiguous on screen
         * and visibly distinct from an ordinary directory name. */
        const char *n;
        if (s_menu.is_dir[i]) {
            if (s_menu.names[i][0] == '.' && s_menu.names[i][1] == '.' &&
                s_menu.names[i][2] == '\0') {
                /* "<DIR> .." - 8 chars, padded to 25 */
                static const char kUp[] = "<DIR> ..";
                for (int j = 0; j < 8; j++) out_push ((uint8_t)kUp[j]);
                for (int j = 8; j < (int)TERM_NAME_COLS; j++) out_push (' ');
            } else {
                /* Use the full name width; don't prefix it with DIR,
                 * since that clipped long folder names and pushed the
                 * type marker past the right edge of the screen. */
                n = s_menu.lfns[i];
                for (int j = 0; j < (int)TERM_NAME_COLS; j++) {
                    char c = n[j];
                    if (c == '\0') {
                        for (; j < (int)TERM_NAME_COLS; j++) out_push (' ');
                        break;
                    }
                    out_push ((uint8_t)c);
                }
            }
            out_str ("  DIR");
        } else {
            /* file: 25 chars of name, padded, then size */
            n = s_menu.lfns[i];
            for (int j = 0; j < (int)TERM_NAME_COLS; j++) {
                char c = n[j];
                if (c == '\0') {
                    for (; j < (int)TERM_NAME_COLS; j++) out_push (' ');
                    break;
                }
                out_push ((uint8_t)c);
            }
            out_push (' ');
            print_size (s_menu.sizes[i]);
        }
        newline ();
    }
    if (s_menu.count == 0U) {
        move_cursor (body_offset, 1);
        out_str ("  (empty directory)");
        newline ();
    }

    /* Two hint rows at the bottom: row 21 is the page indicator, row 22
     * carries the key map. They are anchored to the SAME physical
     * rows as before (21/22), so the body region is now 20 rows tall
     * when the cwd header is hidden, or 19 rows when it is shown.
     * With TERM_PAGE_SIZE = 20 the unadorned list fills the screen;
     * the cwd line costs one body row on a deep list, which is the
     * only honest answer to the user's "where am I?" question. */
    char buf[32];
    move_cursor (21, 1);
    snprintf (buf, sizeof (buf), " Pg %u/%u",
              (unsigned)(s_menu.page + 1U), (unsigned)menu_page_count ());
    out_str (buf);
    newline ();
    move_cursor (22, 1);
    out_str ("N=Nextor B=USB ESC=back");
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

/* Copy a NUL-terminated string into a fixed-size buffer. Truncates
 * silently if the source is longer than the buffer; the destination
 * is always NUL-terminated as long as `cap > 0`. Same semantics as
 * snprintf(src, cap, "%s", s) but smaller and avoids pulling in the
 * full printf machinery for two hot callers. */
static void term_str_copy (char *dst, const char *src, size_t cap) {
    size_t k = 0;
    if (cap == 0U) return;
    while (src[k] != '\0' && k + 1U < cap) {
        dst[k] = src[k];
        k++;
    }
    dst[k] = '\0';
}

/* Build "0:/<cwd><sfx>" into `out` (NUL-terminated, bounded by
 * `out_cap`). Always emits the volume prefix; the cwd (if any) is
 * inserted with its trailing '/'. The result is the absolute path
 * that f_open / f_opendir accept on this volume, and is what
 * Terminal_BootCart used to assemble by hand out of "0:/" + name.
 *
 * Truncation is reported by returning 0 on overflow; the caller is
 * expected to surface that to the UART and refuse the operation, not
 * try to f_open a path that does not fit in our buffer. The LFN
 * truncation in scan_usb() keeps SFNs within 12 chars by design, so
 * the only realistic overflow source is a deeply nested cwd, and
 * the boot caller's own path check in Terminal_BootCart already
 * guarantees 40 bytes there. */
static int term_build_path (char *out, size_t out_cap, const char *sfx) {
    /* "0:/" + cwd + sfx + NUL */
    const char *prefix = "0:/";
    size_t p = 0;
    while (prefix[p] != '\0') {
        if (p + 1U >= out_cap) return 0;
        out[p] = prefix[p];
        p++;
    }
    size_t i = 0;
    while (s_menu.cwd[i] != '\0') {
        if (p + 1U >= out_cap) return 0;
        out[p++] = s_menu.cwd[i++];
    }
    size_t j = 0;
    while (sfx[j] != '\0') {
        if (p + 1U >= out_cap) return 0;
        out[p++] = sfx[j++];
    }
    out[p] = '\0';
    return 1;
}

static uint8_t term_fold_ascii (uint8_t c) {
    if (c >= (uint8_t)'A' && c <= (uint8_t)'Z') {
        c = (uint8_t)(c + ((uint8_t)'a' - (uint8_t)'A'));
    }
    return c;
}

static int term_name_compare (const char *left, const char *right) {
    while (*left != '\0' && *right != '\0') {
        const uint8_t a = term_fold_ascii ((uint8_t)*left++);
        const uint8_t b = term_fold_ascii ((uint8_t)*right++);
        if (a != b) return (a < b) ? -1 : 1;
    }
    if (*left == *right) return 0;
    return (*left == '\0') ? -1 : 1;
}

/* Directories sort before files; within each group use the displayed LFN
 * case-insensitively. The SFN breaks ties consistently when two displayed
 * names truncate to the same text. */
static int menu_entry_compare (uint16_t left, uint16_t right) {
    if (s_menu.is_dir[left] != s_menu.is_dir[right]) {
        return s_menu.is_dir[left] ? -1 : 1;
    }
    {
        const int lfn_order = term_name_compare (s_menu.lfns[left],
                                                s_menu.lfns[right]);
        if (lfn_order != 0) return lfn_order;
    }
    return term_name_compare (s_menu.names[left], s_menu.names[right]);
}

static void menu_entry_swap (uint16_t left, uint16_t right) {
    char name_tmp[FILE_NAME_MAX];
    char lfn_tmp[FILE_NAME_MAX];
    uint32_t size_tmp;
    uint8_t dir_tmp;

    memcpy (name_tmp, s_menu.names[left], sizeof (name_tmp));
    memcpy (s_menu.names[left], s_menu.names[right], sizeof (name_tmp));
    memcpy (s_menu.names[right], name_tmp, sizeof (name_tmp));
    memcpy (lfn_tmp, s_menu.lfns[left], sizeof (lfn_tmp));
    memcpy (s_menu.lfns[left], s_menu.lfns[right], sizeof (lfn_tmp));
    memcpy (s_menu.lfns[right], lfn_tmp, sizeof (lfn_tmp));

    size_tmp = s_menu.sizes[left];
    s_menu.sizes[left] = s_menu.sizes[right];
    s_menu.sizes[right] = size_tmp;
    dir_tmp = s_menu.is_dir[left];
    s_menu.is_dir[left] = s_menu.is_dir[right];
    s_menu.is_dir[right] = dir_tmp;
}

/* Shell sort the collected directory entries. Keep the synthetic parent
 * row fixed at index 0 so it remains the first entry on every subfolder. */
static void sort_menu_entries (void) {
    const uint16_t first = (s_menu.cwd[0] != '\0') ? 1U : 0U;
    uint16_t gap = (uint16_t)((s_menu.count - first) / 2U);

    while (gap > 0U) {
        for (uint16_t i = (uint16_t)(first + gap); i < s_menu.count; i++) {
            uint16_t j = i;
            while (j >= (uint16_t)(first + gap) &&
                   menu_entry_compare ((uint16_t)(j - gap), j) > 0) {
                menu_entry_swap ((uint16_t)(j - gap), j);
                j = (uint16_t)(j - gap);
            }
        }
        gap = (uint16_t)(gap / 2U);
    }
}

static void scan_usb (void) {
    s_menu.count = 0;
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        printf ("TERM: scan_usb - USB not mounted\r\n");
        return;
    }
    /* Add the parent entry before opening the child directory. If the
     * child disappeared or cannot be read, the user can still select
     * this row to return to its parent. */
    if (s_menu.cwd[0] != '\0') {
        term_str_copy (s_menu.names[0], "..", FILE_NAME_MAX);
        term_str_copy (s_menu.lfns[0],  "..", FILE_NAME_MAX);
        s_menu.sizes[0] = 0U;
        s_menu.is_dir[0] = 1U;
        s_menu.count = 1U;
    }
    /* Open the current directory rather than always the root. The
     * path is built from s_menu.cwd; the empty-cwd case yields the
     * volume root. */
    char path[TERM_PATH_MAX + 4U];   /* "0:/" + cwd (63) + NUL = 68 */
    if (!term_build_path (path, sizeof (path), "")) {
        printf ("TERM: scan_usb - cwd path too long ([%s])\r\n", s_menu.cwd);
        return;
    }
    DIR dir; FILINFO fi;
    FRESULT fr = f_opendir (&dir, path);
    if (fr != FR_OK) {
        printf ("TERM: scan_usb - f_opendir([%s]) failed (%u)\r\n",
                path, (unsigned)fr);
        return;
    }

    /* Read the directory twice: first collect directories, then files.
     * This keeps folders together at the top regardless of the order
     * FatFS returns entries in, with the synthetic parent row above
     * them. */
    for (uint8_t pass = 0U; pass < 2U && s_menu.count < TERM_MAX_FILES; pass++) {
        if (pass != 0U) {
            fr = f_rewinddir (&dir);
            if (fr != FR_OK) break;
        }
        while (s_menu.count < TERM_MAX_FILES) {
            fr = f_readdir (&dir, &fi);
            /* End-of-directory check MUST use fi.fname - FatFS only
             * clears fname[0] on end-of-dir, leaving altname holding a
             * stale copy of the previous entry's SFN. */
            if (fr != FR_OK || fi.fname[0] == 0) break;
            /* Skip "." (the directory itself) - the synthetic parent
             * row is the only way to leave a non-root directory. */
            if (fi.fname[0] == '.' && fi.fname[1] == '\0') continue;
            if (fi.fattrib & AM_HID) continue;
            if (fi.fattrib & AM_SYS) continue;
            /* Pass zero is directories; pass one is files. */
            if (((fi.fattrib & AM_DIR) != 0U) != (pass == 0U)) continue;

        /* Store BOTH name forms:
         *   - s_menu.names[] = SFN (fi.altname, 8.3) - used for ALL
         *     file operations (f_open in Terminal_BootCart). LFN
         *     paths fail f_open with FR_NO_FILE (4) on our stick.
         *   - s_menu.lfns[]  = LFN (fi.fname) - display only, so the
         *     menu shows "Metal Gear 2 - Solid Snake" instead of
         *     METALG~1.ROM. Truncated to the printable width with a
         *     trailing '.'-style ellipsis when longer.
         *
         * Directory rows reuse the same two columns. The SFN is what
         * gets passed to term_build_path() when descending, and the
         * SFN is always 8.3 for FAT volumes - so directory navigation
         * is robust regardless of LFN formatting on the stick. */
        const char *sf = fi.altname[0] ? fi.altname : fi.fname;
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
        /* Files: if the LFN is longer than the display width, keep
         * the extension readable: show first (width-4) chars + "~X"
         * style ellipsis + extension. Simplest robust rule: keep
         * the last '.' and truncate the stem.
         *
         * Directories: no extension to preserve, so show the LFN up
         * to the full 25-character name-column width. The SFN fallback
         * covers directory navigation independently of this display
         * name. */
        if (fi.fattrib & AM_DIR) {
            if (ln[k] != '\0') {
                /* LFN was truncated at FILE_NAME_MAX-1; just force a
                 * NUL where the truncation happened and move on. */
                s_menu.lfns[s_menu.count][k] = '\0';
            }
            s_menu.sizes[s_menu.count] = 0U;
            s_menu.is_dir[s_menu.count] = 1U;
            printf ("TERM:   dir [%u] SFN=[%s] LFN=[%s]\r\n",
                    s_menu.count, s_menu.names[s_menu.count], fi.fname);
        } else {
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
            s_menu.is_dir[s_menu.count] = 0U;
            printf ("TERM:   file[%u] SFN=[%s] LFN=[%s] sz=%u\r\n",
                    s_menu.count, s_menu.names[s_menu.count], fi.fname, (unsigned)fi.fsize);
        }
            s_menu.count++;
        }
    }
    f_closedir (&dir);
    sort_menu_entries ();
}

/* ------------------------------------------------------------------ */
/* USB drive management                                                */
/* ------------------------------------------------------------------ */
/* The stick is the raw disk exposed to Nextor. The terminal can create
 * up to four primary FAT16 partitions, format them, and mark each active
 * so Nextor 3 can assign a drive to each one at boot. Preparation replaces
 * the partition table and FAT metadata; it is explicitly confirmed in
 * the UI. Files are not securely overwritten, so this is a format, not
 * a sanitization tool. */

#define MBR_SECT        512U
#define MBR_PT_OFF      446U    /* first of the four partition entries */
#define MBR_PT_STRIDE   16U
#define MBR_PT_MAX      4U
#define MBR_SIG_OFF     510U

/* Static sector scratch: disk setup and verification share one sector
 * buffer rather than placing 512-byte sectors on the embedded stack. */
static uint8_t s_sect[MBR_SECT];

/* What sector 0 says about the stick. */
#define MBRST_VOLUME    0U  /* no partition table: a filesystem boot sector */
#define MBRST_NO_PART   1U  /* a partition table, but no usable active entry */
#define MBRST_READY     2U  /* a partition table with an active partition */

static uint16_t mbr_get16 (const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t mbr_get32 (const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* --- What sector 0 is.
 *
 * The signature alone does not tell a partition table from a filesystem:
 * a FAT boot sector ends in 55AA too, and always has. What separates
 * them is the first instruction. A real filesystem begins with the
 * mandatory `jmp short` (EB xx) or the 3-byte `jmp` (E9), because the
 * jump instruction has to be a fixed size for the loader to know how far
 * to skip the BPB. A partition table has no boot code at all in a
 * stick that is not meant to chain-load, so its first byte is neither.
 * This is the same test fdisk and mtools use.
 */
static uint8_t mbr_classify (const uint8_t *s) {
    if (s[MBR_SIG_OFF] != 0x55U || s[MBR_SIG_OFF + 1U] != 0xAAU) {
        return MBRST_VOLUME;
    }
    if (s[0] == 0xEBU || s[0] == 0xE9U) return MBRST_VOLUME;

    for (uint8_t i = 0U; i < MBR_PT_MAX; i++) {
        const uint8_t *e = s + MBR_PT_OFF + (size_t)i * MBR_PT_STRIDE;
        if (e[4] == 0U) continue;               /* empty entry              */
        if ((e[0] & 0x80U) == 0U) continue;     /* not flagged bootable      */
        if (mbr_get32(&e[8]) == 0U) continue;    /* starts at LBA 0: overlaps
                                                 * the MBR itself           */
        if (mbr_get32(&e[12]) == 0U) continue;  /* zero length              */
        return MBRST_READY;
    }
    return MBRST_NO_PART;
}

/* --- The MBR partition type for the filesystem the BPB describes, or 0
 *     if the BPB does not describe something we can partition.
 *
 * The type byte is what a PC's partition chooser (and Windows, and Linux)
 * reads to decide which driver to mount with, so picking it from the
 * volume's own cluster count is the difference between a stick that
 * mounts everywhere and one that needs a manual "specify filesystem"
 * prompt on another machine. The cluster count has to be derived - the
 * boot sector does not store it - and derived exactly as the FAT
 * specification defines it, because the FAT12/FAT16 boundary at 4085 is
 * where drivers change.
 *
 * Note this reads the FAT count (offset 16) and not a hard-coded two: a
 * volume formatted with three FATs, which some tools still do, has to be
 * measured with three or the derived cluster count is wrong.
 */
static uint8_t mbr_fattype (const uint8_t *s) {
    uint16_t bps, rsvd, spf, root, tot16;
    uint8_t  spc, nfat;
    uint32_t tot, data, clusters;

    bps = mbr_get16(&s[11]);
    if (bps != 512U) return 0U;                 /* the whole mailbox is 512 */
    spc = s[13];
    if (spc == 0U) return 0U;
    rsvd  = mbr_get16(&s[14]);
    nfat  = s[16];
    spf   = mbr_get16(&s[22]);
    root  = mbr_get16(&s[17]);
    tot16 = mbr_get16(&s[19]);
    if (nfat == 0U || spf == 0U) return 0U;

    /* tot16 is authoritative when set and zero for FAT32 by definition. */
    tot = (tot16 != 0U) ? (uint32_t)tot16 : mbr_get32(&s[32]);
    if (tot == 0U) return 0U;

    data = (uint32_t)rsvd + (uint32_t)nfat * (uint32_t)spf
         + ((uint32_t)root * 32U) / (uint32_t)bps;
    if (tot <= data) return 0U;                 /* more metadata than volume */
    clusters = (tot - data) / (uint32_t)spc;

    if (clusters < 4085U)  return 0x01U;       /* FAT12 */
    if (clusters < 65525U) return 0x06U;       /* FAT16 */
    return 0x0BU;                              /* FAT32 */
}

/* --- Wait for one keypress, up to `ms`. Returns 0xFF on timeout.
 *
 * Safe to block on: the key FIFO is filled by an EXTI interrupt on this
 * side and the output ring is drained by an interrupt on the MSX side, so
 * both keep moving while this spins. Nothing on the RISC-V needs the main
 * loop to be inside Terminal_Service for either to happen.
 */
static uint8_t term_wait_key (uint32_t ms) {
    uint32_t waited = 0U;
    for (;;) {
        uint8_t k;
        if (kbd_pop (&k)) return k;
        if (waited >= ms) return 0xFFU;
        Delay_Ms (10U);
        waited += 10U;
    }
}

/* FAT16 partitions are deliberately capped at 2 GiB (4,194,304 sectors
 * of 512 bytes) so cluster sizes remain compatible with DOS variants.
 * The prep wizard creates at most four primary partitions; any extra
 * capacity is left untouched/unpartitioned. */
#define TERM_FAT16_PART_SECTORS  4194304UL
#define TERM_FAT16_MIN_SECTORS   65536UL  /* 32 MiB, safely above FAT12 range */

static int mbr_entry_valid (const uint8_t *e, uint32_t total) {
    uint32_t first = mbr_get32 (&e[8]);
    uint32_t count = mbr_get32 (&e[12]);
    if (e[4] == 0U || first == 0U || count == 0U || first >= total) return 0;
    if (count > total - first) return 0;
    return 1;
}

static const char *mbr_type_name (uint8_t type) {
    if (type == 0x01U) return "FAT12";
    if (type == 0x04U || type == 0x06U || type == 0x0EU) return "FAT16";
    if (type == 0x0BU || type == 0x0CU) return "FAT32";
    return "OTHER";
}

static int mbr_is_fat16 (uint8_t type) {
    return type == 0x04U || type == 0x06U || type == 0x0EU;
}

static void manager_pause (const char *message) {
    clear_screen ();
    out_str (" USB drive manager");
    newline ();
    newline ();
    out_str (message);
    newline ();
    newline ();
    out_str (" Press any key");
    newline ();
    term_wait_drain (200U);
    (void)term_wait_key (15000U);
}

static uint8_t manager_mark_active (uint8_t part, uint32_t total) {
    uint8_t *e;
    if (part >= MBR_PT_MAX || RawDisk_ReadSectors (0U, 1U, s_sect) == 0U)
        return 0U;
    if (s_sect[MBR_SIG_OFF] != 0x55U || s_sect[MBR_SIG_OFF + 1U] != 0xAAU)
        return 0U;
    e = s_sect + MBR_PT_OFF + (size_t)part * MBR_PT_STRIDE;
    if (!mbr_entry_valid (e, total) || !mbr_is_fat16 (e[4])) return 0U;
    /* Set this partition active without clearing any other active flags;
     * Nextor 3 can automap one drive per active FAT partition. */
    s_sect[MBR_PT_OFF + (size_t)part * MBR_PT_STRIDE] = 0x80U;
    if (RawDisk_WriteSectors (0U, 1U, s_sect) == 0U ||
        RawDisk_ReadSectors (0U, 1U, s_sect) == 0U ||
        s_sect[MBR_SIG_OFF] != 0x55U || s_sect[MBR_SIG_OFF + 1U] != 0xAAU)
        return 0U;
    e = s_sect + MBR_PT_OFF + (size_t)part * MBR_PT_STRIDE;
    return (uint8_t)(e[0] == 0x80U && mbr_entry_valid (e, total));
}

/* Mark every valid primary FAT16 partition active, preserving the rest of
 * the MBR. Returns the number verified after writing, or zero on failure. */
static uint8_t manager_mark_all_active (uint32_t total) {
    uint8_t count = 0U;

    if (RawDisk_ReadSectors (0U, 1U, s_sect) == 0U ||
        s_sect[MBR_SIG_OFF] != 0x55U || s_sect[MBR_SIG_OFF + 1U] != 0xAAU)
        return 0U;

    for (uint8_t i = 0U; i < MBR_PT_MAX; i++) {
        uint8_t *e = s_sect + MBR_PT_OFF + (size_t)i * MBR_PT_STRIDE;
        if (mbr_entry_valid (e, total) && mbr_is_fat16 (e[4])) {
            e[0] = 0x80U;
            count++;
        }
    }
    if (count == 0U || RawDisk_WriteSectors (0U, 1U, s_sect) == 0U ||
        RawDisk_ReadSectors (0U, 1U, s_sect) == 0U ||
        s_sect[MBR_SIG_OFF] != 0x55U || s_sect[MBR_SIG_OFF + 1U] != 0xAAU)
        return 0U;

    count = 0U;
    for (uint8_t i = 0U; i < MBR_PT_MAX; i++) {
        const uint8_t *e = s_sect + MBR_PT_OFF + (size_t)i * MBR_PT_STRIDE;
        if (mbr_entry_valid (e, total) && mbr_is_fat16 (e[4])) {
            if (e[0] != 0x80U) return 0U;
            count++;
        }
    }
    return count;
}

static FRESULT manager_prepare_fat16 (uint32_t total) {
    LBA_t parts[MBR_PT_MAX] = {0, 0, 0, 0};
    /* Let FatFs choose the cluster size from the partition capacity. It
     * selects 32 KiB clusters at 2 GiB and smaller clusters for a short
     * final partition, while FM_FAT keeps the filesystem out of FAT32. */
    MKFS_PARM fmt = {FM_FAT, 2U, 0U, 512U, 0U};
    uint32_t remaining = (total > 63U) ? total - 63U : 0U;
    uint8_t count = 0U;
    FRESULT fr;

    for (uint8_t i = 0U; i < MBR_PT_MAX &&
         remaining >= TERM_FAT16_MIN_SECTORS; i++) {
        uint32_t n = (remaining > TERM_FAT16_PART_SECTORS)
                   ? TERM_FAT16_PART_SECTORS : remaining;
        parts[i] = (LBA_t)n;
        remaining -= n;
        count++;
    }
    if (count == 0U) return FR_MKFS_ABORTED;
    if (RawDisk_Probe () == 0U || RawDisk_IsPresent () == 0U ||
        RawDisk_SectorCount () != total) return FR_NOT_READY;

    /* Drop FatFs's cached view before changing the partition table. */
    USB_InvalidateMount ();
    VolToPart[0].pd = 0U;
    VolToPart[0].pt = 0U;
    fr = f_fdisk (0U, parts, s_sect);
    if (fr != FR_OK) goto done;

    for (uint8_t i = 0U; i < count; i++) {
        char line[32];
        snprintf (line, sizeof (line), " Formatting FAT16 %u/%u",
                  (unsigned)(i + 1U), (unsigned)count);
        clear_screen ();
        out_str (" Preparing USB drive");
        newline ();
        out_str (line);
        newline ();
        term_wait_drain (200U);

        VolToPart[0].pt = (BYTE)(i + 1U);
        fr = f_mkfs ("0:", &fmt, s_sect, sizeof (s_sect));
        if (fr != FR_OK) goto done;
        if (RawDisk_ReadSectors (0U, 1U, s_sect) == 0U ||
            !mbr_is_fat16 (s_sect[MBR_PT_OFF + (size_t)i * MBR_PT_STRIDE + 4U])) {
            fr = FR_MKFS_ABORTED;
            goto done;
        }
    }

    /* f_fdisk/f_mkfs create the table and FAT volumes but leave all
     * partitions inactive. Mark every created FAT16 partition active so
     * Nextor 3 can allocate one drive per partition, then verify the MBR. */
    VolToPart[0].pt = 0U;
    if (RawDisk_ReadSectors (0U, 1U, s_sect) == 0U ||
        s_sect[MBR_SIG_OFF] != 0x55U || s_sect[MBR_SIG_OFF + 1U] != 0xAAU) {
        fr = FR_DISK_ERR;
        goto done;
    }
    for (uint8_t i = 0U; i < MBR_PT_MAX; i++) {
        uint8_t *e = s_sect + MBR_PT_OFF + (size_t)i * MBR_PT_STRIDE;
        if (i < count &&
            (!mbr_entry_valid (e, total) || !mbr_is_fat16 (e[4]))) {
            fr = FR_DISK_ERR;
            goto done;
        }
        e[0] = (i < count) ? 0x80U : 0U;
    }
    if (RawDisk_WriteSectors (0U, 1U, s_sect) == 0U ||
        RawDisk_ReadSectors (0U, 1U, s_sect) == 0U ||
        mbr_classify (s_sect) != MBRST_READY) {
        fr = FR_DISK_ERR;
        goto done;
    }
    for (uint8_t i = 0U; i < count; i++) {
        const uint8_t *e = s_sect + MBR_PT_OFF + (size_t)i * MBR_PT_STRIDE;
        if (e[0] != 0x80U || !mbr_entry_valid (e, total) ||
            !mbr_is_fat16 (e[4])) {
            fr = FR_DISK_ERR;
            goto done;
        }
    }

done:
    VolToPart[0].pd = 0U;
    VolToPart[0].pt = 0U;
    if (USB_TryEnsureMounted () != DEF_SUCCESS && fr == FR_OK) {
        fr = FR_NO_FILESYSTEM;
    }
    return fr;
}

static void manager_show (uint8_t present, uint32_t total) {
    char buf[32];
    uint8_t sector_ok;
    clear_screen ();
    out_str (" USB drive manager");
    newline ();
    if (!present) {
        out_str (" No readable USB drive");
        newline ();
    } else {
        snprintf (buf, sizeof (buf), " Stick: %u MB",
                  (unsigned)(total / 2048U));
        out_str (buf);
        newline ();
        out_str (" Partitions (* = active)");
        newline ();
        sector_ok = RawDisk_ReadSectors (0U, 1U, s_sect);
        if (!sector_ok) {
            out_str (" Cannot read partition table");
            newline ();
        } else if (mbr_classify (s_sect) != MBRST_VOLUME) {
            for (uint8_t i = 0U; i < MBR_PT_MAX; i++) {
                const uint8_t *e = s_sect + MBR_PT_OFF + (size_t)i * MBR_PT_STRIDE;
                if (e[4] == 0U) {
                    snprintf (buf, sizeof (buf), " %u: (empty)", (unsigned)(i + 1U));
                } else if (!mbr_entry_valid (e, total)) {
                    snprintf (buf, sizeof (buf), " %u: invalid", (unsigned)(i + 1U));
                } else {
                    snprintf (buf, sizeof (buf), " %u%c %-5s %uMB",
                              (unsigned)(i + 1U), e[0] == 0x80U ? '*' : ' ',
                              mbr_type_name (e[4]),
                              (unsigned)(mbr_get32 (&e[12]) / 2048U));
                }
                move_cursor ((uint8_t)(3U + i), 1U);
                out_str (buf);
                newline ();
            }
        } else {
            uint8_t type = mbr_fattype (s_sect);
            out_str (type ? " No MBR; whole-disk FAT volume" : " No MBR/FAT volume found");
            newline ();
        }
    }
    move_cursor (18U, 1U);
    out_str ("P=prepare FAT16 (ERASES USB)");
    newline ();
    move_cursor (19U, 1U);
    out_str ("A=activate FAT16 partition");
    newline ();
    move_cursor (20U, 1U);
    out_str ("M=activate all FAT16");
    newline ();
    move_cursor (21U, 1U);
    out_str ("ESC=return");
    newline ();
}

static int manager_confirm_prepare (uint32_t total) {
    uint8_t k;
    clear_screen ();
    out_str (" PREPARE USB AS FAT16?");
    newline ();
    newline ();
    out_str (" THIS ERASES ALL DATA");
    newline ();
    out_str (" and replaces the table.");
    newline ();
    newline ();
    snprintf ((char *)s_sect, sizeof (s_sect), " Stick: %u MB",
              (unsigned)(total / 2048U));
    out_str ((const char *)s_sect);
    newline ();
    out_str (" Up to 4 x 2GB FAT16.");
    newline ();
    out_str (" All created partitions active.");
    newline ();
    out_str (" Nextor can assign A-D.");
    newline ();
    newline ();
    out_str (" Confirm? Y/N");
    newline ();
    term_wait_drain (200U);
    k = term_wait_key (15000U);
    return (k == 'Y' || k == 'y');
}

static void manager_activate_partition (uint32_t total) {
    uint8_t k, part;
    uint8_t *e;
    char buf[32];

    if (RawDisk_ReadSectors (0U, 1U, s_sect) == 0U ||
        mbr_classify (s_sect) == MBRST_VOLUME) {
        manager_pause (" No MBR partition table");
        return;
    }
    clear_screen ();
    out_str (" Activate which FAT16?");
    newline ();
    for (uint8_t i = 0U; i < MBR_PT_MAX; i++) {
        e = s_sect + MBR_PT_OFF + (size_t)i * MBR_PT_STRIDE;
        if (mbr_entry_valid (e, total) && mbr_is_fat16 (e[4])) {
            snprintf (buf, sizeof (buf), " %u%c FAT16 %uMB",
                      (unsigned)(i + 1U), e[0] == 0x80U ? '*' : ' ',
                      (unsigned)(mbr_get32 (&e[12]) / 2048U));
        } else {
            snprintf (buf, sizeof (buf), " %u: not FAT16", (unsigned)(i + 1U));
        }
        move_cursor ((uint8_t)(2U + i), 1U);
        out_str (buf);
        newline ();
    }
    move_cursor (8U, 1U);
    out_str ("Press 1-4, ESC=cancel");
    newline ();
    term_wait_drain (200U);
    k = term_wait_key (15000U);
    if (k < '1' || k > '4') return;
    part = (uint8_t)(k - '1');
    e = s_sect + MBR_PT_OFF + (size_t)part * MBR_PT_STRIDE;
    if (!mbr_entry_valid (e, total) || !mbr_is_fat16 (e[4])) {
        manager_pause (" Selected partition is not FAT16");
        return;
    }
    clear_screen ();
    snprintf (buf, sizeof (buf), " Set partition %u active?",
              (unsigned)(part + 1U));
    out_str (buf);
    newline ();
    newline ();
    out_str (" This adds a boot choice.");
    newline ();
    out_str (" Other active flags stay set.");
    newline ();
    out_str (" Filesystem data is kept.");
    newline ();
    newline ();
    out_str (" Confirm? Y/N");
    newline ();
    term_wait_drain (200U);
    k = term_wait_key (15000U);
    if (k != 'Y' && k != 'y') return;

    if (RawDisk_Probe () == 0U || RawDisk_IsPresent () == 0U ||
        RawDisk_SectorCount () != total) {
        manager_pause (" USB drive changed; cancelled");
        return;
    }
    USB_InvalidateMount ();
    if (!manager_mark_active (part, total)) {
        (void)USB_TryEnsureMounted ();
        manager_pause (" Active flag write failed");
        return;
    }
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        manager_pause (" Active set; FAT mount failed");
    } else {
        manager_pause (" FAT16 partition is active");
    }
}

static void manager_activate_all_partitions (uint32_t total) {
    uint8_t k, count;
    char message[32];

    clear_screen ();
    out_str (" ACTIVATE ALL FAT16?");
    newline ();
    newline ();
    out_str (" All valid primary FAT16");
    newline ();
    out_str (" partitions become active.");
    newline ();
    out_str (" Filesystem data is kept.");
    newline ();
    out_str (" Nextor may assign one drive");
    newline ();
    out_str (" per partition at reboot.");
    newline ();
    newline ();
    out_str (" Confirm? Y/N");
    newline ();
    term_wait_drain (200U);
    k = term_wait_key (15000U);
    if (k != 'Y' && k != 'y') return;

    if (RawDisk_Probe () == 0U || RawDisk_IsPresent () == 0U ||
        RawDisk_SectorCount () != total) {
        manager_pause (" USB drive changed; cancelled");
        return;
    }
    USB_InvalidateMount ();
    count = manager_mark_all_active (total);
    if (count == 0U) {
        (void)USB_TryEnsureMounted ();
        manager_pause (" No FAT16 partitions or write failed");
        return;
    }
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        manager_pause (" Active flags set; FAT mount failed");
    } else {
        snprintf (message, sizeof (message), " %u FAT16 partitions active",
                  (unsigned)count);
        manager_pause (message);
    }
}

/* B opens a small on-device drive-preparation wizard. Raw-sector and
 * FatFs work only runs here while the terminal mapper owns the cartridge;
 * the Nextor mailbox cannot have an I/O request in flight. */
static void usb_drive_manager (void) {
    uint32_t total = 0U;
    uint8_t present = (uint8_t)(RawDisk_Probe () != 0U && RawDisk_IsPresent () != 0U);
    uint8_t key;
    if (present) total = RawDisk_SectorCount ();

    for (;;) {
        manager_show (present, total);
        term_wait_drain (100U);
        key = term_wait_key (15000U);
        if (key == 0x1BU) return;
        if (key == 'P' || key == 'p') {
            FRESULT fr;
            if (!present) {
                manager_pause (" No USB drive to prepare");
                continue;
            }
            if (!manager_confirm_prepare (total)) {
                manager_pause (" Cancelled; drive unchanged");
                continue;
            }
            fr = manager_prepare_fat16 (total);
            if (fr == FR_OK) {
                manager_pause (" Copy DOS files to partition 1");
            } else {
                char msg[32];
                snprintf (msg, sizeof (msg), " Prepare failed (FatFs %u)",
                          (unsigned)fr);
                manager_pause (msg);
            }
        } else if ((key == 'A' || key == 'a') && present) {
            manager_activate_partition (total);
        } else if ((key == 'M' || key == 'm') && present) {
            manager_activate_all_partitions (total);
        }
        /* Refresh capacity/presence after every action, including a
         * format which may have caused the USB device to reset. */
        present = (uint8_t)(RawDisk_Probe () != 0U && RawDisk_IsPresent () != 0U);
        total = present ? RawDisk_SectorCount () : 0U;
    }
}

/* ------------------------------------------------------------------ */
/* Boot path                                                            */
/* ------------------------------------------------------------------ */

/* Redraws the progress bar in place on the fixed screen row the
 * header occupied. Called from USB_FileToPSRAM's read loop via the
 * USB_ProgressCB hook - keep it short. */
static void term_progress_cb (uint32_t done, uint32_t total) {
    static uint32_t s_last_pct = 0xFFFFU;
    static int s_bar_dbg = 0;

    if (total == 0U) return;
    /* Throttle: only redraw when the percentage changes. */
    uint32_t pct = (uint32_t)(((uint64_t)done * 100ULL) / total);
    if (pct == s_last_pct) return;
    s_last_pct = pct;

    /* DIAGNOSTIC: first few percentage changes print to the UART. If
     * the load freezes mid-copy, the last "bar N%" line brackets which
     * chunk the freeze happened in (1% of 16 KiB = every ~512 B for
     * tiny ROMs, i.e. every chunk). */
    if (s_bar_dbg < 8) {
        printf ("TERM: bar %u%%\r\n", (unsigned)pct);
        s_bar_dbg++;
    }

    /* Backpressure: the ring is only 2048 bytes and the MSX drains at
     * ~50 µs/byte, so a bar redraw every 1% can outrun it. If the
     * queue is high, SKIP this redraw - a skipped cosmetic update is
     * harmless, but out_push() silently drops bytes when full, and
     * dropping the trailing " OK - booting MSX into cart" + 0x03 launch
     * byte (see soft_reset_into_cart) leaves the MSX in the terminal
     * loop and the cart never boots. Threading the ring below ~50%
     * always leaves room for the launch sequence. */
    if (Terminal_OutLevel () > 1024U) return;

    /* Row 3, column 0 - must match the header layout in
     * Terminal_BootCart. */
    draw_bar (3U, 0U, pct);
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
        if (Terminal_OutLevel () != 0U) {
            printf ("TERM: WARNING FIFO not drained (level=%u at swap "
                    "time) - MSX has not read the 0x03 yet, it may "
                    "show rubbish\r\n", (unsigned)Terminal_OutLevel ());
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
    printf ("TERM: BootCart start mapper=%u (%s) file=[%s] cwd=[%s]\r\n",
            mapper_idx, kMapperNames[mapper_idx], filename, s_menu.cwd);

    /* Build "0:/<cwd><filename>" using the shared helper. The path
     * has to be assembled from the CURRENT directory, not the
     * volume root, because the user may have descended several
     * levels and the filename the caller holds is just the leaf
     * (s_menu.picked) - the cwd is what gives the path context.
     *
     * The 40-byte budget the original code used is no longer enough
     * for a deeply nested cwd; size the buffer at TERM_PATH_MAX plus
     * 4 ("0:/" + cwd + filename + NUL, with the filename bounded by
     * FILE_NAME_MAX). The 64-byte cwd ceiling set in scan_usb()
     * means a worst-case total of 4 + 63 + 25 = 92 bytes plus NUL,
     * which is what the buffer holds. */
    char path[TERM_PATH_MAX + FILE_NAME_MAX + 4U];
    if (!term_build_path (path, sizeof (path), filename)) {
        printf ("TERM: BootCart REFUSED - path too long (cwd=[%s] "
                "file=[%s])\r\n", s_menu.cwd, filename);
        clear_screen ();
        out_str (" Path too deep to boot");
        newline ();
        return;
    }
    printf ("TERM: BootCart assembled path [%s]\r\n", path);

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
         * would otherwise only ever reach ~12%). The address is
         * bounded to the GAME-IMAGE window (see
         * USB_FileToPSRAM's bounds check); a ROM that exceeds
         * CART_GAME_MAX_BYTES is rejected with "USB: REJECT" before
         * any byte hits PSRAM. */
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
        if (Terminal_OutLevel () != 0U) {
            printf ("TERM: WARNING MSX still draining (level=%u after "
                    "600 ms) - launch text will queue behind it\r\n",
                    (unsigned)Terminal_OutLevel ());
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

/* Boot the flash-served Nextor kernel. Reachable from two screens - the
 * file list and the Nextor submenu - because the whole point of the
 * submenu is arriving here with device 2 already set up. */
static void nextor_boot (void) {
    /* Boot the flash-served Nextor kernel (ASCII16K: 16 KiB banks of
     * nextor_rom[] at 0x4000..0x7FFF selected by a bank number
     * written to 0x6000, plus the driver mailbox at
     * 0x7FF0..0x7FF5 serviced by nextor.c). Uses EXACTLY the
     * proven game-launch dance (soft_reset_into_cart): push the
     * launch byte, the MSX-side terminal's rom_start runs rst 0
     * from MSX RAM, and the BIOS re-probe finds the armed cart -
     * here the kernel's 'AB' at 0x4000 (nextor_rom bank 0 of the
     * MSXSoftware/NextorDriver/Nextor-3.0.RISKYMSX2.ROM image).
     * Physical F1 cannot be sniffed through the menu's CHGET
     * forwarding (the BIOS turns F1 into its KEY string), so the
     * menu accepts the letter N; the footer hints it. */
    printf ("TERM: N -> NEXTOR\r\n");
    clear_screen ();
    out_str (" Booting Nextor...");
    newline ();
    /* Let the MSX print the launch text before the 0x03 byte and
     * the mapper swap (same discipline as Terminal_BootCart).
     * CRITICAL: keep the soft_reset_into_cart sequence exactly
     * as for game ROMs - the Nextor kernel's init has the same
     * "MSX slot probe 0x4000 within ~20 ms of RST 0" timing, so
     * the 30 ms swap window in soft_reset_into_cart is what
     * avoids the boot hang regression. */
    term_wait_drain (600U);
    soft_reset_into_cart (CART_MAP_NEXTOR);
}

/* Trim the trailing '/name' component off the end of s_menu.cwd, so
 * "GAMES/SUB/" becomes "GAMES/" and "GAMES/" becomes "". Idempotent
 * on an already-empty cwd. Used when the user picks the synthetic
 * "<DIR> .." row at the top of a subdirectory listing. */
static void cwd_pop (void) {
    size_t n = 0U;
    while (s_menu.cwd[n] != '\0' && n < sizeof (s_menu.cwd) - 1U) n++;
    /* Walk back over the trailing-slash + last-component. The cwd
     * invariant is "trailing '/' iff non-empty" - see the scan
     * function. If n is 0 the cwd is already empty and there is
     * nothing to pop. */
    if (n == 0U) return;
    /* Drop the trailing '/' (always present when n > 0). */
    n--;
    if (n == 0U) { s_menu.cwd[0] = '\0'; return; }
    /* Now find the previous '/' so the surviving prefix still ends in
     * one. */
    while (n > 0U && s_menu.cwd[n - 1U] != '/') n--;
    s_menu.cwd[n] = '\0';
}

/* Append "<name>/" to s_menu.cwd, bounded by TERM_PATH_MAX. The name
 * is the SFN of the subdirectory the user picked, which is always
 * 8.3 on this volume - so the 64-byte budget covers deep trees with
 * plenty of room. Returns 0 on overflow (caller refuses and tells
 * the user). */
static int cwd_push (const char *name) {
    size_t cur = 0U;
    while (s_menu.cwd[cur] != '\0' && cur < sizeof (s_menu.cwd)) cur++;
    /* cur is the NUL position. The cwd already ends in '/' when
     * non-empty, so the next char is the start of the new component
     * and the trailing '/' has to be appended. The total write is
     * name + '/' + NUL. */
    size_t need = 0U;
    while (name[need] != '\0') need++;
    /* + 1 for the trailing '/', + 1 for the NUL. */
    if (cur + need + 2U > sizeof (s_menu.cwd)) return 0;
    for (size_t k = 0U; k < need; k++) s_menu.cwd[cur + k] = name[k];
    s_menu.cwd[cur + need]     = '/';
    s_menu.cwd[cur + need + 1U] = '\0';
    return 1;
}

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
        if (s_menu.is_dir[s_menu.sel]) {
            /* Directory row: descend, or ascend on the synthetic
             * "<DIR> .." entry. The list is rebuilt from disk in both
             * cases - the directory contents may have changed since
             * the user last visited (especially for a USB stick the
             * user is editing on a PC), and the cost is one
             * f_readdir loop on a small directory. */
            const char *name = s_menu.names[s_menu.sel];
            if (name[0] == '.' && name[1] == '.' && name[2] == '\0') {
                printf ("TERM: RET on <DIR> .. -> up\r\n");
                cwd_pop ();
            } else {
                if (!cwd_push (name)) {
                    /* The cwd ran out of room. The boot path would
                     * also overflow, so refusing to descend is the
                     * right answer - surface it to the UART and stay
                     * on the current list. */
                    printf ("TERM: REFUSED descend into [%s] - cwd full "
                            "(len=%u)\r\n", name, (unsigned)sizeof (s_menu.cwd));
                    out_str (" Path too deep");
                    newline ();
                    return;
                }
                printf ("TERM: RET on dir [%s] -> descend, cwd now [%s]\r\n",
                        name, s_menu.cwd);
            }
            scan_usb ();
            s_menu.page = 0;
            s_menu.sel  = 0;
            print_file_list ();
            return;
        }
        /* file row: copy the SFN into s_menu.picked, then show the
         * mapper menu. */
        int i = 0;
        const char *n = s_menu.names[s_menu.sel];
        while (n[i] && i < FILE_NAME_MAX - 1) { s_menu.picked[i] = n[i]; i++; }
        s_menu.picked[i] = '\0';
        printf ("TERM: picked file [%s] (cwd=[%s]) -> mapper menu\r\n",
                s_menu.picked, s_menu.cwd);
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
    } else if (key == 0x1B) {            /* ESC = parent, or root rescan */
        if (s_menu.cwd[0] != '\0') {
            /* ESC is a quick back action inside a folder. At the root
             * it retains its original meaning: rescan the USB volume. */
            printf ("TERM: ESC -> parent from cwd=[%s]\r\n", s_menu.cwd);
            cwd_pop ();
        } else {
            printf ("TERM: ESC -> rescan root\r\n");
        }
        scan_usb ();
        s_menu.page = 0;
        s_menu.sel   = 0;
        print_file_list ();
        return;
    } else if (key == 'N' || key == 'n') {
        nextor_boot ();
        return;
    } else if (key == 'B' || key == 'b') {
        usb_drive_manager ();
        scan_usb ();
        print_file_list ();
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
    /* Boot gate. The MSX-side ROM's init() writes its GRAPH decision to
     * the control mailbox before it starts polling; consume it once.
     *   0x00 -> GRAPH held     -> render the terminal menu.
     *   0x01 -> GRAPH not held -> boot Nextor instead of rendering.
     * Any other value (e.g. the legacy v303 0x04) is treated as
     * "terminal" so a stale/older MSX-side ROM can never silently boot
     * Nextor. */
    uint8_t c = g_term_mbox.control;
    if (c != 0xFFU) {
        g_term_mbox.control = 0xFFU;
        if (!s_bootgate_done) {
            s_bootgate_done = 1U;
            if (c == TERM_CTL_BOOT_NEXTOR) {
                printf ("TERM: boot gate -> GRAPH not held, booting Nextor\r\n");
                /* Do not render the menu: hand straight to the Nextor
                 * launch, which pushes its own text + 0x03. */
                s_list_pending = 0U;
                nextor_boot ();
                return;
            }
            printf ("TERM: boot gate -> GRAPH held, terminal menu\r\n");
        }
    }

    /* Waiting for the boot-gate decision: do not render yet, or a
     * no-GRAPH power-on would flash the menu for ~40 ms before Nextor
     * boots. The timeout is the safety net for a missing/stale
     * MSX-side ROM - it falls back to the pre-gate behaviour (menu). */
    if (!s_bootgate_done) {
        if (s_bootgate_t0 == 0U) {
            s_bootgate_t0 = term_rdcycle ();
            if (s_bootgate_t0 == 0U) s_bootgate_t0 = 1U;  /* keep != 0 */
        }
        if ((uint32_t)(term_rdcycle () - s_bootgate_t0)
            < TERM_BOOTGATE_TIMEOUT_CYC) {
            return;
        }
        printf ("TERM: boot gate timeout - rendering menu\r\n");
        s_bootgate_done = 1U;
    }

    /* Drain keyboard FIFO. */
    uint8_t key;
    while (kbd_pop (&key)) {
        switch (s_menu.state) {
        case MENU_MAPPER:
            handle_mapper_key (key);
            break;
        case MENU_LIST:
        default:
            /* default, not a plain MENU_LIST: s_menu.state is only
             * ever set to one of the two, so a corrupt value would
             * otherwise land in the file list and quietly answer with
             * file-selection behaviour. */
            handle_list_key (key);
            break;
        }
    }

    /* On the first service pass of each terminal session, render the
     * file list once - the trigger is s_list_pending (re-armed by
     * Terminal_Reset), not a never-reset static, so sessions entered
     * via the boot gate render correctly too. */
    if (s_list_pending) {
        s_list_pending = 0;
        printf ("TERM: %s - first service, scanning USB\r\n",
                TERM_MENU_VERSION);
        scan_usb ();
        printf ("TERM: scan_usb -> count=%u\r\n", s_menu.count);
        print_file_list ();
    }
}

#pragma GCC pop_options
