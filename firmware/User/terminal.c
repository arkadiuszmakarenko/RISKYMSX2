
#include "terminal.h"
#include "cart.h"
#include "psram.h"
#include "usb_disk.h"
#include "nextor.h"
#include "dsk_image.h"
#include "img_image.h"
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

/* Terminal menu build tag, printed once per session on the UART.
 *
 * Why a tag at all: the on-screen footers are the user-facing version
 * marker, and they are 32-column lines that get edited often enough
 * to be ambiguous ("is this row 21 the new one or the old one?").
 * One log line at session start settles it, and it costs one string.
 * Bump this whenever the key map changes. */
#define TERM_MENU_VERSION "v3 (D/F1=nextor menu, N=boot)"

typedef enum {
    MENU_LIST,
    MENU_MAPPER,
    MENU_NEXTOR,
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

/* ------------------------------------------------------------------ */
/* Nextor submenu state                                                 */
/* ------------------------------------------------------------------ */

/* Declared up here rather than next to the renderer because
 * Terminal_Reset() has to clear the outcome line, and the submenu's
 * state is the same kind of thing s_menu is: per-session menu state
 * that a mapper swap must not carry into the next session. */

typedef enum {
    NXA_CREATE = 0,   /* create a new image of `sectors`      */
    NXA_DELETE,       /* delete the image that is there      */
    NXA_RESCAN,       /* re-probe both devices               */
} NextorActionKind;

typedef struct {
    uint8_t  kind;     /* NextorActionKind */
    uint32_t sectors;  /* NXA_CREATE only: image size in 512B sectors */
} NextorAction;

/* Action rows, at most five: four sizes while the image is missing,
 * one delete while it is there, plus the rescan row. The list is
 * rebuilt (never patched) by the render, so a create or a delete
 * cannot leave a row behind that points at a file which is gone.
 * s_menu.sel is the highlighted index on this screen - the same field
 * the file list and the mapper menu reuse for their own selection. */
static NextorAction s_nx_act[5];
static uint8_t      s_nx_nact;    /* rows in s_nx_act */

/* Outcome line of the last action, row 14. Static rather than a local
 * because the action runs, THEN the screen is redrawn - and the
 * redraw is a full clear_screen(), so anything printed during the
 * action is gone.
 *
 * Sized to the 30-column screen budget (see print_nx_img_line): a
 * longer message is truncated rather than wrapped, because a wrapped
 * message would push the two hint lines off the bottom of the screen. */
static char         s_nx_msg[30];

/* Last percentage drawn by term_nextor_progress, 0xFFFF = none. */
static uint32_t     s_nx_last_pct = 0xFFFFU;

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
    /* Nextor submenu state. The action list is not rebuilt here - it is
     * built by the render, which is the first thing that needs it - but
     * the outcome line has to be cleared, or a session that started
     * with a soft reset into the cart would show the previous
     * session's "created 512 MB image" as if it were current. */
    s_nx_msg[0]     = '\0';
    s_nx_nact       = 0U;
    s_nx_last_pct   = 0xFFFFU;
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

    /* Two hint rows at the bottom. Row 21 is the page indicator (it was
     * the only footer before the Nextor menu existed), row 22 is free
     * and carries the second half of the key map.
     *
     * "D" rather than "F1" is the entry point that works on this
     * hardware: pressing F1 forwards no byte at all through the
     * terminal's CHGET loop (see handle_list_key), so advertising it
     * as the way in would send the user to a key that does nothing.
     * F1 is still accepted for BIOSes that do deliver it. */
    char buf[32];
    move_cursor ((uint8_t)(1 + TERM_PAGE_SIZE), 1);
    snprintf (buf, sizeof (buf), " Pg %u/%u   D=nextor menu",
              (unsigned)(s_menu.page + 1U), (unsigned)menu_page_count ());
    out_str (buf);
    newline ();
    move_cursor ((uint8_t)(2 + TERM_PAGE_SIZE), 1);
    out_str (" N=boot  F=old  ESC=scan");
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
/* Nextor screen                                                         */
/* ------------------------------------------------------------------ */

/* Defined further down: the FAT scan owns the FatFs directory walk,
 * and nextor_boot is the N-key launch sequence, which lives with the
 * key handlers so the one soft-reset-into-cart call site stays next to
 * its race-window comment. */
static void scan_usb (void);
static void nextor_boot (void);

/* Two things need explaining on screen here, and neither can be
 * explained by a key alone:
 *
 *   1. The way in is 'D' on the file list, not F1. Physical F1 is
 *      accepted and advertised nowhere: on this hardware the MSX-side
 *      terminal loop forwards no byte for it at all (F1 is swallowed
 *      somewhere in the key path before CHSNS sees it), and the file
 *      list's footer is the only place a key can be announced.
 *
 *   2. Device 2's image is not shipped. It has to be as large as the
 *      stick can spare, and only the user knows how much that is, so
 *      creating it is an explicit act rather than something the
 *      firmware does when it does not find the file. The screen is
 *      that act: it reports what both devices look like right now,
 *      and offers the create / delete / rescan rows.
 *
 * Row map (0-based, the MSX accepts any 0..23):
 *
 *      0  title
 *      1  device 1, what it is
 *      2  device 1, the file and its size - or "(not present)"
 *      3  device 2, what it is
 *      4  device 2, the file and its size - or "(not present)"
 *      5  free space left on the stick
 *      6  (blank)
 *      7  action row 0, at TERM_NX_ROW0
 *      8  action row 1
 *      9  action row 2
 *     10  action row 3
 *     11  action row 4
 *     12  (blank)
 *     13  create progress bar
 *     14  outcome of the last action
 *     15-20 unused
 *     21  hint line 1
 *     22  hint line 2
 */
#define TERM_NX_ROW0        7U     /* first action row               */
#define TERM_NX_ROW_PROG    13U    /* create progress bar            */
#define TERM_NX_ROW_MSG     14U    /* outcome of the last action     */

/* Size presets offered for a new image, in 512-byte sectors. Four
 * entries rather than a typed-in number because this is an F1 screen
 * on a 32-column display with no text entry, and a size the user has
 * to count keystrokes to specify is one they will not specify.
 *
 * The floor is 16 MB because the volume inside the image has to come
 * out as FAT16 and MSX-DOS identifies anything below 4085 clusters as
 * FAT12 (see img_image.h); the ceiling is 512 MB because a bigger file
 * is minutes of cluster-chain walking over USB and this hardware's
 * stick is the one being carved up. */
static const uint32_t kNxSectors[4] = {
    32768UL,        /*  16 MB */
    131072UL,       /*  64 MB */
    262144UL,       /* 128 MB */
    1048576UL,      /* 512 MB */
};

static void nextor_build_actions (void) {
    uint8_t i;

    s_nx_nact = 0U;
    if (ImgImage_IsPresent () == 0U) {
        for (i = 0U; i < (uint8_t)(sizeof (kNxSectors) / sizeof (kNxSectors[0])); i++) {
            s_nx_act[s_nx_nact].kind    = NXA_CREATE;
            s_nx_act[s_nx_nact].sectors = kNxSectors[i];
            s_nx_nact++;
        }
    } else {
        /* No create row while the image is there: ImgImage_Create()
         * refuses to overwrite it, and offering a row that always
         * fails would be worse than not offering it. The delete is
         * the explicit, separate step. */
        s_nx_act[s_nx_nact].kind    = NXA_DELETE;
        s_nx_act[s_nx_nact].sectors = 0U;
        s_nx_nact++;
    }
    s_nx_act[s_nx_nact].kind    = NXA_RESCAN;
    s_nx_act[s_nx_nact].sectors = 0U;
    s_nx_nact++;

    if (s_menu.sel >= s_nx_nact) {
        s_menu.sel = (uint16_t)(s_nx_nact - 1U);
    }
}

/* Free space on the stick, in MiB, or `ok` = 0 when FatFs could not
 * say (stick absent, exFAT, or an I/O error). Shown because the size
 * presets are absolute: "512 MB" is only a sensible offer on a stick
 * that has 512 MB free, and this is the number the user needs to
 * decide.
 *
 * f_getfree() is not cheap - it walks the host FAT counting clusters,
 * a fraction of a second on a big stick - so it is called once per
 * screen redraw and its result is not cached across redraws. A
 * progress redraw never comes through here (it only repaints the bar),
 * so the create path is unaffected. */
#if (FF_MAX_SS != FF_MIN_SS)
/* csize is expressed in FF_MAX_SS units; with the two unequal the
 * sector size is per-volume and the arithmetic below is wrong. */
#error "term_nx_free_mib assumes FF_MAX_SS == FF_MIN_SS (csize counts 512B sectors)"
#endif
static uint32_t term_nx_free_mib (uint8_t *ok) {
    FATFS  *fs = (FATFS *)0;
    DWORD   ncl = 0U;
    uint32_t mib;

    *ok = 0U;
    if (f_getfree ("0:/", &ncl, &fs) != FR_OK || fs == (FATFS *)0) {
        return 0U;
    }
    /* FatFs's "could not count" answer, which is also what a full
     * volume reports - the same thing to show the user either way. */
    if (ncl == 0xFFFFFFFFU) {
        return 0U;
    }
    *ok = 1U;
    mib = ((uint32_t)ncl * (uint32_t)fs->csize) / (1024U * 2U);  /* -> MiB */
    return mib;
}

/* Create progress, in place on one row. Same shape and the same
 * discipline as term_progress_cb: throttle to a percentage change,
 * and skip the repaint when the MSX has not drained the ring - a
 * dropped cosmetic update is harmless, but a dropped one lands in the
 * middle of the "done" line the caller prints afterwards.
 *
 * ImgImage_Create() calls this once per 8 MiB hop of the cluster-chain
 * walk, so a 512 MB image is 64 callbacks: far below the rate that
 * would need throttling to be cheap, and the throttle is kept anyway
 * because the callback is the documented contract rather than an
 * implementation detail of today's step size. */
static void term_nextor_progress (uint32_t done, uint32_t total) {
    uint32_t pct;

    if (total == 0U) {
        return;
    }
    pct = (uint32_t)(((uint64_t)done * 100ULL) / (uint64_t)total);
    if (pct == s_nx_last_pct) {
        return;
    }
    s_nx_last_pct = pct;
    if (g_term_mbox.out_n > 1024U) {
        return;         /* MSX is behind; see term_progress_cb */
    }
    draw_bar (TERM_NX_ROW_PROG, 1U, pct);
}

/* One action row. The number in column 2 is the shortcut that runs
 * this row, so the digits and the labels cannot drift apart: both
 * come from the row's index. */
static void print_nx_action (const NextorAction *a, uint8_t idx) {
    char b[30];

    if (a->kind == NXA_CREATE) {
        snprintf (b, sizeof (b), "  %u  create  %u MB",
                  (unsigned)(idx + 1U), (unsigned)(a->sectors / 2048U));
    } else if (a->kind == NXA_DELETE) {
        const char *path = ImgImage_PrimaryPath ();
        snprintf (b, sizeof (b), "  %u  delete %s",
                  (unsigned)(idx + 1U), (path != (const char *)0) ? path : "(none)");
    } else {
        snprintf (b, sizeof (b), "  %u  rescan devices", (unsigned)(idx + 1U));
    }
    out_str (b);
}

/* The device-2 line under its description. MiB rather than sectors
 * because a disk is thought of in megabytes; the exact sector count is
 * what the kernel reads from CAPACITY and nobody types it in.
 *
 * "missing" rather than "(not present)" for the same reason every
 * other line here is short: the MSX is 32 columns wide and a line that
 * reaches the last column leaves the cursor wrapped, so the CR/LF
 * after it skips a screen row. 30 characters is the budget, and the
 * s_nx_msg / print_nx_action buffers are sized to truncate at it
 * rather than wrap. */
static void print_nx_img_line (void) {
    if (ImgImage_IsPresent () != 0U) {
        char b[30];
        snprintf (b, sizeof (b), "   %s  %u MB",
                  ImgImage_Path () ? ImgImage_Path () : "?",
                  (unsigned)(ImgImage_SectorCount () / 2048U));
        out_str (b);
    } else {
        char b[30];
        const char *path = ImgImage_PrimaryPath ();
        snprintf (b, sizeof (b), "   %s  missing",
                  (path != (const char *)0) ? path : "(no name set)");
        out_str (b);
    }
}

static void print_nextor_menu (void) {
    uint8_t i;
    uint8_t have_free;
    uint32_t free_mib;
    char b[30];

    /* Action list FIRST, because it decides how many rows there are
     * and clamps s_menu.sel - the arrow below has to be placed against
     * the post-clamp index, or a create (5 rows) followed by a redraw
     * (2 rows) leaves the sprite parked on a row that is no longer
     * highlighted. */
    nextor_build_actions ();

    /* Arrow first, same rationale as print_file_list / print_mapper_menu:
     * the sprite is on-screen before any of the text that explains
     * which row it is on. */
    move_pointer (0, (uint8_t)(TERM_NX_ROW0 + s_menu.sel));

    clear_screen ();
    move_cursor (0, 0);
    out_str (" NEXTOR   RET run  ESC back");
    newline ();

    /* Device 1. The name comes from the backend rather than being
     * spelled out here, so this screen and nextor.c cannot end up
     * describing different files. */
    move_cursor (1, 0);
    out_str (" D1 floppy 720K, read only");
    newline ();
    move_cursor (2, 1);
    if (DskImage_IsPresent () != 0U) {
        snprintf (b, sizeof (b), "   %s  %u sectors",
                  DskImage_Path () ? DskImage_Path () : "?",
                  (unsigned)DskImage_SectorCount ());
    } else {
        snprintf (b, sizeof (b), "   %s  missing",
                  DskImage_PrimaryPath () ? DskImage_PrimaryPath ()
                                          : "(no name set)");
    }
    out_str (b);
    newline ();

    /* Device 2. */
    move_cursor (3, 0);
    out_str (" D2 disk, read + write");
    newline ();
    move_cursor (4, 1);
    print_nx_img_line ();
    newline ();

    /* Free space on the stick. */
    free_mib = term_nx_free_mib (&have_free);
    move_cursor (5, 0);
    if (have_free != 0U) {
        snprintf (b, sizeof (b), " free on stick: %u MB", (unsigned)free_mib);
        out_str (b);
    } else {
        out_str (" free on stick: ?");
    }
    newline ();

    for (i = 0U; i < s_nx_nact; i++) {
        move_cursor ((uint8_t)(TERM_NX_ROW0 + i), 1);
        print_nx_action (&s_nx_act[i], i);
        newline ();
    }

    /* Progress row, blanked rather than left stale: the outcome line
     * below it is what the user reads, and a half-drawn bar under a
     * "failed" message is worse than no bar. */
    move_cursor (TERM_NX_ROW_PROG, 0);
    out_str ("                              ");
    newline ();

    move_cursor (TERM_NX_ROW_MSG, 1);
    out_str (s_nx_msg);
    newline ();

    move_cursor (21, 1);
    out_str (" RET run  ESC back to list");
    newline ();
    move_cursor (22, 1);
    out_str (" N boots Nextor right now");
    newline ();
}

/* Re-probe both devices, ignoring the outcome - the screen reports
 * what is there afterwards, and a failure to open is exactly the
 * "not present" case it renders. */
static void nextor_rescan (void) {
    (void)DskImage_Open ();
    (void)ImgImage_Open ();
    (void)snprintf (s_nx_msg, sizeof (s_nx_msg), "rescanned the USB stick");
}

static void nextor_create (uint32_t sectors) {
    const char *path = ImgImage_PrimaryPath ();
    ImgErr       err;

    if (path == (const char *)0) {
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg), "no image name configured");
        print_nextor_menu ();
        return;
    }

    /* Drop anything typed during the wait: those keys were meant for
     * the screen the user was on before, and replaying them onto the
     * freshly drawn one would look like the menu moved on its own. */
    {
        uint8_t junk;
        while (kbd_pop (&junk)) { }
    }

    s_nx_last_pct = 0xFFFFU;
    /* The "working" text goes on the outcome row, not the bar row: the
     * bar redraws in place on its own row and would wipe whatever was
     * written there. The two rows are adjacent for that reason. */
    move_cursor (TERM_NX_ROW_MSG, 1);
    out_str (" creating, please wait...");
    draw_bar (TERM_NX_ROW_PROG, 1, 0U);

    ImgImage_ProgressCB = term_nextor_progress;
    err = ImgImage_Create (path, sectors);
    ImgImage_ProgressCB = 0;

    /* Re-open rather than leave the state half-updated: the screen
     * below reports IsPresent(), and an image the screen calls
     * "(not present)" while the kernel sees it is exactly the kind of
     * disagreement that wastes an hour of debugging. ImgImage_Create
     * leaves the file closed, so this is just a probe. */
    ImgImage_Invalidate ();
    (void)ImgImage_Open ();

    printf ("TERM: nextor create '%s' %u sectors -> %d\r\n",
            path, (unsigned)sectors, (int)err);

    switch (err) {
    case IMG_OK:
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg),
                        "created %u MB image", (unsigned)(sectors / 2048U));
        break;
    case IMG_ERR_NO_SPACE:
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg),
                        "not enough space on stick");
        break;
    case IMG_ERR_EXISTS:
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg),
                        "image exists - delete first");
        break;
    case IMG_ERR_NOT_MOUNTED:
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg), "USB stick not mounted");
        break;
    case IMG_ERR_BAD_SIZE:
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg),
                        "%u MB: bad FAT16 size",
                        (unsigned)(sectors / 2048U));
        break;
    case IMG_ERR_FATFS:
    default:
        /* The specific FatFs error is on the UART log, which is where
         * the FRESULT belongs; the screen cannot say more usefully. */
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg),
                        "write failed - see UART log");
        break;
    }
    /* The action list is a different list now (the create rows are
     * gone, a delete row has appeared), so the old index means
     * something else. Start at the top rather than let the clamp in
     * nextor_build_actions() land the arrow on the last row. */
    s_menu.sel = 0U;
    print_nextor_menu ();
}

static void nextor_delete (void) {
    const char *path = ImgImage_PrimaryPath ();
    ImgErr       err;
    uint8_t      junk;

    if (path == (const char *)0) {
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg), "no image name configured");
        print_nextor_menu ();
        return;
    }
    while (kbd_pop (&junk)) { }

    /* Drop the backend's view of the image BEFORE unlinking. Leaving
     * it open would leave IsPresent() true and the cached sector count
     * intact, so the redraw below would report a disk that is no
     * longer there and offer "delete" instead of "create" - the screen
     * and the stick disagreeing, which is worse than either alone. */
    ImgImage_Invalidate ();
    err = ImgImage_Delete (path);
    if (err == IMG_OK) {
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg), "deleted the disk image");
    } else {
        (void)snprintf (s_nx_msg, sizeof (s_nx_msg),
                        "delete failed - see UART log");
    }
    printf ("TERM: nextor delete '%s' -> %d\r\n", path, (int)err);
    s_menu.sel = 0U;
    print_nextor_menu ();
}

static void nextor_run (uint8_t idx) {
    if (idx >= s_nx_nact) {
        return;
    }
    switch (s_nx_act[idx].kind) {
    case NXA_CREATE:
        nextor_create (s_nx_act[idx].sectors);
        break;
    case NXA_DELETE:
        nextor_delete ();
        break;
    case NXA_RESCAN:
    default:
        nextor_rescan ();
        print_nextor_menu ();
        break;
    }
}

static void handle_nextor_key (uint8_t key) {
    printf ("TERM: nextor key=0x%02X sel=%u nact=%u\r\n",
            key, s_menu.sel, s_nx_nact);

    if (key == 0x1E) {                   /* up */
        if (s_menu.sel > 0U) s_menu.sel--;
        move_pointer (0, (uint8_t)(TERM_NX_ROW0 + s_menu.sel));
        return;
    }
    if (key == 0x1F) {                   /* down */
        if ((int)s_menu.sel + 1 < (int)s_nx_nact) s_menu.sel++;
        move_pointer (0, (uint8_t)(TERM_NX_ROW0 + s_menu.sel));
        return;
    }
    if (key == 0x0D) {                   /* RET - run the highlighted row */
        printf ("TERM: nextor RET -> action %u\r\n", s_menu.sel);
        nextor_run ((uint8_t)s_menu.sel);
        return;
    }
    if (key == 0x1B) {                   /* ESC - back to the file list */
        printf ("TERM: nextor ESC -> file list\r\n");
        s_menu.state = MENU_LIST;
        /* Rescan: creating or deleting the image changes what the
         * file list has to show, and the list is not otherwise
         * refreshed on the way back. */
        scan_usb ();
        s_menu.page = 0;
        s_menu.sel  = 0;
        print_file_list ();
        return;
    }
    if (key >= '1' && key <= '9') {
        const uint8_t idx = (uint8_t)(key - '1');
        if (idx < s_nx_nact) {
            s_menu.sel = idx;
            nextor_run (idx);
        }
        return;
    }
    if (key == 'N' || key == 'n') {
        /* Booting from here is the point of the screen: the image the
         * user just created is already open (nextor_create re-opens
         * it), so the kernel finds device 2 on its first probe
         * instead of waiting out the 2 s retry window. */
        nextor_boot ();
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

/* Redraws the progress bar in place on the fixed screen row the
 * header occupied. Called from USB_FileToPSRAM's read loop via the
 * USB_ProgressCB hook - keep it short. */
static void term_progress_cb (uint32_t done, uint32_t total) {
    static uint32_t s_last_pct = 0xFFFFU;

    if (total == 0U) return;
    /* Throttle: only redraw when the percentage changes. */
    uint32_t pct = (uint32_t)(((uint64_t)done * 100ULL) / total);
    if (pct == s_last_pct) return;
    s_last_pct = pct;

    /* Backpressure: the ring is only 2048 bytes and the MSX drains at
     * ~50 µs/byte, so a bar redraw every 1% can outrun it. If out_n is
     * high, SKIP this redraw - a skipped cosmetic update is harmless,
     * but out_push() silently drops bytes when full, and dropping the
     * trailing " OK - booting MSX into cart" + 0x03 launch byte (see
     * soft_reset_into_cart) leaves the MSX in the terminal loop and
     * the cart never boots. Threading the ring below ~50% always
     * leaves room for the launch sequence. */
    if (g_term_mbox.out_n > 1024U) return;

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
        nextor_boot ();
        return;
    } else if (key == 0x3C || key == 0x01 || key == 'D' || key == 'd') {
        /* -> the Nextor submenu.
         *
         * 'D' is the documented key and F1 is a bonus, because F1
         * cannot be relied on here. The MSX side forwards CHGET's
         * return value verbatim (RomLoader/asm/terminal.asm) and
         * CHGET only produces a byte for a function key if the
         * machine's BIOS key buffer actually receives it. Measured on
         * this hardware: pressing F1 forwards NOTHING - not 0x01, not
         * the 0x3C matrix code, not any byte at all - while ordinary
         * keys arrive normally. Something in the key path eats the
         * F-keys before CHSNS can report them, and the fix for that
         * belongs on the MSX side (scan the matrix directly, SNSMAT)
         * rather than in a key guess here.
         *
         * So the entry point is a plain letter, which every BIOS
         * delivers, and the two F1 encodings are still accepted for
         * the machines where they do arrive:
         *
         *   0x3C  the raw key-matrix code, on a BIOS whose KEYB table
         *          gives F1 a matrix code rather than a token
         *   0x01  the KEY-string token, which is what a stock
         *          MSX-BIOS returns for F1
         *
         * None of the three means anything else on this screen, so
         * accepting all of them costs nothing - and picking the wrong
         * one would leave the menu with an entry point the user can
         * see advertised in the footer and never reach. */
        printf ("TERM: key 0x%02X -> NEXTOR menu\r\n", key);
        s_menu.sel   = 0;          /* first action row */
        s_menu.state = MENU_NEXTOR;
        s_nx_msg[0]  = '\0';       /* no stale outcome from a prior visit */
        print_nextor_menu ();
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
        switch (s_menu.state) {
        case MENU_MAPPER:
            handle_mapper_key (key);
            break;
        case MENU_NEXTOR:
            handle_nextor_key (key);
            break;
        case MENU_LIST:
        default:
            /* default, not a plain MENU_LIST: s_menu.state is only
             * ever set to one of the three, so a corrupt value would
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