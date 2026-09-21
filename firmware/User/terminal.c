
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
#define FILE_NAME_MAX    28U    /* printed-column width */

typedef enum {
    MENU_LIST,
    MENU_MAPPER,
} MenuState;

/* Per-file menu state. */
static struct {
    char    names[TERM_MAX_FILES][FILE_NAME_MAX];
    uint32_t sizes[TERM_MAX_FILES];  /* bytes */
    uint16_t count;
    uint16_t sel;       /* currently-highlighted index */
    uint8_t  loaded;    /* "ROM was loaded into PSRAM" */
    MenuState state;
    char     picked[FILE_NAME_MAX]; /* file name the user selected */
} s_menu;

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
    uint8_t next = (uint8_t)((g_term_mbox.out_tail + 1U) % 2048U);
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

/* ------------------------------------------------------------------ */
/* Public mailbox lifecycle                                            */
/* ------------------------------------------------------------------ */

void Terminal_Reset (void) {
    g_term_mbox.kbd_head = g_term_mbox.kbd_tail = g_term_mbox.kbd_n = 0;
    g_term_mbox.out_head = g_term_mbox.out_tail = g_term_mbox.out_n = 0;
    g_term_mbox.control = 0xFFU;
    s_menu.count = 0;
    s_menu.sel   = 0;
    s_menu.loaded = 0;
    s_menu.state  = MENU_LIST;
    s_menu.picked[0] = '\0';
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
    out_str (" RISKYMSX2 ^v RET ESC F=old");
    newline ();
}

static void print_file_list (void) {
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
    move_pointer (0, (uint8_t)(1 + s_menu.sel));

    move_cursor (0, 0);
    print_menu_title ();
    for (uint16_t i = 0; i < s_menu.count; i++) {
        /* Reserve column 0 for the sprite cursor - text starts at
         * column 1. */
        move_cursor ((uint8_t)(1 + i), 1);
        /* pad filename to FILE_NAME_MAX */
        const char *n = s_menu.names[i];
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
}

static void print_mapper_menu (void) {
    /* Sprite first - same rationale as print_file_list: the arrow
     * lands on the highlighted mapper row before the MSX has to read
     * any of the surrounding text. */
    move_pointer (0, (uint8_t)(4 + s_menu.sel));

    clear_screen ();
    move_cursor (0, 0);
    out_str (" File: ");
    out_str (s_menu.picked);
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
        if (fr != FR_OK || fi.altname[0] == 0) break;
        if (fi.fattrib & AM_DIR) continue;
        /* Accept any file - the v303 firmware filtered by extension
         * (.ROM) but the cart loader may serve .BIN/.MX1 too.
         *
         * Use the 8.3 SFN (fi.altname), not the LFN (fi.fname). LFN
         * paths like "0:/Metal Gear 2 - Solid Snake" fail f_open
         * with FR_NO_FILE (4) for some reason on our stick - the SFN
         * "METALG~1.ROM" works correctly. The loader (loader.c) was
         * already using the SFN; align the terminal with that.
         *
         * Trade-off: the file shown in the menu is now
         * METALG~1.ROM, not the human-friendly LFN. Good enough for
         * testing; can switch back to LFN once f_open's LFN handling
         * is diagnosed. */
        const char *n = fi.altname;
        size_t k = 0;
        while (n[k] && k < FILE_NAME_MAX - 1) {
            s_menu.names[s_menu.count][k] = n[k];
            k++;
        }
        s_menu.names[s_menu.count][k] = '\0';
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
     */
    out_push (0x03);
    /* 20 ms: enough for the MSX to read 0x03 (one screen-refresh poll)
     * and enter rom_start. At 200 MHz HCLK and ~5 ns/nop, 4,000,000
     * iterations = 20 ms. */
    for (volatile uint32_t i = 0; i < 4000000U; i++) { __asm__ volatile ("nop"); }
    printf ("TERM: soft_reset swap mapper=%d\r\n", (int)m);
    /* Swap the mapper. The MSX-side rom_start is now executing in
     * MSX RAM; both path (A) (UGLY_PATCH) and path (B) (RST 0 from
     * unknown) reach the BIOS slot probe within ~20 ms and find the
     * new ROM's 'AB' header. */
    (void)Cart_SetMapper_Safe (m);
    /* 50 ms cushion for PSRAM settle + new INIT LDIR copy + slot
     * probe completion + return-to-user-code. */
    for (volatile uint32_t i = 0; i < 10000000U; i++) { __asm__ volatile ("nop"); }
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

    uint32_t got = USB_FileToPSRAM (path,
                                    PSRAM_CART_BASE + CART_GAME_BASE,
                                    PSRAM_CART_SIZE - CART_GAME_BASE);
    printf ("TERM: USB_FileToPSRAM got=%u\r\n", (unsigned)got);
    if (got == 0U) {
        out_str (" Load FAILED");
        newline ();
        return;
    }
    s_menu.loaded = 1;
    out_str (" OK - booting MSX into cart");
    newline ();
    printf ("TERM: soft_reset_into_cart mapper=%d\r\n", (int)m);
    soft_reset_into_cart (m);
}

/* ------------------------------------------------------------------ */
/* Service - main-loop pump                                            */
/* ------------------------------------------------------------------ */

static void handle_list_key (uint8_t key) {
    /* arrow up / down / RET / ESC */
    printf ("TERM: list key=0x%02X sel=%u count=%u\r\n", key, s_menu.sel, s_menu.count);
    if (key == 0x1E) {                   /* up */
        if (s_menu.sel > 0) s_menu.sel--;
    } else if (key == 0x1F) {            /* down */
        if (s_menu.sel + 1U < s_menu.count) s_menu.sel++;
    } else if (key == 0x0D && s_menu.count > 0U) {
        /* pick this file -> show mapper menu */
        int i = 0;
        const char *n = s_menu.names[s_menu.sel];
        while (n[i] && i < FILE_NAME_MAX - 1) { s_menu.picked[i] = n[i]; i++; }
        s_menu.picked[i] = '\0';
        printf ("TERM: picked file [%s] -> mapper menu\r\n", s_menu.picked);
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
    }
    /* redraw cursor + arrow on the line we landed on */
    move_pointer (0, (uint8_t)(1 + s_menu.sel));
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

    /* On first entry after Terminal_Reset, render the file list once
     * - the trigger is "state == MENU_LIST and count == 0 (no scan
     * yet) and the FIFO is not currently being filled with a
     * boot-cart message." */
    static uint8_t s_first = 1;
    if (s_first) {
        s_first = 0;
        printf ("TERM: first service - scanning USB\r\n");
        scan_usb ();
        printf ("TERM: scan_usb -> count=%u\r\n", s_menu.count);
        print_file_list ();
    }
}

#pragma GCC pop_options