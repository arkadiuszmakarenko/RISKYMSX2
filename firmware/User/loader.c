/********************************** (C) COPYRIGHT *******************************
 * File Name          : loader.c
 * Description        : ROM-loader command engine (see loader.h).
 *********************************************************************************/

#include "loader.h"
#include "cart.h"
#include "psram.h"
#include "usb_disk.h"
#include "ch32v4x7.h"
#include "debug.h"
#include <string.h>

Loader_Mailbox g_loader_mbox;

/* Directory scan state. */
static uint8_t  s_names[LOADER_MAX_FILES][LOADER_NAME_LEN];
static uint16_t s_file_count;
static uint16_t s_dir_index;      /* next file DIR_READ will return */

/* Pending mapper (applied by CMD_RESET so the reset atomically swaps
 * the served ROM). */
static Cart_Mapper s_pending_mapper = CART_MAP_NONE;

void Loader_Reset (void) {
    memset ((void *)&g_loader_mbox, 0, sizeof (g_loader_mbox));
    g_loader_mbox.status = LOADER_ST_READY | LOADER_ST_DONE;
    s_file_count = 0;
    s_dir_index  = 0;
}

/* Push a result byte (main-loop context only). */
static void push_result (uint8_t b) {
    /* ring buffer; if full the oldest byte is lost - the loader never
     * asks for more than 11 bytes without popping. */
    uint8_t next = (uint8_t)((g_loader_mbox.res_tail + 1U)
                            % LOADER_FIFO_DEPTH);
    if (next == g_loader_mbox.res_head) {
        return;  /* full */
    }
    g_loader_mbox.res[g_loader_mbox.res_tail] = b;
    g_loader_mbox.res_tail = next;
    g_loader_mbox.res_n++;
}

/* ------------------------------------------------------------------ */
/* Directory scan: USB stick root, .ROM files only (8.3 names).        */
/* ------------------------------------------------------------------ */

/* Convert an LFN/8.3 name to the fixed 11-byte "NAME    ROM" form.
 * Takes the extension from the last dot; truncates name to 8 / ext to
 * 3; uppercase (the MSX UI prints it as-is). */
static void make_83_name (const char *fname, uint8_t out[LOADER_NAME_LEN]) {
    memset (out, ' ', LOADER_NAME_LEN);
    const char *dot = strrchr (fname, '.');
    size_t base_len = dot ? (size_t)(dot - fname) : strlen (fname);
    if (base_len > 8) base_len = 8;
    for (size_t i = 0; i < base_len; i++) {
        char c = fname[i];
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        out[i] = (uint8_t)c;
    }
    if (dot) {
        const char *ext = dot + 1;
        size_t ext_len = strlen (ext);
        if (ext_len > 3) ext_len = 3;
        for (size_t i = 0; i < ext_len; i++) {
            char c = ext[i];
            if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
            out[8 + i] = (uint8_t)c;
        }
    }
}

static uint8_t is_rom_ext (const char *fname) {
    const char *dot = strrchr (fname, '.');
    if (dot == NULL) return 0;
    dot++;
    if ((dot[0] == 'R' || dot[0] == 'r')
        && (dot[1] == 'O' || dot[1] == 'o')
        && (dot[2] == 'M' || dot[2] == 'm')
        && dot[3] == '\0') {
        return 1;
    }
    return 0;
}

static void cmd_dir_open (void) {
    FRESULT fr;
    DIR dir;
    FILINFO fi;

    s_file_count = 0;
    s_dir_index  = 0;

    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        goto done;
    }

    fr = f_opendir (&dir, "0:/");
    if (fr != FR_OK) goto done;

    while (s_file_count < LOADER_MAX_FILES) {
        fr = f_readdir (&dir, &fi);
        if (fr != FR_OK || fi.fname[0] == 0) break;
        if (fi.fattrib & AM_DIR) continue;
        if (!is_rom_ext (fi.fname)) continue;
        make_83_name (fi.fname, s_names[s_file_count]);
        s_file_count++;
    }
    f_closedir (&dir);

done:
    push_result ((uint8_t)(s_file_count & 0xFF));
    push_result ((uint8_t)(s_file_count >> 8));
    printf ("LOADER: DIR_OPEN -> %u rom files\r\n",
            (unsigned)s_file_count);
}

static void cmd_dir_read (void) {
    if (s_dir_index < s_file_count) {
        for (uint8_t i = 0; i < LOADER_NAME_LEN; i++) {
            push_result (s_names[s_dir_index][i]);
        }
        s_dir_index++;
    } else {
        /* out of range: pad with spaces (defensive; the loader only
         * reads file_count entries). */
        for (uint8_t i = 0; i < LOADER_NAME_LEN; i++) {
            push_result (' ');
        }
    }
}

/* ------------------------------------------------------------------ */
/* CMD_LOAD_ROM: stream the named file into PSRAM at cart offset 0.   */
/* ------------------------------------------------------------------ */

static void cmd_load_rom (const uint8_t *name83) {
    /* Rebuild "0:/NAME.EXT" from the 11-byte 8.3 form. */
    char path[32];
    uint8_t p = 0;
    path[p++] = '0'; path[p++] = ':'; path[p++] = '/';
    for (uint8_t i = 0; i < 8; i++) {
        if (name83[i] == ' ') break;
        path[p++] = (char)name83[i];
    }
    path[p++] = '.';
    for (uint8_t i = 8; i < 11; i++) {
        if (name83[i] == ' ') break;
        path[p++] = (char)name83[i];
    }
    path[p] = '\0';

    printf ("LOADER: LOAD_ROM '%s'\r\n", path);

    uint32_t got = USB_FileToPSRAM (path, PSRAM_CART_BASE,
                                    PSRAM_CART_SIZE);
    /* result: 4-byte length BE */
    push_result ((uint8_t)(got >> 24));
    push_result ((uint8_t)(got >> 16));
    push_result ((uint8_t)(got >> 8));
    push_result ((uint8_t)(got));
    printf ("LOADER: LOAD_ROM copied %u bytes -> PSRAM\r\n",
            (unsigned)got);
}

/* ------------------------------------------------------------------ */
/* CMD_SET_MAPPER: remember the mapper for CMD_RESET.                 */
/* ------------------------------------------------------------------ */

static void cmd_set_mapper (uint8_t m) {
    if (m == 0 || m >= CART_MAP_MAX) {
        push_result (1);   /* refused */
        return;
    }
    s_pending_mapper = (Cart_Mapper)m;
    push_result (0);       /* accepted */
    printf ("LOADER: SET_MAPPER %s (pending reset)\r\n",
            Cart_MapperNames[m]);
}

/* ------------------------------------------------------------------ */
/* CMD_RESET: apply mapper + pulse the MSX reset line.               */
/* ------------------------------------------------------------------ */

static void cmd_reset (void) {
    printf ("LOADER: RESET -> mapper %s, pulsing MSX reset\r\n",
            Cart_MapperNames[(unsigned)s_pending_mapper]);
    if (s_pending_mapper != CART_MAP_NONE) {
        (void)Cart_SetMapper (s_pending_mapper);
    }
    /* Give the print time to drain, then hold the MSX in reset long
     * enough for a clean power-on-style boot. */
    Delay_Ms (50);
    Cart_AssertMSXReset (100);
}

/* ------------------------------------------------------------------ */
/* Main-loop service                                                    */
/* ------------------------------------------------------------------ */

void Loader_Service (void) {
    if (!g_loader_mbox.have_cmd) {
        return;
    }
    g_loader_mbox.have_cmd = 0;

    switch (g_loader_mbox.cmd) {
    case LOADER_CMD_DIR_OPEN:
        cmd_dir_open ();
        break;
    case LOADER_CMD_DIR_READ:
        cmd_dir_read ();
        break;
    case LOADER_CMD_DIR_CLOSE:
        /* nothing to do; no result */
        break;
    case LOADER_CMD_LOAD_ROM:
        cmd_load_rom ((const uint8_t *)g_loader_mbox.args);
        break;
    case LOADER_CMD_SET_MAPPER:
        cmd_set_mapper (g_loader_mbox.args[0]);
        break;
    case LOADER_CMD_RESET:
        cmd_reset ();
        break;
    default:
        break;
    }

    /* Mark the command finished (the loader's wait-loop polls this). */
    g_loader_mbox.status |= LOADER_ST_DONE;
}