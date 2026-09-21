/********************************** (C) COPYRIGHT *******************************
 * File Name          : loader.c
 * Description        : ROM-loader command engine (see loader.h).
 *********************************************************************************/

#include "loader.h"
#include "cart.h"
#include "psram.h"
#include "scc.h"
#include "usb_disk.h"
#include "ch32v4x7.h"
#include "debug.h"
#include <string.h>

Loader_Mailbox g_loader_mbox;

/* Directory scan state. */
static uint8_t  s_names[LOADER_MAX_FILES][LOADER_NAME_LEN];
static uint16_t s_file_count;
static uint16_t s_dir_index;      /* next file DIR_READ will return */

/* Pending mapper (applied by CMD_SOFTRESET so the RAM-resident Z80
 * slingshot delays before jp 0x0000; the swap happens
 * pre-jump-from-RAM so BIOS INIT re-probes slots after the new mapper
 * is live). */
static Cart_Mapper s_pending_mapper = CART_MAP_NONE;

/* Bytes successfully loaded into PSRAM by the most recent CMD_LOAD_ROM.
 * CMD_SET_MAPPER / CMD_SOFTRESET are refused while this is zero, so a
 * failed file open (e.g. SFN mismatch on the stick) can't trap the MSX
 * by switching mappers with a 0xFF-filled PSRAM window. The MSX-side
 * loader checks the 4-byte length returned by LOAD_ROM before going on
 * to the mapper menu, but it doesn't gate the cart mapper on it - so
 * the firmware MUST refuse here. */
static uint32_t s_bytes_loaded = 0U;

void Loader_Reset (void) {
    memset ((void *)&g_loader_mbox, 0, sizeof (g_loader_mbox));
    g_loader_mbox.status = LOADER_ST_READY | LOADER_ST_DONE;
    s_file_count = 0;
    s_dir_index  = 0;
    s_bytes_loaded = 0U;
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

/* Copy the on-disk SFN ("altname" from f_readdir) into the 12-byte
 * wire slot the MSX receives. The SFN is what f_open() will accept -
 * it is the FAT short filename, with a "~N" tilde-tail when the
 * original LFN was longer than 8 chars (e.g. "Knightmare.rom" ->
 * "KNIGHT~1.ROM", 12 chars). The slot is null-padded to LOADER_NAME_LEN
 * so cmd_load_rom can use it as a C string without walking past the
 * stored bytes. */
static void copy_altname (const char *alt, uint8_t out[LOADER_NAME_LEN]) {
    size_t n = strlen (alt);
    if (n > LOADER_NAME_LEN) n = LOADER_NAME_LEN;
    for (size_t i = 0; i < n; i++) out[i] = (uint8_t)alt[i];
    for (size_t i = n; i < LOADER_NAME_LEN; i++) out[i] = 0U;
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
        copy_altname (fi.altname, s_names[s_file_count]);
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

static void cmd_load_rom (const uint8_t *name12) {
    /* The 12-byte slot holds the FAT short name as it lives on disk
     * (verbatim from fi.altname at DIR_OPEN time) - already
     * null-terminated by copy_altname(). f_open accepts the SFN
     * directly, so we just wrap it with the volume prefix. */
    char path[32];
    uint8_t p = 0;
    path[p++] = '0'; path[p++] = ':'; path[p++] = '/';
    for (uint8_t i = 0; i < LOADER_NAME_LEN && name12[i] != 0U; i++) {
        path[p++] = (char)name12[i];
    }
    path[p] = '\0';

    printf ("LOADER: LOAD_ROM '%s'\r\n", path);

    /* Reject obvious garbage: a path with no extension is almost
     * certainly a desync (the MSX-side loader sends 12 fixed bytes;
     * if the upper bytes are garbage the path looks like
     * "0:/FOO.ROMJUNK" and f_open will find nothing). Bailing early
     * avoids the misleading "f_open failed" print and keeps
     * s_bytes_loaded at 0 so CMD_SET_MAPPER / CMD_SOFTRESET refuse
     * the cart swap. */
    if (strchr (path + 3, '.') == NULL) {
        printf ("LOADER: LOAD_ROM refusing bad name '%s'\r\n", path);
        push_result (0); push_result (0); push_result (0); push_result (0);
        s_bytes_loaded = 0U;
        return;
    }

    uint32_t got = USB_FileToPSRAM (path,
                                    PSRAM_CART_BASE + CART_GAME_BASE,
                                    PSRAM_CART_SIZE - CART_GAME_BASE);
    /* result: 4-byte length BE */
    push_result ((uint8_t)(got >> 24));
    push_result ((uint8_t)(got >> 16));
    push_result ((uint8_t)(got >> 8));
    push_result ((uint8_t)(got));
    s_bytes_loaded = got;
    printf ("LOADER: LOAD_ROM copied %u bytes -> PSRAM+0x%x\r\n",
            (unsigned)got, (unsigned)CART_GAME_BASE);
}

/* ------------------------------------------------------------------ */
/* CMD_SET_MAPPER: remember the mapper for CMD_SOFTRESET.             */
/* ------------------------------------------------------------------ */

static void cmd_set_mapper (uint8_t m) {
    if (m == 0 || m >= CART_MAP_MAX) {
        push_result (1);   /* refused */
        return;
    }
    /* Refuse to latch a PSRAM-backed mapper until a ROM has actually
     * been loaded. The MSX-side loader doesn't gate the menu on the
     * LOAD_ROM length it read back, so a failed file open would
     * otherwise trap the MSX in a cart window full of 0xFF (from
     * psram_copy_rom's init fill) with a mapper installed: every
     * read at 0x4000 returns 0xFF, no 'AB' header, the BIOS drops
     * to BASIC (best case) or executes 0xFF (RST 38h) in a tight
     * loop (worst case, with bank-switching mappers). Only the
     * FLASH mapper is exempt - it serves the flash-resident
     * selector_rom[] image, NOT PSRAM, so an empty PSRAM window
     * doesn't affect it. */
    if (m != CART_MAP_FLASH && s_bytes_loaded == 0U) {
        printf ("LOADER: SET_MAPPER %s refused (no ROM loaded)\r\n",
                Cart_MapperNames[m]);
        push_result (1);   /* refused */
        return;
    }
    s_pending_mapper = (Cart_Mapper)m;
    push_result (0);       /* accepted */
    printf ("LOADER: SET_MAPPER %s (pending reset)\r\n",
            Cart_MapperNames[m]);
}

/* ------------------------------------------------------------------ */
/* CMD_SOFTRESET: apply mapper without driving ~RESET.               */
/* ------------------------------------------------------------------ */
/* The MSX cart edge has no ~RESET driver on most MSX2+ machines:
 * the reset circuit lives inside the mainboard and the cart edge is
 * either unconnected or input-only. Driving PE4 low would damage
 * some designs and would not reboot the MSX anyway. CMD_SOFTRESET
 * is therefore the ONLY reboot path.
 *
 * Sequence: the MSX-side loader copies a small Z80 slingshot to MSX
 * RAM at 0xF000 (first thing in main()), then `jp 0xF000`s into it.
 * The slingshot di's, resets SP/I, runs a bounded delay loop
 * (~120 ms at 3.58 MHz - plenty for our ~2 ms Cart_SetMapper_Safe +
 * a generous margin), then `jp 0x0000`. BIOS INIT runs AFTER the
 * firmware has finished swapping the mapper, so the BIOS re-probes
 * 0x4000 and finds the new mapper live on the cart bus.
 *
 * The firmware never drives ~RESET: same safety gate as the legacy
 * CMD_RESET (fall back to FLASH if no ROM was loaded, so we never
 * serve 0xFF from PSRAM with a mapper installed), same SCC-queue
 * drain on the way out of KONAMISCC, same Cart_SetMapper_Safe() to
 * close the VTF/g_mapper/bankOffsets race window - just with
 * hold_msx_reset=0. The Z80 keeps running, fetches the jp 0xF000
 * from MSX RAM, and stays in RAM for the entire delay (so the cart
 * swap can't yank executing code out from under the Z80). */

static void cmd_softreset (void) {
    printf ("LOADER: SOFTRESET -> mapper %s, no ~RESET drive\r\n",
            Cart_MapperNames[(unsigned)s_pending_mapper]);
    /* Same safety gate as the (now-removed) CMD_RESET: never install a
     * PSRAM-backed mapper with no image. Falling back to CART_MAP_FLASH
     * keeps the MSX in the loader menu so the user can retry the
     * LOAD_ROM. */
    if (s_pending_mapper == CART_MAP_NONE
        || (s_pending_mapper != CART_MAP_FLASH
            && s_bytes_loaded == 0U)) {
        printf ("LOADER: SOFTRESET aborting - no ROM loaded, "
                "keeping FLASH mapper\r\n");
        s_pending_mapper = CART_MAP_FLASH;
        /* The Z80 slingshot will see FLASH install in place; no
         * ~RESET drive (we never touch PE4). */
        (void)Cart_SetMapper_Safe (CART_MAP_FLASH);
        return;
    }

    /* Drain SCC queue on the way out of KONAMISCC (prevents a leftover
     * write from being applied with the new non-SCC mapper as
     * g_mapper). */
    if (Cart_GetMapper () == CART_MAP_KONAMISCC
        && s_pending_mapper != CART_MAP_KONAMISCC) {
        const uint32_t drained = SCC_FlushQueue ();
        if (drained) {
            printf ("LOADER: SOFTRESET flushed %u queued SCC writes\r\n",
                    (unsigned)drained);
        }
    }

    /* Cart_SetMapper_Safe closes the VTF/g_mapper/bankOffsets race
     * window (disables EXTI0 + global IRQ, waits for ~SLTSL high,
     * drives bus off, clears phantom EXTI0 edge, DSB/ISB fence,
     * re-enables IRQ). The Z80 slingshot in MSX RAM is what actually
     * reboots - it runs the PSRAM-settling delay itself so the
     * firmware's idle loop stays free to service CLI + USB while the
     * MSX waits. The MSX-side `soft_reset()` delay is in
     * MSXSoftware/RomLoader/romloader.c, between the mbox_cmd ack
     * and the jp 0xE000. */
    const int swap_rc = Cart_SetMapper_Safe (s_pending_mapper);
    printf ("LOADER: SOFTRESET swap done rc=%d\r\n", swap_rc);
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
    case LOADER_CMD_SOFTRESET:
        /* Mark DONE BEFORE cmd_softreset swaps the mapper. The FLASH
         * handler is what serves the mailbox window at 0x7FF0; the
         * PSRAM-backed mappers don't expose it. If we set DONE
         * post-swap (the normal post-switch path below), the MSX's
         * mbox_wait_done() loop reads from the new mapper's window
         * (PSRAM bytes), never sees ST_DONE, and hangs forever -
         * which is why "BOOT..." printed but nothing else happened.
         * Set it here, BEFORE the swap; the post-switch |= 1 below
         * is then a redundant no-op. */
        g_loader_mbox.status |= LOADER_ST_DONE;
        cmd_softreset ();
        break;
    default:
        break;
    }

    /* Mark the command finished (the loader's wait-loop polls this). */
    g_loader_mbox.status |= LOADER_ST_DONE;
}