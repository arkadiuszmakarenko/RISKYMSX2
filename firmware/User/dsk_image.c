/*
 * dsk_image.c - MSX .dsk image backend.
 *
 * See dsk_image.h for the design notes and the rationale for keeping
 * this in its own module. Briefly: nextor.c owns the mailbox protocol,
 * the driver owns the wire contract, and the BPB/geometry/open/reopen
 * state machine lives here so neither of the other two has to.
 *
 * Threading: main-loop only. None of these functions is IRQ-safe.
 *
 * BPB layout (FAT12, MSX-DOS convention):
 *   +0   jmp short NOP          3 bytes
 *   +3   OEM name               8 bytes
 *   +11  bytes/sector           LE16  (always 512)
 *   +13  sectors/cluster        u8
 *   +14  reserved sectors       LE16  (FAT start LBA)
 *   +16  number of FATs         u8    (usually 2)
 *   +17  root entry count       LE16  (112 for 720K)
 *   +19  total sectors (16)     LE16  (1440 for 720K)
 *   +21  media descriptor       u8    (0xF9 for DS/DD)
 *   +22  sectors per FAT        LE16  (3 for 720K)
 *   +24  sectors per track      LE16  (9 for 720K)
 *   +26  heads                  LE16  (2 for 720K)
 *   +28  hidden sectors         LE32  (0 on flat image)
 *   +32  total sectors (32)     LE32  (0 on 720K; larger images use this)
 */

#include "dsk_image.h"
#include "usb_disk.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================
 * State
 * ====================================================================== */

static struct {
    FIL         fp;                /* FatFs file handle                       */
    DskGeometry geo;               /* parsed BPB                              */
    uint8_t     open;              /* FIL is open and usable                  */
    uint8_t     present;           /* last value reported via MediaState      */
    uint8_t     changed;           /* media-change latch                      */
    uint8_t     probed;            /* at least one Open call has happened     */
    uint8_t     last_fr;           /* FRESULT of the last underlying call     */
    char        path[32];          /* first 12 chars of the open file (SFN)   */
} s;

/* Candidate file names. Set by DskImage_SetCandidates (default: empty,
 * so an unset caller gets a clear "no candidates" failure instead of
 * silently opening the wrong file). */
static const char *const *s_candidates;
static uint8_t           s_candidate_count;

/* Convenience: read +3..+35 of the boot sector in one FatFs call. */
#define DSK_BPB_OFF_BPS         11U   /* bytes per sector       */
#define DSK_BPB_OFF_SPC         13U   /* sectors per cluster    */
#define DSK_BPB_OFF_RSV         14U   /* reserved sectors       */
#define DSK_BPB_OFF_NFAT        16U   /* number of FATs         */
#define DSK_BPB_OFF_ROOT        17U   /* root entry count       */
#define DSK_BPB_OFF_TOT16       19U   /* total sectors (16)     */
#define DSK_BPB_OFF_MEDIA       21U   /* media descriptor       */
#define DSK_BPB_OFF_SPFAT       22U   /* sectors per FAT        */
#define DSK_BPB_OFF_SPT         24U   /* sectors per track      */
#define DSK_BPB_OFF_HEADS       26U   /* heads                  */
#define DSK_BPB_OFF_HIDDEN      28U   /* hidden sectors         */
#define DSK_BPB_OFF_TOT32       32U   /* total sectors (32)     */
#define DSK_BPB_END             36U   /* one past the BPB       */

/* ========================================================================
 * BPB parsing
 * ====================================================================== */

/* Compute ceil(a/b) for positive integers. (a + b - 1) / b would do,
 * but the explicit form is faster on a RISC-V with no hardware divide. */
static uint32_t div_ceil (uint32_t a, uint32_t b) {
    return (a / b) + (uint32_t)((a % b) != 0U);
}

/* Little-endian loads from an arbitrary byte pointer. The BPB is read
 * sector-by-sector, so these cannot assume alignment - the FAT and
 * root directory entries are also unaligned and the FAT12 chain reader
 * below uses the same helpers. */
static uint16_t load_le16 (const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t load_le32 (const uint8_t *p) {
    return  (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* Convert a candidate path + index to "we picked this one". Updates
 * s.path so the boot log can name the source. */
static void record_path (const char *p) {
    size_t n = 0U;
    while ((n < sizeof(s.path) - 1U) && (p[n] != '\0')) {
        s.path[n] = p[n];
        n++;
    }
    s.path[n] = '\0';
}

/* Validate the parsed BPB. The constraints are FAT12-specific (this
 * driver is fixed at FAT12 - we never boot exFAT or FAT16 images) and
 * match what a stock MSX-DOS 2.44 / Nextor 720K image ships. Anything
 * else is rejected with DSK_ERR_BAD_BPB so a wrong file (e.g. a 1.44M
 * image on a 720K format) shows as a clear parse failure rather than
 * silently handing the kernel a broken geometry. */
static DskErr bpb_validate (const DskGeometry *g) {
    if (g->bytes_per_sector != DSK_SECTOR_SIZE) {
        return DSK_ERR_BAD_BPB;
    }
    if (g->sectors_per_cluster == 0U) {
        return DSK_ERR_BAD_BPB;
    }
    if ((g->sectors_per_cluster & (g->sectors_per_cluster - 1U)) != 0U) {
        /* must be a power of two - FAT12 demands it for cluster math. */
        return DSK_ERR_BAD_BPB;
    }
    if (g->num_fats == 0U || g->num_fats > 4U) {
        return DSK_ERR_BAD_BPB;
    }
    if (g->root_entry_count == 0U || (g->root_entry_count % 16U) != 0U) {
        return DSK_ERR_BAD_BPB;
    }
    if (g->sectors_per_fat == 0U) {
        return DSK_ERR_BAD_BPB;
    }
    if (g->total_sectors == 0U) {
        return DSK_ERR_BAD_BPB;
    }
    /* FAT12 ceiling: cluster count must fit in 12 bits (<= 4084). */
    if (g->cluster_count < 1U || g->cluster_count > 4084U) {
        return DSK_ERR_BAD_BPB;
    }
    /* data region must not run past the image. */
    if ((g->data_start_lba + g->cluster_count * g->sectors_per_cluster)
        > g->total_sectors) {
        return DSK_ERR_TRUNCATED;
    }
    return DSK_OK;
}

/* Read sector 0 (the boot sector) into s.path... no, into a caller-
 * supplied scratch buffer. Parses the BPB into s.geo. Returns DSK_OK
 * or one of DSK_ERR_*. */
static DskErr bpb_parse (uint8_t *scratch) {
    DskGeometry *g = &s.geo;
    FRESULT fr;
    UINT    br;
    uint32_t data_clusters;

    fr = f_lseek (&s.fp, 0U);
    if (fr != FR_OK) { s.last_fr = (uint8_t)fr; return DSK_ERR_FATFS; }

    fr = f_read (&s.fp, scratch, DSK_SECTOR_SIZE, &br);
    if (fr != FR_OK) { s.last_fr = (uint8_t)fr; return DSK_ERR_FATFS; }
    if (br != DSK_SECTOR_SIZE) {
        return DSK_ERR_TRUNCATED;
    }

    /* Signature check: jmp short + NOP. MSX-DOS uses 0xEB 0x?? 0x90
     * (the classic DOS jump); reject anything else so a stray binary
     * that happens to live at NEXTOR.DSK does not parse. */
    if (scratch[0] != 0xEBU && scratch[0] != 0xE9U) {
        return DSK_ERR_BAD_BPB;
    }

    /* BPB fields - field-by-field so the offsets are greppable. */
    g->bytes_per_sector  = load_le16 (&scratch[DSK_BPB_OFF_BPS]);
    g->sectors_per_cluster = scratch[DSK_BPB_OFF_SPC];
    g->reserved_sectors  = load_le16 (&scratch[DSK_BPB_OFF_RSV]);
    g->num_fats          = scratch[DSK_BPB_OFF_NFAT];
    g->root_entry_count  = load_le16 (&scratch[DSK_BPB_OFF_ROOT]);
    g->total_sectors_16  = load_le16 (&scratch[DSK_BPB_OFF_TOT16]);
    g->media_descriptor  = scratch[DSK_BPB_OFF_MEDIA];
    g->sectors_per_fat   = load_le16 (&scratch[DSK_BPB_OFF_SPFAT]);
    g->sectors_per_track = load_le16 (&scratch[DSK_BPB_OFF_SPT]);
    g->heads             = load_le16 (&scratch[DSK_BPB_OFF_HEADS]);

    /* Total sectors: BPB+19 if non-zero, else BPB+32. */
    {
        uint32_t t32 = load_le32 (&scratch[DSK_BPB_OFF_TOT32]);
        g->total_sectors = (g->total_sectors_16 != 0U)
                         ? (uint32_t)g->total_sectors_16
                         : t32;
    }

    /* Derived LBA map - what every other part of this file uses. */
    g->fat_start_lba      = g->reserved_sectors;
    g->root_dir_sectors   = div_ceil ((uint32_t)g->root_entry_count * 32U,
                                      DSK_SECTOR_SIZE);
    g->root_dir_start_lba = g->fat_start_lba
                          + (uint32_t)g->num_fats * g->sectors_per_fat;
    g->data_start_lba     = g->root_dir_start_lba + g->root_dir_sectors;

    data_clusters = (g->total_sectors - g->data_start_lba)
                  / g->sectors_per_cluster;
    g->cluster_count = data_clusters;

    return bpb_validate (g);
}

/* ========================================================================
 * Open / close / invalidate
 * ====================================================================== */

static void drop (void) {
    if (s.open != 0U) {
        (void)f_close (&s.fp);
    }
    s.open = 0U;
    (void)memset (&s.geo, 0, sizeof (s.geo));
}

void DskImage_Init (void) {
    /* Close before clearing, same as ImgImage_Init and for the same
     * reason: Nextor_Init runs again on every swap to CART_MAP_NEXTOR,
     * and a mapper swap is not a power cycle.
     *
     * The cost of skipping the f_close is not a leaked allocation (this
     * build has FF_FS_LOCK == 0, so a FIL is not drawn from a pool) but
     * the two things f_close actually returns: FatFs's sector window and
     * the volume's dirty flags. Zeroing s makes the next probe re-open
     * the same file while the stick's FAT is still marked dirty from
     * whatever the previous owner wrote, and the window still attributed
     * to a handle nobody will ever close again. */
    if (s.open != 0U) {
        (void)f_close (&s.fp);
    }
    (void)memset (&s, 0, sizeof (s));
    s_candidates     = (const char *const *)0;
    s_candidate_count = 0U;
}

void DskImage_Invalidate (void) {
    if (s.open != 0U) {
        (void)f_close (&s.fp);
        printf ("[dsk] handle closed: stick removed\r\n");
    }
    drop ();
    /* Mark as not-present so the next STATUS reports the transition
     * once and the kernel sees a media change. */
    if (s.probed != 0U) {
        s.changed = 1U;
    }
    s.present = 0U;
}

DskErr DskImage_Open (void) {
    uint8_t scratch[DSK_SECTOR_SIZE];
    DskErr  err;
    uint8_t i;

    if (s.open != 0U) {
        return DSK_OK;
    }
    s.probed   = 1U;
    s.last_fr  = 0U;
    s.path[0]  = '\0';

    if (s_candidates == (const char *const *)0 || s_candidate_count == 0U) {
        return DSK_ERR_NO_FILE;
    }
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        return DSK_ERR_NOT_MOUNTED;
    }

    /* Try each candidate SFN in order. The first f_open that returns
     * FR_OK wins. Record the path so the boot log can name the source. */
    for (i = 0U; i < s_candidate_count; i++) {
        const char *name = s_candidates[i];
        FRESULT fr = f_open (&s.fp, name, FA_READ);
        s.last_fr = (uint8_t)fr;
        if (fr == FR_OK) {
            record_path (name);
            break;
        }
    }
    if (s.last_fr != (uint8_t)FR_OK) {
        return DSK_ERR_NO_FILE;
    }

    err = bpb_parse (scratch);
    if (err != DSK_OK) {
        printf ("[dsk] BPB parse failed on '%s' (fr=%u)\r\n",
                s.path, (unsigned)s.last_fr);
        (void)f_close (&s.fp);
        s.open = 0U;
        return err;
    }

    s.open    = 1U;
    s.present = 1U;
    s.changed = 1U;          /* transition absent->present */
    printf ("[dsk] open '%s': %u bytes, BPB bps=%u spc=%u "
            "rsv=%u nfat=%u root=%u tot=%u spfat=%u spt=%u heads=%u\r\n",
            s.path,
            (unsigned)(s.geo.total_sectors * s.geo.bytes_per_sector),
            (unsigned)s.geo.bytes_per_sector,
            (unsigned)s.geo.sectors_per_cluster,
            (unsigned)s.geo.reserved_sectors,
            (unsigned)s.geo.num_fats,
            (unsigned)s.geo.root_entry_count,
            (unsigned)s.geo.total_sectors,
            (unsigned)s.geo.sectors_per_fat,
            (unsigned)s.geo.sectors_per_track,
            (unsigned)s.geo.heads);
    return DSK_OK;
}

/* ========================================================================
 * Cheap queries
 * ====================================================================== */

uint8_t DskImage_HasProbed (void) {
    return s.probed;
}

uint8_t DskImage_IsPresent (void) {
    return (uint8_t)((s.open != 0U && s.geo.total_sectors != 0U) ? 1U : 0U);
}

uint32_t DskImage_SectorCount (void) {
    return s.geo.total_sectors;
}

const DskGeometry *DskImage_Geometry (void) {
    return &s.geo;
}

const char *DskImage_Path (void) {
    return s.path[0] != '\0' ? s.path : (const char *)0;
}

const char *DskImage_PrimaryPath (void) {
    return (s_candidates != (const char *const *)0 && s_candidate_count != 0U)
           ? s_candidates[0] : (const char *)0;
}

uint8_t DskImage_LastFR (void) {
    return s.last_fr;
}

uint8_t DskImage_MediaState (void) {
    if (!DskImage_IsPresent ()) {
        return 0U;
    }
    return s.changed ? 2U : 1U;
}

void DskImage_MediaLatchConsume (void) {
    s.changed = 0U;
}

void DskImage_SetCandidates (const char *const *names, uint8_t count) {
    s_candidates      = names;
    s_candidate_count = count;
}

/* ========================================================================
 * Sector read
 *
 * One retry per call - a single transient stick error should not wedge
 * the driver. A second consecutive failure drops the handle so the next
 * probe re-opens from the (possibly re-inserted) stick. The driver never
 * sees this directly: it only sees the result byte "success/no media".
 * ====================================================================== */

uint8_t DskImage_ReadSector (uint32_t lba, uint8_t *dst) {
    UINT    br = 0U;
    UINT    attempt;
    FRESULT fr;
    uint32_t retry = 0U;

    if (dst == (uint8_t *)0) {
        return 0U;
    }
    if (!DskImage_IsPresent ()) {
        return 0U;
    }
    if (lba >= s.geo.total_sectors) {
        return 0U;
    }

    for (attempt = 0U; attempt < 2U; attempt++) {
        fr = f_lseek (&s.fp, (FSIZE_t)lba * (FSIZE_t)DSK_SECTOR_SIZE);
        if (fr == FR_OK) {
            fr = f_read (&s.fp, dst, DSK_SECTOR_SIZE, &br);
        }
        s.last_fr = (uint8_t)fr;
        if ((fr == FR_OK) && (br == DSK_SECTOR_SIZE)) {
            return 1U;
        }
        retry++;
    }
    (void)retry;

    /* Consecutive failures - drop the handle and report absent. The
     * next probe re-opens. We deliberately do NOT arm the change latch
     * here: the served content (this same file on the same stick) has
     * not actually changed, and a false media-change after a transient
     * I/O error is what makes the kernel unmap the drive for no reason.
     * The state machines above will report "no media" until the next
     * Open succeeds. */
    (void)f_close (&s.fp);
    s.open    = 0U;
    s.present = 0U;
    return 0U;
}

uint8_t DskImage_ReadBootSector (uint8_t *dst) {
    if (dst == (uint8_t *)0 || !DskImage_IsPresent ()) {
        return 0U;
    }
    return DskImage_ReadSector (0U, dst);
}
