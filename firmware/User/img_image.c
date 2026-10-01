/*
 * img_image.c - MSX .img image backend (device 2).
 *
 * See img_image.h for why this is a separate module from dsk_image.c
 * rather than a mode flag inside it. In short: device 1 is a bare FAT12
 * floppy image whose geometry lives in its own boot sector, device 2 is
 * a disk whose capacity is its file length and whose contents the
 * kernel has to scan for partitions. Two small objects, one dispatch
 * table in nextor.c.
 *
 * Threading: main-loop only. None of these functions is IRQ-safe, and
 * none of them may be called from a command handler - see nextor.c's
 * "Probe policy" note for why probing lives on the idle path.
 *
 * Threading of a different kind, which does matter: the module is used
 * by two callers that never overlap. nextor.c probes and serves, and
 * only while the Nextor mapper is the one installed; the terminal's
 * Nextor screen creates and deletes the file, and only while it is.
 * FatFs is not reentrant (FF_FS_REENTRANT is 0) and the host is single
 * threaded, so the two never see the same FIL at the same time.
 */

#include "img_image.h"
#include "usb_disk.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================
 * On-disk layout
 *
 * Every offset below is a byte offset in a 512-byte sector, written out
 * explicitly rather than through a packed struct. The structures in the
 * Nextor SDK (partit.h fatBootSector / masterBootRecord) would say the
 * same thing with less code, but they are the kernel's Z80 layout: the
 * `ulong serialNumber` is 4-byte aligned there, so the serial sits at
 * +40 rather than +39, and the fatTypeString at +55 rather than +54.
 * Spelling the offsets out keeps this file readable next to the
 * structures it mirrors, and the fields that actually matter (the ones
 * CHECK_FAT_BOOT and the MSX-DOS BPB loader read) are all at their
 * standard offsets either way.
 * ====================================================================== */

/* --- Master boot record, sector 0 of the file --- */
#define MBR_JMP            0U    /* EB FE 90, the DOS long jump        */
#define MBR_OEM            3U    /* "NEXTO30" - Nextor's own OEM name  */
#define MBR_PARTITION      446U  /* four 16-byte table entries          */
#define MBR_SIGNATURE      510U  /* 55 AA                              */

/* MBR partition table entry, 16 bytes at MBR_PARTITION + 16*n */
#define PT_STATUS          0U    /* 80h = active                       */
#define PT_TYPE            3U    /* 0Eh = FAT16, LBA addressing       */
#define PT_FIRST_LBA       8U    /* 4 bytes LE                         */
#define PT_SECTOR_COUNT    12U   /* 4 bytes LE                         */

/* --- FAT16 boot sector, sector 1 of the file --- */
#define BS_JMP             0U
#define BS_OEM             3U    /* "NEXTOR20"                         */
#define BS_SECTOR_SIZE     11U   /* 2 bytes LE, 512                    */
#define BS_SPC             13U   /* sectors per cluster                */
#define BS_RESERVED        14U   /* 2 bytes LE, 1                      */
#define BS_NUM_FATS        16U
#define BS_ROOT_ENTRIES    17U   /* 2 bytes LE, 512                    */
#define BS_TOTAL_16        19U   /* 2 bytes LE, 0 when > 65535        */
#define BS_MEDIA           21U   /* F0h                                */
#define BS_SECTORS_PER_FAT 22U   /* 2 bytes LE                         */
#define BS_SECTORS_PER_TRK 24U   /* 2 bytes LE, 0 - see the header    */
#define BS_HEADS           26U   /* 2 bytes LE, 0                      */
#define BS_HIDDEN          28U   /* 4 bytes LE, 0                      */
#define BS_TOTAL_32        32U   /* 4 bytes LE                         */
#define BS_DRIVE_NUM       36U   /* 80h: hard disk, 0 if a floppy      */
#define BS_EXT_SIGNATURE   38U   /* 29h                                */
#define BS_VOLUME_SERIAL   40U   /* 4 bytes - see fat16_serial()       */
#define BS_VOLUME_LABEL    44U   /* 11 bytes                           */
#define BS_FAT_TYPE        55U   /* 8 bytes, "FAT16   "               */
#define BS_SIGNATURE       510U  /* 55 AA                              */

/* The first sector of each FAT: media descriptor, then 0xFFFF, 0xFFFF
 * (cluster 1 = reserved/bad), then 0xFFFF for the 16-bit end-of-chain of
 * cluster 2. Everything past byte 4 in that sector is a free cluster
 * entry and has to be zero. */
#define FAT_HEAD_0         0xF0U
#define FAT_HEAD_1         0xFFU
#define FAT_HEAD_2         0xFFU
#define FAT_HEAD_3         0xFFU

/* FAT16 shape, fixed (partit.h): two FATs, 512 root entries = 16
 * sectors, 256 FAT16 entries per sector. The two limits that matter:
 * MAX_FAT16_SECTORS_PER_FAT is a hard ceiling because the kernel's
 * CHECK_FAT_BOOT rejects anything above 256 with no diagnostic, and
 * MAX_FAT16_CLUSTER_COUNT is the 16-bit cluster-number ceiling.
 * FAT16_MIN_CLUSTERS is the floor: below 4085 clusters MSX-DOS (and
 * every other FAT driver) identifies the volume as FAT12, and we would
 * be shipping an image the kernel mounts with the wrong driver. */
#define FAT_COPIES         2U
#define FAT16_ROOT_ENTRIES 512U
#define DIR_ENTRIES_PER_SECTOR 16U
#define FAT16_ROOT_SECTORS (FAT16_ROOT_ENTRIES / DIR_ENTRIES_PER_SECTOR)
#define FAT16_ENTRIES_PER_SECTOR 256U
#define FAT16_MAX_SECTORS_PER_FAT 256U
#define FAT16_MAX_CLUSTERS 65524U
#define FAT16_MIN_CLUSTERS 4085U

/* ========================================================================
 * State
 * ====================================================================== */

static struct {
    FIL     fp;              /* FatFs file handle, opened R+W        */
    uint64_t size;            /* file length at open time, in bytes   */
    uint32_t sectors;         /* size / 512                            */
    uint8_t open;             /* FIL is open and usable               */
    uint8_t present;          /* last value reported via MediaState   */
    uint8_t changed;          /* media-change latch                   */
    uint8_t probed;           /* at least one Open call has happened  */
    uint8_t last_fr;          /* FRESULT of the last FatFs call       */
    char    path[32];         /* the open file's path, for the log    */
} s;

static const char *const *s_candidates;
static uint8_t           s_candidate_count;

ImgProgressCB ImgImage_ProgressCB;

/* ========================================================================
 * Small helpers
 * ====================================================================== */

static uint32_t div_ceil (uint32_t a, uint32_t b) {
    return (a / b) + (uint32_t)((a % b) != 0U);
}

static void store_le16 (uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static void store_le32 (uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void record_path (const char *p) {
    size_t n = 0U;
    while ((n < sizeof(s.path) - 1U) && (p[n] != '\0')) {
        s.path[n] = p[n];
        n++;
    }
    s.path[n] = '\0';
}

/* ========================================================================
 * Open / close / invalidate
 * ====================================================================== */

static void drop (void) {
    if (s.open != 0U) {
        (void)f_close (&s.fp);
    }
    s.open    = 0U;
    s.present = 0U;
    s.size    = 0U;
    s.sectors = 0U;
}

void ImgImage_Init (void) {
    /* Close before clearing. A mapper swap is not a power cycle: the
     * Nextor mapper can be installed more than once per session (the
     * terminal's N key), and Nextor_Init runs on every swap. Zeroing
     * the state without the f_close would strand the FIL object and its
     * cached sector on the stick for the rest of the power cycle. */
    if (s.open != 0U) {
        (void)f_close (&s.fp);
    }
    (void)memset (&s, 0, sizeof (s));
    s_candidates     = (const char *const *)0;
    s_candidate_count = 0U;
}

void ImgImage_Invalidate (void) {
    if (s.open != 0U) {
        printf ("[img] handle closed\r\n");
    }
    drop ();
    /* Report the transition once, so the kernel sees a media change and
     * re-probes. Only after a probe has happened: a device that has
     * never had an image must not look like it lost one. */
    if (s.probed != 0U) {
        s.changed = 1U;
    }
    s.present = 0U;
}

ImgErr ImgImage_Open (void) {
    uint8_t i;

    if (s.open != 0U) {
        return IMG_OK;
    }
    s.probed  = 1U;
    s.last_fr = 0U;
    s.path[0] = '\0';

    if (s_candidates == (const char *const *)0 || s_candidate_count == 0U) {
        return IMG_ERR_NO_FILE;
    }
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        return IMG_ERR_NOT_MOUNTED;
    }

    /* Read AND write. A file the stick refuses to open for writing is
     * not presented at all rather than presented as writable: the +7
     * flag byte the driver hands the kernel is per device, not per
     * medium, so a read-only .img would be advertised as writable and
     * fail every write with an I/O error instead of not being there. */
    for (i = 0U; i < s_candidate_count; i++) {
        FRESULT fr = f_open (&s.fp, s_candidates[i], FA_READ | FA_WRITE);
        s.last_fr = (uint8_t)fr;
        if (fr == FR_OK) {
            record_path (s_candidates[i]);
            break;
        }
    }
    if (s.last_fr != (uint8_t)FR_OK) {
        return IMG_ERR_NO_FILE;
    }

    s.size = (uint64_t)f_size (&s.fp);
    if ((s.size % (uint64_t)IMG_SECTOR_SIZE) != 0U) {
        printf ("[img] '%s' is %u bytes - not a whole number of "
                "512-byte sectors\r\n", s.path, (unsigned)s.size);
        (void)f_close (&s.fp);
        s.size = 0U;
        return IMG_ERR_BAD_SIZE;
    }
    s.sectors = (uint32_t)(s.size / (uint64_t)IMG_SECTOR_SIZE);
    if (s.sectors == 0U) {
        printf ("[img] '%s' is empty\r\n", s.path);
        (void)f_close (&s.fp);
        return IMG_ERR_BAD_SIZE;
    }

    s.open    = 1U;
    s.present = 1U;
    s.changed = 1U;          /* transition absent -> present */
    printf ("[img] open '%s': %u bytes, %u sectors\r\n",
            s.path, (unsigned)s.size, (unsigned)s.sectors);
    return IMG_OK;
}

/* ========================================================================
 * Cheap queries
 * ====================================================================== */

uint8_t ImgImage_HasProbed (void)  { return s.probed; }
uint8_t ImgImage_IsPresent (void) { return (uint8_t)(s.open != 0U ? 1U : 0U); }
uint32_t ImgImage_SectorCount (void) { return s.sectors; }
uint64_t ImgImage_FileSize (void)  { return s.size; }
const char *ImgImage_Path (void)    { return s.path[0] != '\0' ? s.path : (const char *)0; }
uint8_t ImgImage_LastFR (void)      { return s.last_fr; }

const char *ImgImage_PrimaryPath (void) {
    return (s_candidates != (const char *const *)0 && s_candidate_count != 0U)
           ? s_candidates[0] : (const char *)0;
}

uint8_t ImgImage_MediaState (void) {
    if (!ImgImage_IsPresent ()) {
        return 0U;
    }
    return s.changed ? 2U : 1U;
}

void ImgImage_MediaLatchConsume (void) {
    s.changed = 0U;
}

void ImgImage_SetCandidates (const char *const *names, uint8_t count) {
    s_candidates      = names;
    s_candidate_count = count;
}

/* ========================================================================
 * Sector transfer
 * ====================================================================== */

uint8_t ImgImage_ReadSector (uint32_t lba, uint8_t *dst) {
    UINT    br = 0U;
    UINT    attempt;
    FRESULT fr;

    if (dst == (uint8_t *)0 || !ImgImage_IsPresent ()) {
        return 0U;
    }
    if (lba >= s.sectors) {
        return 0U;
    }
    for (attempt = 0U; attempt < 2U; attempt++) {
        fr = f_lseek (&s.fp, (FSIZE_t)lba * (FSIZE_t)IMG_SECTOR_SIZE);
        if (fr == FR_OK) {
            fr = f_read (&s.fp, dst, IMG_SECTOR_SIZE, &br);
        }
        s.last_fr = (uint8_t)fr;
        if ((fr == FR_OK) && (br == IMG_SECTOR_SIZE)) {
            return 1U;
        }
    }

    /* Two failures in a row: the stick went away (or the file did). Drop
     * the handle so the next probe re-opens, and report absent WITHOUT
     * arming the change latch - the medium has not changed, the
     * transport has, and a false media change is what unmaps a working
     * drive. See the note in img_image.h. */
    drop ();
    return 0U;
}

uint8_t ImgImage_WriteSector (uint32_t lba, const uint8_t *src) {
    UINT    bw = 0U;
    UINT    attempt;
    FRESULT fr;

    if (src == (const uint8_t *)0 || !ImgImage_IsPresent ()) {
        return 0U;
    }
    if (lba >= s.sectors) {
        return 0U;
    }
    for (attempt = 0U; attempt < 2U; attempt++) {
        fr = f_lseek (&s.fp, (FSIZE_t)lba * (FSIZE_t)IMG_SECTOR_SIZE);
        if (fr == FR_OK) {
            fr = f_write (&s.fp, src, IMG_SECTOR_SIZE, &bw);
        }
        s.last_fr = (uint8_t)fr;
        if ((fr == FR_OK) && (bw == IMG_SECTOR_SIZE)) {
            return 1U;
        }
    }
    drop ();
    return 0U;
}

/* ========================================================================
 * FAT16 geometry for a new image
 *
 * Deliberately NOT Nextor's CalculateFatFileSystemParametersFat16 (the
 * firmware never has a partition table to divide up, and that function
 * derives the sector count FROM a size in K rather than the other way
 * round). What it does do is pick the same sectors-per-cluster for the
 * same size, so an image built here and one built by Nextor's own FDISK
 * are the same layout for the same size.
 * ====================================================================== */

typedef struct {
    uint8_t  spc;            /* sectors per cluster (a power of two)  */
    uint8_t  spc_log2;
    uint16_t spf;            /* sectors per FAT, 1..256               */
    uint32_t clusters;       /* data clusters                          */
    uint32_t part_sectors;   /* what the partition spans              */
} fat16_params;

/* Pick sectors per cluster from the partition size, on the same ladder
 * fdisk.c uses (its size unit is 1 MiB of partition; ours is 512-byte
 * sectors, so 2048 sectors to the MiB). */
static void pick_spc (uint32_t part_sectors, uint8_t *spc, uint8_t *log2) {
    const uint32_t mb = div_ceil (part_sectors, 2048U);
    uint8_t        p;

    if      (mb <=  128U) { p = 2U; }   /*    4 sectors =  2 KiB */
    else if (mb <=  256U) { p = 3U; }   /*    8 sectors =  4 KiB */
    else if (mb <=  512U) { p = 4U; }   /*   16 sectors =  8 KiB */
    else if (mb <= 1024U) { p = 5U; }   /*   32 sectors = 16 KiB */
    else if (mb <= 2048U) { p = 6U; }   /*   64 sectors = 32 KiB */
    else                  { p = 7U; }   /*  128 sectors = 64 KiB */
    *spc    = (uint8_t)(1U << p);
    *log2   = p;
}

/* Solve for the FAT size and the cluster count of a FAT16 filesystem
 * that fits in `part_sectors`.
 *
 * The two are mutually dependent - the FAT occupies sectors that would
 * otherwise be clusters - so this is a fixed-point iteration rather than
 * a formula: guess a FAT size, count the clusters that fit, ask how big
 * that FAT has to be, repeat. It converges in two or three rounds
 * because the FAT grows more slowly than the cluster count it displaces,
 * and the loop bound turns "does not converge" into "use the last
 * guess" instead of a hang.
 *
 * Returns 1 on success, 0 if the size cannot host a real FAT16 (too few
 * clusters to be FAT16 at all, or not even room for the metadata). */
static uint8_t fat16_solve (uint32_t part_sectors, fat16_params *p) {
    uint32_t clusters = 0U;
    uint32_t need;
    uint32_t i;

    /* Room for the reserved sector, two minimum-size FATs and the root
     * directory. Anything under this cannot be laid out at all, and the
     * subtractions below would wrap. */
    if (part_sectors < (1U + (2U * 1U) + FAT16_ROOT_SECTORS + 1U)) {
        return 0U;
    }

    p->part_sectors = part_sectors;
    pick_spc (part_sectors, &p->spc, &p->spc_log2);

    p->spf = 1U;
    for (i = 0U; i < 8U; i++) {
        clusters = (part_sectors - 1U - (FAT_COPIES * p->spf)
                                  - FAT16_ROOT_SECTORS) >> p->spc_log2;
        need = div_ceil (clusters + 2U, FAT16_ENTRIES_PER_SECTOR);
        if (need < 1U) {
            need = 1U;
        }
        if (need > FAT16_MAX_SECTORS_PER_FAT) {
            need = FAT16_MAX_SECTORS_PER_FAT;
        }
        if (need == (uint32_t)p->spf) {
            break;              /* fixed point reached */
        }
        p->spf = (uint16_t)need;
    }

    /* One last count with the settled FAT size, then the two ceilings.
     * The cluster ceiling is the one that can bind: past 65524 the
     * 16-bit cluster numbers overflow, so the count is cut and the FAT
     * is pinned at its 256-sector maximum (which is also all the kernel
     * will accept). Cutting the cluster count leaves the tail of the
     * partition unused, which is normal for a partitioned disk. */
    clusters = (part_sectors - 1U - (FAT_COPIES * p->spf)
                              - FAT16_ROOT_SECTORS) >> p->spc_log2;
    if (clusters > FAT16_MAX_CLUSTERS) {
        clusters = FAT16_MAX_CLUSTERS;
        p->spf    = FAT16_MAX_SECTORS_PER_FAT;
    }
    if (clusters < FAT16_MIN_CLUSTERS) {
        return 0U;              /* would be a FAT12 volume */
    }
    p->clusters = clusters;

    /* The layout has to fit. clusters is floored, so the slack is
     * under one cluster - but assert it rather than trust it, because a
     * partition whose metadata overruns it is a corrupt disk and the
     * symptom (DOS writing past the end) is far away from the cause. */
    if ((1U + (FAT_COPIES * p->spf) + FAT16_ROOT_SECTORS
             + (clusters << p->spc_log2)) > part_sectors) {
        return 0U;
    }
    return 1U;
}

/* ========================================================================
 * Creation
 * ====================================================================== */

/* Write one sector of the new image. Sequential access, so the seek is
 * a hint; the caller walks the image front to back. */
static FRESULT put_sector (FIL *fp, uint32_t lba, const uint8_t *buf) {
    UINT    bw = 0U;
    FRESULT fr;

    fr = f_lseek (fp, (FSIZE_t)lba * (FSIZE_t)IMG_SECTOR_SIZE);
    if (fr != FR_OK) {
        return fr;
    }
    fr = f_write (fp, buf, IMG_SECTOR_SIZE, &bw);
    if ((fr == FR_OK) && (bw != IMG_SECTOR_SIZE)) {
        return FR_DISK_ERR;
    }
    return fr;
}

/* Zero `count` sectors starting at `lba`, in 8 KiB chunks so the USB
 * side sees 16-sector writes rather than one per sector. */
static FRESULT put_zeros (FIL *fp, uint32_t lba, uint32_t count) {
    static uint8_t zeros[16U * IMG_SECTOR_SIZE];
    uint32_t       done = 0U;
    FRESULT        fr;

    fr = f_lseek (fp, (FSIZE_t)lba * (FSIZE_t)IMG_SECTOR_SIZE);
    if (fr != FR_OK) {
        return fr;
    }
    while (done < count) {
        uint32_t n = count - done;
        UINT     bw = 0U;
        if (n > (uint32_t)(sizeof (zeros) / IMG_SECTOR_SIZE)) {
            n = (uint32_t)(sizeof (zeros) / IMG_SECTOR_SIZE);
        }
        fr = f_write (fp, zeros, n * IMG_SECTOR_SIZE, &bw);
        if (fr != FR_OK) {
            return fr;
        }
        if (bw != (n * IMG_SECTOR_SIZE)) {
            return FR_DISK_ERR;
        }
        done += n;
    }
    return FR_OK;
}

/* Free space on the volume holding `path`, in clusters. A negative
 * result means "could not tell", which the caller treats as "go" - the
 * create then fails on its own with a FatFs error rather than being
 * refused on a guess. */
static int64_t free_clusters (const char *path) {
    FATFS  *fs  = (FATFS *)0;
    DWORD   ncl = 0U;
    FRESULT fr;

    fr = f_getfree (path, &ncl, &fs);
    if (fr != FR_OK) {
        return -1;
    }
    /* A cluster count FatFs could not fill in means "full", which is
     * the same answer as zero. */
    if (ncl == 0xFFFFFFFFU) {
        return 0;
    }
    return (int64_t)ncl;
}

ImgErr ImgImage_Create (const char *path, uint32_t total_sectors) {
    static uint8_t sector[IMG_SECTOR_SIZE];
    fat16_params   g;
    FIL            fp;
    FRESULT        fr;
    uint32_t       part_sectors;
    uint32_t       fat_heads;      /* first sector of FAT #2 */
    uint32_t       written;        /* sectors the metadata occupies */
    uint32_t       pos;
    uint32_t       step;
    int64_t        have;

    if (path == (const char *)0 || path[0] == '\0') {
        return IMG_ERR_BAD_SIZE;
    }
    if (total_sectors < IMG_MIN_SECTORS || total_sectors > IMG_MAX_SECTORS) {
        return IMG_ERR_BAD_SIZE;
    }
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        return IMG_ERR_NOT_MOUNTED;
    }

    /* Refuse to touch a file that is already there. This never
     * overwrites: the file IS the disk, and a disk image is exactly the
     * kind of thing a user has data in. */
    fr = f_open (&fp, path, FA_READ);
    if (fr == FR_OK) {
        (void)f_close (&fp);
        return IMG_ERR_EXISTS;
    }

    /* One sector of the file is the MBR; the partition starts at LBA 1
     * and runs to the end. */
    part_sectors = total_sectors - 1U;
    if (fat16_solve (part_sectors, &g) == 0U) {
        printf ("[img] %u sectors does not host a FAT16 "
                "(spc=%u spf=%u)\r\n", (unsigned)total_sectors,
                (unsigned)g.spc, (unsigned)g.spf);
        return IMG_ERR_BAD_SIZE;
    }

    /* The image cannot be smaller than the metadata it has to contain:
     * MBR, boot sector, both FATs, root directory. */
    written = 1U + 1U + (FAT_COPIES * g.spf) + FAT16_ROOT_SECTORS;
    if (written > total_sectors) {
        return IMG_ERR_BAD_SIZE;
    }

    /* Free space. The image needs one cluster per 512 bytes of itself
     * (clusters are never smaller), so this is the tightest honest
     * check that does not need the host volume's cluster size - and
     * there is no slack term, because the metadata sectors are inside
     * the same count. */
    have = free_clusters (path);
    if ((have >= 0) && (have < (int64_t)total_sectors)) {
        printf ("[img] not enough space: need %u clusters, have %lld\r\n",
                (unsigned)total_sectors, (long long)have);
        return IMG_ERR_NO_SPACE;
    }

    printf ("[img] creating '%s': %u sectors (%u MiB) spc=%u spf=%u "
            "clusters=%u\r\n", path, (unsigned)total_sectors,
            (unsigned)(total_sectors / 2048U), (unsigned)g.spc,
            (unsigned)g.spf, (unsigned)g.clusters);

    fr = f_open (&fp, path, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf ("[img] f_open create failed (%u)\r\n", (unsigned)fr);
        return IMG_ERR_FATFS;
    }

    /* --- sector 0: the MBR --- */
    (void)memset (sector, 0, sizeof (sector));
    sector[MBR_JMP + 0U] = 0xEBU;
    sector[MBR_JMP + 1U] = 0xFEU;
    sector[MBR_JMP + 2U] = 0x90U;
    {
        static const char oem[8] = { 'N','E','X','T','O','3','0',' ' };
        (void)memcpy (&sector[MBR_OEM], oem, sizeof (oem));
    }
    sector[MBR_PARTITION + PT_STATUS]   = 0x80U;   /* active         */
    sector[MBR_PARTITION + PT_TYPE]     = 0x0EU;   /* FAT16, LBA     */
    store_le32 (&sector[MBR_PARTITION + PT_FIRST_LBA], 1U);
    store_le32 (&sector[MBR_PARTITION + PT_SECTOR_COUNT], part_sectors);
    /* The CHS fields at +447 and +451 stay zero, which is the
     * conventional "not representable in CHS" answer for anything past
     * 1024 cylinders - and fdisk.c leaves them zero too for the same
     * reason on a disk this size. */
    sector[MBR_SIGNATURE + 0U] = 0x55U;
    sector[MBR_SIGNATURE + 1U] = 0xAAU;
    fr = put_sector (&fp, 0U, sector);
    if (fr != FR_OK) {
        goto fail;
    }

    /* --- sector 1: the FAT16 boot sector --- */
    (void)memset (sector, 0, sizeof (sector));
    sector[BS_JMP + 0U] = 0xEBU;
    sector[BS_JMP + 1U] = 0xFEU;
    sector[BS_JMP + 2U] = 0x90U;
    {
        static const char oem[8] = { 'N','E','X','T','O','R','2','0' };
        static const char lbl[11] = { 'N','E','X','T','O','R',' ','2','.','0',' ' };
        static const char fst[8]  = { 'F','A','T','1','6',' ',' ',' ' };
        (void)memcpy (&sector[BS_OEM], oem, sizeof (oem));
        (void)memcpy (&sector[BS_VOLUME_LABEL], lbl, sizeof (lbl));
        (void)memcpy (&sector[BS_FAT_TYPE], fst, sizeof (fst));
    }
    store_le16 (&sector[BS_SECTOR_SIZE], (uint16_t)IMG_SECTOR_SIZE);
    sector[BS_SPC] = g.spc;
    store_le16 (&sector[BS_RESERVED], 1U);
    sector[BS_NUM_FATS] = (uint8_t)FAT_COPIES;
    store_le16 (&sector[BS_ROOT_ENTRIES], (uint16_t)FAT16_ROOT_ENTRIES);
    /* The 16-bit sector count is only valid when it fits; the 32-bit
     * field is always filled, which is what MSX-DOS actually uses for a
     * volume this size. */
    if (part_sectors <= 0xFFFFU) {
        store_le16 (&sector[BS_TOTAL_16], (uint16_t)part_sectors);
    }
    sector[BS_MEDIA] = FAT_HEAD_0;
    store_le16 (&sector[BS_SECTORS_PER_FAT], g.spf);
    /* sectors-per-track and heads stay 0: the image has no CHS
     * geometry, and the guide's answer for "not available" is zero. */
    store_le32 (&sector[BS_TOTAL_32], part_sectors);
    sector[BS_DRIVE_NUM]     = 0x80U;   /* hard disk, not a floppy */
    sector[BS_EXT_SIGNATURE] = 0x29U;
    /* Volume serial. Any value is legal; this is a fixed one so an
     * image is recognisable in a partition editor, and the low byte
     * carries the size in MiB so two sizes do not look identical. */
    store_le32 (&sector[BS_VOLUME_SERIAL],
                0x4E455854UL | ((uint32_t)(total_sectors / 2048U) & 0xFFU));
    sector[BS_SIGNATURE + 0U] = 0x55U;
    sector[BS_SIGNATURE + 1U] = 0xAAU;
    fr = put_sector (&fp, 1U, sector);
    if (fr != FR_OK) {
        goto fail;
    }

    /* --- the FATs and the root directory ---
     * Both FATs start at the reserved sector, so the second is one
     * sectorsPerFat further along. The first sector of each carries the
     * media descriptor and the two reserved cluster entries; everything
     * after it, and the whole root directory, is zero. A non-zero byte
     * anywhere in here is a cluster chain the filesystem driver will
     * follow, so this is the part of the image that genuinely has to be
     * written rather than merely allocated.
     *
     * Sector map of the file, `written` sectors in total:
     *
     *     0                    MBR
     *     1                    FAT16 boot sector (partition sector 0)
     *     2      .. 1+spf      FAT #1   (2 = sector 0, F0 FF FF FF)
     *     2+spf   .. 1+2spf    FAT #2   (2+spf = sector 0, same 4 bytes)
     *     2+2spf  .. written-1 root directory
     *     written .. end       the data area: allocated below, not written
     *
     * Everything that is not one of the four non-zero sectors above is
     * zero - including the whole of FAT #2, which is why the two FAT
     * heads are written separately rather than as one run. */
    (void)memset (sector, 0, sizeof (sector));
    sector[0] = FAT_HEAD_0;
    sector[1] = FAT_HEAD_1;
    sector[2] = FAT_HEAD_2;
    sector[3] = FAT_HEAD_3;
    fr = put_sector (&fp, 2U, sector);           /* FAT #1, sector 0 */
    if (fr != FR_OK) {
        goto fail;
    }
    fat_heads = 2U + g.spf;                      /* FAT #2, sector 0 */
    fr = put_sector (&fp, fat_heads, sector);
    if (fr != FR_OK) {
        goto fail;
    }
    if (g.spf >= 2U) {
        fr = put_zeros (&fp, 3U, g.spf - 1U);    /* rest of FAT #1 */
        if (fr != FR_OK) {
            goto fail;
        }
    }
    fr = put_zeros (&fp, fat_heads + 1U,
                    (g.spf - 1U) + FAT16_ROOT_SECTORS);
    if (fr != FR_OK) {
        goto fail;
    }

    /* --- the rest of the file: allocate, do not write ---
     * f_lseek in write mode walks the FAT one cluster at a time and
     * extends objsize to the new offset, so this costs the cluster
     * chain and nothing else. That is the whole reason creating a
     * 512 MiB image is a few seconds rather than the minutes that
     * writing 512 MiB of zeros through USB would take.
     *
     * The bytes behind those clusters are whatever the stick used to
     * hold. That is correct rather than sloppy: to the FAT inside the
     * image they are free clusters, and free clusters have no contents.
     *
     * Stepped in 8 MiB hops only so ImgImage_ProgressCB has something
     * to report - the FAT walk is identical either way, this is not a
     * chunked-write optimisation. */
    step = (8UL * 1024UL * 1024UL) / IMG_SECTOR_SIZE;
    for (pos = written; pos < total_sectors; pos += step) {
        uint32_t target = pos + step;
        if (target > total_sectors) {
            target = total_sectors;
        }
        fr = f_lseek (&fp, (FSIZE_t)target * (FSIZE_t)IMG_SECTOR_SIZE);
        if (fr != FR_OK) {
            printf ("[img] extend to sector %u failed (%u)\r\n",
                    (unsigned)target, (unsigned)fr);
            goto fail;
        }
        /* FatFs clips the file size instead of failing when the volume
         * runs out of clusters, so a short file is not an error as far
         * as f_lseek is concerned - and a short file is a small disk
         * that still automaps, which is a much worse outcome than no
         * disk. Check the length after every hop and stop the moment
         * it is not what was asked for. */
        if ((uint64_t)f_size (&fp)
            != (uint64_t)target * (uint64_t)IMG_SECTOR_SIZE) {
            printf ("[img] volume filled at sector %u of %u\r\n",
                    (unsigned)target, (unsigned)total_sectors);
            fr = FR_DISK_ERR;
            goto fail;
        }
        if (ImgImage_ProgressCB != (ImgProgressCB)0) {
            ImgImage_ProgressCB (target, total_sectors);
        }
    }

    fr = f_close (&fp);
    if (fr != FR_OK) {
        printf ("[img] f_close after create failed (%u)\r\n", (unsigned)fr);
        return IMG_ERR_FATFS;
    }
    printf ("[img] created '%s' (%u sectors, %u MiB)\r\n", path,
            (unsigned)total_sectors, (unsigned)(total_sectors / 2048U));
    return IMG_OK;

fail:
    printf ("[img] create '%s' failed (%u); removing the partial "
            "file\r\n", path, (unsigned)fr);
    (void)f_close (&fp);
    /* Leave nothing half-made behind: a truncated file would open as a
     * small disk and be automapped, which is a far worse failure than
     * no disk at all. */
    (void)f_unlink (path);
    return IMG_ERR_FATFS;
}

ImgErr ImgImage_Delete (const char *path) {
    FRESULT fr;

    if (path == (const char *)0 || path[0] == '\0') {
        return IMG_ERR_BAD_SIZE;
    }
    fr = f_unlink (path);
    if (fr != FR_OK) {
        printf ("[img] f_unlink '%s' failed (%u)\r\n", path, (unsigned)fr);
        return IMG_ERR_FATFS;
    }
    printf ("[img] deleted '%s'\r\n", path);
    return IMG_OK;
}
