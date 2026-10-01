/*
 * img_image.h - MSX .img image backend (writable, hard-disk shaped).
 *
 * The second Nextor device. Where dsk_image.c serves device 1 - one fixed
 * read-only 720 KiB FAT12 floppy image whose geometry comes out of a BPB
 * the file carries - this module serves device 2: a plain block device
 * whose entire content is "a file on the USB stick, addressed by LBA".
 *
 * Three differences from dsk_image, all deliberate:
 *
 *   1. No BPB parse. The capacity is the file length rounded down to a
 *      whole number of 512-byte sectors. Nextor is handed the disk
 *      exactly as the file holds it, and the kernel's own partition
 *      scan finds the MBR - the image is not a filesystem, it is a
 *      disk that happens to contain one.
 *
 *   2. Writable. Sectors go back into the file through f_lseek +
 *      f_write, and the handle is kept open across commands so a write
 *      costs the same as a read: one seek, one 512-byte transfer. The
 *      driver's READ_WRITE entry picks the direction and nothing else;
 *      the medium is the same either way.
 *
 *   3. Nothing is created implicitly. A missing NEXTOR.IMG means "no
 *      media" and the drive stays absent, which is the honest reading
 *      of a file that is not there. ImgImage_Create() below writes a
 *      Nextor-mountable image (MBR + FAT16, byte for byte the layout
 *      Nextor's own bank5/fdisk.c CreatePartition + CreateFatFileSystem
 *      produce) and is called from the terminal, never from a probe.
 *
 * Why a class again, and why not reuse dsk_image.c: the two backends
 * answer the same six questions (present? how many sectors? read? write?
 * media state? which file?) but compute them from completely different
 * things, and the writable one additionally has a constructor that has
 * no counterpart in the read-only one. Folding the differences into
 * flag tests inside dsk_image would put a "am I the floppy or the disk"
 * question in front of every read - and both nextor.c and the driver
 * then need the device number threaded through calls that have no use
 * for it. Two small objects, one table in nextor.c.
 *
 * Lifetime: ImgImage_Init once at boot, ImgImage_Open lazily from the
 * main loop (nextor.c's img_probe), the sector calls from anywhere on
 * the command path, ImgImage_Invalidate on stick removal, and
 * ImgImage_Create / ImgImage_Delete from the terminal's Nextor screen.
 * Main-loop only - none of this is IRQ-safe.
 */

#ifndef __IMG_IMAGE_H
#define __IMG_IMAGE_H

#include <stdint.h>
#include <ff.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Geometry
 *
 * Hardcoded sector size, for the same reason as dsk_image.h: the driver
 * is fixed at 512 (DO_DEVQ_GET_PARAMS fills +1/+2 with 0x0002 for both
 * devices) and the Nextor kernel refuses to automap any block device
 * whose sector size is not 512. There is no BPB to disagree with, so
 * this is the whole of the layout.
 * ====================================================================== */

#define IMG_SECTOR_SIZE       512U
#define IMG_SECTOR_SIZE_CODE  2U   /* the 0x0002 written into +1/+2 */

/* Smallest image ImgImage_Create() will build.
 *
 * The floor is FAT16, not a round number: Nextor's CHECK_FAT_BOOT
 * (bank4/partit.mac) rejects a boot sector with zero root directory
 * entries - that test is how it tells FAT32 from FAT16 - and MSX-DOS
 * identifies a volume as FAT12 below 4085 clusters and FAT16 from
 * there up to 65524 (partit.h MAX_FAT16_CLUSTER_COUNT). 16 MiB with
 * Nextor's own sectors-per-cluster table lands at 8171 clusters, in
 * the FAT16 window with room to spare, and is the smallest size the
 * terminal offers. */
#define IMG_MIN_SECTORS       32768U   /* 16 MiB */

/* Largest image ImgImage_Create() will build.
 *
 * 2 GiB: past that the BPB's 32-bit total-sector field is no longer the
 * only thing that matters, the MBR's own sector count (4 bytes, signed
 * in practice) becomes a real constraint, and nothing on this mapper
 * needs a disk that big. 512 MiB is the largest the terminal offers. */
#define IMG_MAX_SECTORS       (4UL * 1024UL * 1024UL)  /* 2 GiB in 512B sectors */

/* ========================================================================
 * Result codes
 * ====================================================================== */

typedef enum {
    IMG_OK                  =  0,
    IMG_ERR_NOT_MOUNTED     = -1,  /* stick not enumerated / not mounted   */
    IMG_ERR_NO_FILE         = -2,  /* no candidate name matched a file     */
    IMG_ERR_BAD_SIZE        = -3,  /* length is not a whole number of
                                       sectors, or out of the size range  */
    IMG_ERR_NO_SPACE        = -4,  /* not enough free clusters on the stick */
    IMG_ERR_FATFS           = -5,  /* an f_open / f_lseek / f_write failed */
    IMG_ERR_EXISTS          = -6,  /* Create: the file is already there    */
} ImgErr;

/* ========================================================================
 * Progress hook
 *
 * ImgImage_Create() can take tens of seconds on a 512 MiB image - the
 * cluster chain alone is a million FatFs entries - and the only thing
 * on screen is the terminal's Nextor screen. A caller that wants the
 * wait to look alive installs one of these; NULL (the default) is
 * silent, so the UART log stays the only output in the common case.
 *
 * (done, total) counts clusters linked, and is sampled a few times per
 * percent, so a callback must be cheap and must not re-enter FatFs. */
typedef void (*ImgProgressCB)(uint32_t done, uint32_t total);
extern ImgProgressCB ImgImage_ProgressCB;

/* ========================================================================
 * Public API
 * ====================================================================== */

/* Power-on state. Called once from main(), same lifetime as
 * Nextor_Init, and again on every swap to CART_MAP_NEXTOR. Unlike
 * DskImage_Init this closes a still-open handle first: a mapper swap is
 * not a power cycle, and dropping the FIL on the floor would leak the
 * object (and its cached sector) for the rest of the session. */
void ImgImage_Init (void);

/* Try to open the first available image for reading AND writing. Called
 * from the main loop while no request is in flight. Returns IMG_OK on
 * success, one of IMG_ERR_* on failure. Idempotent: returns IMG_OK
 * immediately if the image is already open.
 *
 * A file that opens but is not a whole number of 512-byte sectors, or
 * is empty, is rejected (IMG_ERR_BAD_SIZE) rather than presented as a
 * small disk: the most likely cause is a truncated copy, and a device
 * that reports 300 sectors of garbage produces a confusing kernel
 * error instead of a clear "no media".
 *
 * The IMG_MIN_SECTORS / IMG_MAX_SECTORS range is NOT enforced here -
 * it is a constraint on what ImgImage_Create() will build, not on what
 * may be opened. An image outside it is a user-supplied disk, and
 * refusing to open it would make a large working disk invisible rather
 * than merely unusual. */
ImgErr ImgImage_Open (void);

/* Drop the file handle and the cached size. Called from the USB
 * disconnect path and after ImgImage_Create / ImgImage_Delete, so the
 * next probe re-opens whatever is on the stick now. */
void ImgImage_Invalidate (void);

/* Was Open called at least once (regardless of outcome)? Gates the
 * media-change latch, so a missing image does not read as a change on
 * every STATUS. */
uint8_t ImgImage_HasProbed (void);

/* Image present and usable? Cheap, no I/O. */
uint8_t ImgImage_IsPresent (void);

/* Number of 512-byte sectors in the file (0 when no image). */
uint32_t ImgImage_SectorCount (void);

/* Exact file length in bytes, or 0 when no image. */
uint64_t ImgImage_FileSize (void);

/* Read one sector into dst / write one sector from src.
 *   lba   sector number, 0-based from the start of the image
 *   dst   caller-supplied buffer of IMG_SECTOR_SIZE bytes
 * Returns 1 on success, 0 on any failure (no image, LBA out of range,
 * FatFs error). On a failed read the destination is left unchanged.
 *
 * Both retry once, and a second consecutive failure drops the handle so
 * the next probe re-opens from the (possibly re-inserted) stick. That
 * is deliberately NOT the same as reporting a media change: the served
 * content has not changed, and a false media change is what makes the
 * kernel unmap a drive for no reason. */
uint8_t ImgImage_ReadSector (uint32_t lba, uint8_t *dst);
uint8_t ImgImage_WriteSector (uint32_t lba, const uint8_t *src);

/* Media-change latch. STATUS consumes it, STAPEEK must not. */
void    ImgImage_MediaLatchConsume (void);
uint8_t ImgImage_MediaState (void);   /* 0 absent / 1 ready / 2 changed */

/* Path of the file that actually opened, or NULL. Used by the boot log
 * and by the terminal's Nextor screen to name the source. NULL means
 * "not open", which is NOT the same as "no such file" - for the name
 * of the file this backend looks for, see ImgImage_PrimaryPath. */
const char *ImgImage_Path (void);

/* The FIRST candidate name, or NULL when no list has been set.
 *
 * The one to create and delete from outside. The terminal's Nextor
 * screen calls this instead of carrying a second copy of the path: a
 * second copy is a name the firmware would never open, which is a
 * silently dead menu entry rather than an error. */
const char *ImgImage_PrimaryPath (void);

/* FRESULT of the most recent underlying FatFs call (0 = FR_OK). */
uint8_t ImgImage_LastFR (void);

/* Candidate file names. Order matters - first match wins. */
void ImgImage_SetCandidates (const char *const *names, uint8_t count);

/* -------------------------------------------------------------------- *
 * Creating and removing the image
 * -------------------------------------------------------------------- */

/* Create `path` as a Nextor-mountable disk image of `total_sectors`
 * 512-byte sectors, and return IMG_OK.
 *
 * The layout is the one Nextor's own FDISK writes, reproduced from
 * bank5/fdisk.c so the kernel cannot tell the two apart:
 *
 *   sector 0            MBR, one active FAT16 LBA partition starting at
 *                       LBA 1 and running to the end of the file
 *   sector 1            FAT16 boot sector ("NEXTOR20", 512 B sectors,
 *                       2 FATs, 512 root entries, media F0, ext sig
 *                       29, "FAT16   ", 55AA at +510)
 *   sectors 2..         both FATs (first sector of each F0 FF FF FF,
 *                       the rest zero) and the zeroed root directory
 *   the remainder       allocated but unwritten
 *
 * The remainder is allocated by seeking past the end of the fresh file:
 * f_lseek in write mode walks the FAT and grows the cluster chain, and
 * objsize is set to the new offset - so a 512 MiB image costs 512 MiB
 * of free space on the stick and no more than a second of CPU, instead
 * of the half a minute of zero-filling that writing the file out would
 * cost. The bytes behind those clusters are whatever the stick had, and
 * that is correct: to the FAT inside the image they are free clusters,
 * and to a filesystem that is what unread sectors are.
 *
 * Fails with IMG_ERR_NO_SPACE if the volume cannot supply the clusters,
 * IMG_ERR_EXISTS if the file is already there (this never overwrites a
 * disk the user may have data on - delete it first), and IMG_ERR_BAD_SIZE
 * if the requested size is outside the FAT16 window.
 *
 * Does not open the result for serving; call ImgImage_Invalidate()
 * afterwards if an image was already open. */
ImgErr ImgImage_Create (const char *path, uint32_t total_sectors);

/* Remove `path`. Returns IMG_OK, or the FatFs error that stopped it. */
ImgErr ImgImage_Delete (const char *path);

/* -------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __IMG_IMAGE_H */
