/*
 * dsk_image.h - MSX .dsk image backend (FAT12 2DD floppy, 720 KiB).
 *
 * A small class that owns:
 *
 *   - the open FIL handle (lazy mount + f_open of one of a short list
 *     of candidate SFN paths on the USB stick);
 *   - the BPB parse (bytes/sector, sectors/cluster, reserved sectors,
 *     FAT count, root entries, total sectors, media descriptor,
 *     sectors-per-FAT, geometry) so nextor.c / the driver never have
 *     to know what a BPB is;
 *   - the FAT12 chain read for root-directory / cluster lookup - the
 *     kernel walks directory entries and FatFs's directory read path
 *     costs two sector reads per entry, which the BPB-aware path
 *     collapses to one;
 *   - the media-change latch and the image-present bit consumed by
 *     CMD_STATUS / CMD_STAPEEK.
 *
 * Why a class (instead of plain functions in nextor.c): the kernel
 * polls CAPACITY / STATUS hundreds of times during a single sector
 * load, and every one of those asks "what geometry?" "what media?".
 * nextor.c is the wrong place for that - it has no variable to put it
 * in except as a private struct - and the driver has no RAM of its
 * own to cache it in. A small object with parse-on-open / cheap-getters
 * keeps nextor.c focused on the mailbox protocol and lets the BPB
 * evolve (DSK with a 1 MiB / 1.44 MiB layout, or even a future hard-
 * disk image) without touching the mailbox code.
 *
 * Lifetime: DskImage_Init once at boot, DskImage_Open lazily from the
 * main loop (DskImage_Service), DskImage_Sector / DskImage_Geometry
 * from anywhere, DskImage_Close + DskImage_Invalidate on stick
 * removal. Reentrant against EXTI0 (no calls from IRQ context, but
 * reads from the main loop are safe alongside IRQ writes to FAT
 * caches because none of those caches are touched by the IRQ path).
 */

#ifndef __DSK_IMAGE_H
#define __DSK_IMAGE_H

#include <stdint.h>
#include <ff.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Geometry (parsed from the .dsk BPB at open time).
 *
 * Hardcoded sector size: every MSX-DOS floppy image in the wild is
 * 512 B/sector. The driver is fixed at 512 too (see DO_DEVQ_GET_PARAMS
 * in MSXSoftware/NextorDriver/driver.asm, which fills +1/+2 with 0x0002
 * unconditionally). Parsing bytes-per-sector is therefore purely a
 * sanity check, not a knob.
 * ====================================================================== */

#define DSK_SECTOR_SIZE       512U
#define DSK_SECTOR_SIZE_CODE  2U   /* BPB "bytes per sector" field, 2 = 512 */

typedef struct {
    uint16_t bytes_per_sector;   /* always 512 for now */
    uint8_t  sectors_per_cluster;
    uint16_t reserved_sectors;
    uint8_t  num_fats;
    uint16_t root_entry_count;
    uint16_t total_sectors_16;   /* BPB field, 0 if total > 65535      */
    uint32_t total_sectors;     /* 16-bit or 32-bit (BPB field @+32)   */
    uint8_t  media_descriptor;
    uint16_t sectors_per_fat;
    uint16_t sectors_per_track;
    uint16_t heads;
    uint32_t fat_start_lba;      /* = reserved_sectors (BPB field @+14) */
    uint32_t root_dir_start_lba; /* fat_start + num_fats * sectors_per_fat */
    uint32_t data_start_lba;     /* root_dir_start + root_entry_count * 32 / 512 */
    uint32_t root_dir_sectors;   /* = ceil(root_entry_count * 32 / 512) */
    uint32_t cluster_count;      /* data clusters, = (total - data_start) / spc */
} DskGeometry;

/* ========================================================================
 * BPB-parse result codes. Returned by DskImage_Open and the parse step.
 * ====================================================================== */

typedef enum {
    DSK_OK                  =  0,
    DSK_ERR_NOT_MOUNTED     = -1,   /* stick not enumerated / not mounted  */
    DSK_ERR_NO_FILE         = -2,   /* no candidate name matched a file   */
    DSK_ERR_BAD_BPB         = -3,   /* BPB fields inconsistent for FAT12   */
    DSK_ERR_TRUNCATED       = -4,   /* file shorter than BPB says          */
    DSK_ERR_FATFS           = -5,   /* underlying f_open / f_lseek / f_read
                                      * failed; call site should close and
                                      * let the next probe retry            */
} DskErr;

/* ========================================================================
 * Public API
 * ====================================================================== */

/* Power-on state. Called once from main(), same lifetime as Nextor_Init. */
void DskImage_Init (void);

/* Try to open + parse the first available .dsk image. Called from the
 * main loop while no request is in flight. Returns DSK_OK on success,
 * one of DSK_ERR_* on failure. Idempotent: returns DSK_OK immediately
 * if the image is already open. */
DskErr DskImage_Open (void);

/* Drop the file handle and reset parsed geometry. Called from
 * usb_disk.c's disconnect path; safe to call from anywhere outside
 * IRQ context. */
void DskImage_Invalidate (void);

/* Was Open called at least once (regardless of outcome)? Used to gate
 * the media-change latch so a missing image does not show as a change
 * on every STATUS. */
uint8_t DskImage_HasProbed (void);

/* Image present and ready? Cheap query, no I/O. */
uint8_t DskImage_IsPresent (void);

/* Number of 512-byte sectors reported to the driver (CMD_CAPACITY).
 * 0 when no image. */
uint32_t DskImage_SectorCount (void);

/* Copy out the parsed geometry. Cheap, no I/O. dst may be NULL. */
const DskGeometry *DskImage_Geometry (void);

/* Read one 512-byte sector into dst.
 *   lba   sector number (0-based from start of image)
 *   dst   caller-supplied buffer of DSK_SECTOR_SIZE bytes
 * Returns 1 on success, 0 on any failure (no image, LBA out of range,
 * FatFs error). On failure the buffer is left unchanged. */
uint8_t DskImage_ReadSector (uint32_t lba, uint8_t *dst);

/* Read the first DSK_SECTOR_SIZE bytes of the boot sector (for
 * diagnostics). Returns 1 on success, 0 on no image / I/O failure. */
uint8_t DskImage_ReadBootSector (uint8_t *dst);

/* Media-change latch control. The driver (CMD_STATUS) consumes it;
 * CMD_STAPEEK must observe without consuming. */
void     DskImage_MediaLatchConsume (void);
uint8_t  DskImage_MediaState (void);   /* 0 absent / 1 ready / 2 changed */

/* String of the candidate path that actually opened, or NULL if no
 * image is open. Used by the boot log to name the source. NULL means
 * "not open", not "no such file" - for the name of the file this
 * backend looks for, see DskImage_PrimaryPath. */
const char *DskImage_Path (void);

/* The FIRST candidate name, or NULL when no list has been set. Read
 * only - device 1 is never written, so there is nothing to create or
 * delete. The terminal's Nextor screen shows it so the name is
 * answerable without a second copy of the list in the UI. */
const char *DskImage_PrimaryPath (void);

/* FRESULT of the most recent underlying f_open / f_lseek / f_read
 * call. 0 = FR_OK. Used by the diagnostic log to explain failures. */
uint8_t DskImage_LastFR (void);

/* Candidate file names. Order matters - first match wins. */
void DskImage_SetCandidates (const char *const *names, uint8_t count);

/* -------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __DSK_IMAGE_H */
