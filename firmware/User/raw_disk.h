/*
 * raw_disk.h - the USB stick as ONE raw block device, with no filesystem
 *              anywhere in the data path.
 *
 * ===========================================================================
 * Why this module exists
 * ===========================================================================
 *
 * The previous arrangement gave Nextor two devices, and each one was a FILE
 * on the stick, served sector by sector through FatFs:
 *
 *   device 1   NEXTOR.DSK - a read-only floppy image, f_lseek + f_read
 *   device 2   NEXTOR.IMG - a read-write disk image, f_lseek + f_read/write
 *
 * That is a container format, not a disk. It costs an f_lseek, an f_open and
 * a FatFs sector-cache lookup on every single 512-byte transfer, it cannot
 * express a partition table the user did not think to build by hand, and it
 * makes the capacity a file length instead of a property of the hardware.
 *
 * The USB stick IS already a block device. The MSC layer in
 * FATFS/usb_disk.c can do SCSI READ CAPACITY, READ(10) and WRITE(10)
 * against it - those functions are what FatFs's own disk_read/disk_write sit
 * on top of, and they know nothing about files. So this module skips FatFs
 * entirely and hands the raw medium to Nextor, which is exactly the shape of
 * the hardware an IDE driver sees:
 *
 *   LBA 0                 MBR / partition table
 *   partition N first LBA FAT boot sector
 *   the FAT volume itself
 *
 * which is also what the reference Sunrise IDE driver for Nextor v3 does, and
 * the reason driver.asm is now written in that driver's shape. Nextor's own
 * partition scanner takes it from there.
 *
 * ===========================================================================
 * What comes from where
 * ===========================================================================
 *
 *   capacity, sector size      SCSI READ CAPACITY (10/16), i.e. from USB
 *                              enumeration - no file length, no guessing
 *   device name                SCSI INQUIRY vendor + product strings
 *   sectors, heads, cylinders  SYNTHESISED, see RawDisk_Geometry()
 *
 * The CHS triple is the one thing that has no honest source here: READ
 * CAPACITY reports only a block count, and no flash stick has a real CHS
 * geometry. Nextor only uses +8..+11 (cylinders/heads/sectors-per-track) to
 * display a size and, on the floppy path, to decide a device is a floppy;
 * this device is never flagged as a floppy, so the kernel's partition scan
 * ignores those bytes entirely. We report the conventional 16 heads / 63
 * sectors-per-track split that every PC BIOS uses, which makes the drive
 * show up with sane numbers in Nextor's device screen.
 *
 * ===========================================================================
 * Sector size
 * ===========================================================================
 *
 * 512 only. The mailbox carries one sector in a fixed-size buffer and
 * driver.asm's transfer loop is built for 512-byte blocks, so a stick that
 * reports anything else is reported as ABSENT rather than half-supported -
 * which is also what FatFs's disk_initialize() already does with it.
 */

#ifndef __RAW_DISK_H
#define __RAW_DISK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bytes per sector. Compile-time constant, and the reason raw_disk.c
 * rejects a stick whose READ CAPACITY block size is not this. */
#define RAW_DISK_SECTOR_SIZE   512U

/* Media state codes, matching what the mailbox STATUS command returns and
 * what Nextor device query 3 expects:
 *   0 = no medium, 1 = present and unchanged, 2 = present and changed
 * (the "changed" is consumed by RawDisk_MediaLatchConsume)
 */
#define RAW_MEDIA_NONE          0U
#define RAW_MEDIA_READY         1U
#define RAW_MEDIA_CHANGED       2U

/* --- Lifecycle ------------------------------------------------------------ */

/* Force a re-probe (drop the cached capacity and the INQUIRY strings, mark
 * the device absent). Called at boot and whenever the USB layer reports a
 * disconnect. */
void RawDisk_Invalidate (void);

/* Zero the whole module. Called from Nextor_Init(). */
void RawDisk_Init (void);

/* True once RawDisk_Probe() has run at least once, whatever its outcome.
 * The probe scheduler uses this to distinguish "never tried" from "tried
 * and failed", so a device that has not yet been probed is not locked out
 * by the retry rate limit. */
uint8_t RawDisk_HasProbed (void);

/* Enumerate the stick and read its capacity. Returns 1 on success (capacity
 * is then valid and IsPresent() returns 1), 0 on failure.
 *
 * EXPENSIVE - it can take hundreds of milliseconds because it may have to
 * enumerate a freshly plugged device. Callers MUST run it from the main
 * loop with no mailbox request in flight, never from a command handler. */
uint8_t RawDisk_Probe (void);

/* --- Cheap queries (all safe from a command handler) --------------------- */

uint8_t  RawDisk_IsPresent (void);
uint32_t RawDisk_SectorCount (void);
uint16_t RawDisk_SectorSize (void);

/* Synthesised CHS geometry. Any output pointer may be NULL. */
void RawDisk_Geometry (uint16_t *cylinders, uint8_t *heads,
                       uint8_t *sectors_per_track);

/* 0 = no medium / error, 1 = ready / 2 = changed (see RAW_MEDIA_*). */
uint8_t RawDisk_MediaState (void);
void    RawDisk_MediaLatchConsume (void);

/* The stick's INQUIRY strings, NUL-terminated. Never NULL: returns "" until
 * the first successful probe. */
const char *RawDisk_Manufacturer (void);
const char *RawDisk_Model (void);

/* Why the most recent RawDisk_ReadSectors / RawDisk_WriteSectors failed, in
 * words: "", "no medium (...)", "LBA out of range (...)",
 * "SCSI READ(10) failed (...)". A log line, not a control-flow value.
 *
 * Empty when the last transfer succeeded OR when nothing has been attempted
 * yet - the two are indistinguishable here, which is deliberate: a caller
 * only reads this AFTER a transfer has returned 0, so "empty" is never the
 * answer it is asking about. The contents are only valid until the next
 * transfer call. */
const char *RawDisk_LastFailure (void);

/* --- Data path ------------------------------------------------------------ */

/* Read `count` consecutive 512-byte sectors into `dst`. Returns 1 on
 * success, 0 on any failure (no medium, LBA out of range, SCSI error).
 * `dst` must have room for count * RAW_DISK_SECTOR_SIZE bytes. */
uint8_t RawDisk_ReadSectors (uint32_t lba, uint32_t count, uint8_t *dst);

/* As above for writes. The medium is writable: this is the whole point of
 * exposing the raw drive. */
uint8_t RawDisk_WriteSectors (uint32_t lba, uint32_t count,
                              const uint8_t *src);

#ifdef __cplusplus
}
#endif

#endif /* __RAW_DISK_H */