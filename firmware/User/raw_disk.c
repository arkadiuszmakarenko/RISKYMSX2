/*
 * raw_disk.c - the USB stick as one raw block device.
 *
 * The data path is deliberately two calls deep:
 *
 *     RawDisk_ReadSectors -> usb_scsi_read_sector  (SCSI READ(10), 1 block)
 *                         -> USBHSH_SendEndpData / USBHSH_GetEndpData
 *
 * Nothing in between. No FatFs, no FIL, no f_lseek, no cluster cache. That
 * is the whole point of the module - see raw_disk.h for why the previous
 * file-per-device arrangement was replaced.
 *
 * See raw_disk.h for the API contract and raw_disk.c's own section notes
 * for the timing rules.
 */

#include "debug.h"
#include "usb_disk.h"
#include "raw_disk.h"

#if !RISKY_DEBUG
#define printf(...) do { } while (0)
#endif

/* ------------------------------------------------------------------------- */
/* State                                                                     */
/* ------------------------------------------------------------------------- */

static struct {
    uint8_t  probed;        /* RawDisk_Probe() has run at least once      */
    uint8_t  present;       /* capacity is valid and reads are expected to
                             * work                                          */
    uint8_t  changed;       /* media-change latch, see RawDisk_MediaState  */
    uint32_t sectors;       /* from READ CAPACITY                          */
    uint16_t sector_size;   /* from READ CAPACITY; must be 512            */

    /* INQUIRY strings, from raw_disk.c's own buffers so the pointers stay
     * valid for the lifetime of the module. */
    char     manufacturer[9];
    char     model[21];
} s;

/* Why the last transfer failed, in words.
 *
 * The callers above (nextor.c, and anything else that gets a 0 back) can
 * only report "the read failed". That is not a diagnosis: the three ways a
 * transfer fails here - the stick is gone, the LBA is out of range, the
 * SCSI command itself failed - have completely different fixes, and a log
 * that says only "ERR 2" once per retry per second cannot tell them apart.
 *
 * A pointer into a string literal rather than an enum: this is a log line,
 * not a control-flow value, and every new failure mode here would otherwise
 * need an enum, a switch and a name table to keep in step. Empty until
 * something actually fails, so a caller that checks it cannot mistake "no
 * failure recorded yet" for "no failure". */
static const char *s_last_failure = "";

/* ------------------------------------------------------------------------- */
/* Lifecycle                                                                 */
/* ------------------------------------------------------------------------- */

void RawDisk_Init (void) {
    RawDisk_Invalidate ();
    s.probed = 0U;
    s.sector_size = RAW_DISK_SECTOR_SIZE;
}

void RawDisk_Invalidate (void) {
    const uint8_t was_probed = s.probed;

    s.present     = 0U;
    s.sectors     = 0U;
    s.sector_size = RAW_DISK_SECTOR_SIZE;
    s.manufacturer[0] = '\0';
    s.model[0]        = '\0';

    /* Report the transition exactly once. Without this the kernel would keep
     * seeing "1 = ready" for a stick that is no longer there and would go on
     * reading sectors that cannot be read; with it set unconditionally on
     * every invalidate the drive would report a media change on every single
     * status poll, which is what makes the kernel unmap a drive for no
     * reason. So it is armed on absent->present and on present->absent, and
     * consumed by the STATUS command like every other latch. */
    if (was_probed != 0U) {
        s.changed = 1U;
    }
}

uint8_t RawDisk_HasProbed (void) {
    return s.probed;
}

/* ------------------------------------------------------------------------- */
/* Probe                                                                     */
/* ------------------------------------------------------------------------- */

/* Copy a fixed-width, space-padded SCSI INQUIRY string field into a
 * NUL-terminated C string, trimming trailing spaces.
 *
 * INQUIRY returns vendor as bytes 8..15 and product as 16..31, both padded
 * with spaces to the full width and NOT necessarily NUL-terminated. Handing
 * those bytes straight to a %s would run off the end of the buffer, which is
 * why this trims rather than copies. */
static void inq_copy (char *dst, const uint8_t *src, uint8_t width) {
    uint8_t n = 0U;
    uint8_t i;

    for (i = 0U; i < width; i++) {
        /* Anything outside printable ASCII is not a character a drive name
         * should carry; fold it to a space so the trim below catches it. */
        const uint8_t c = (uint8_t)(src[i] >= 0x20U && src[i] < 0x7FU)
                              ? src[i] : (uint8_t)' ';
        if (c == (uint8_t)' ') {
            break;
        }
        dst[n++] = (char)c;
    }
    dst[n] = '\0';
}

uint8_t RawDisk_Probe (void) {
    uint32_t blocks  = 0U;
    uint32_t bsize   = 0U;
    uint8_t  inq[36];

    s.probed = 1U;

    /* Enumeration may take hundreds of milliseconds on a freshly attached
     * stick, so this is a main-loop-only call. See raw_disk.h. */
    if (USB_TryEnsureEnumerated () != DEF_SUCCESS) {
        s.present = 0U;
        s.sectors = 0U;
        return 0U;
    }

    /* READ CAPACITY (10), falling back to (16) for a stick larger than 2 TiB
     * or one that refuses the 10-byte form. This is the "geometry and block
     * count come from USB enumeration" part of the design: the capacity is
     * the device's own answer, not a file length. */
    if (usb_scsi_read_capacity (&blocks, &bsize) != 0U) {
        printf ("[raw] READ CAPACITY failed\r\n");
        s.present = 0U;
        s.sectors = 0U;
        return 0U;
    }

    /* Only 512-byte blocks. The mailbox's result FIFO is sized for exactly
     * one 512-byte sector and driver.asm's transfer loop steps a 512-byte
     * counter, so there is no honest way to serve any other block size -
     * and no point pretending to. Reported as ABSENT rather than as a
     * broken capacity: the kernel keeps the drive and retries when a
     * 512-byte-capable stick turns up, which is strictly better than a
     * drive that reports a size it cannot deliver. Same policy as
     * FATFS/diskio.c's disk_initialize(). */
    if (bsize != RAW_DISK_SECTOR_SIZE) {
        printf ("[raw] unsupported block size %lu, need %u\r\n",
                (unsigned long)bsize, (unsigned)RAW_DISK_SECTOR_SIZE);
        s.present = 0U;
        s.sectors = 0U;
        return 0U;
    }

    /* A zero-block stick is not a disk either (an empty card reader, say). */
    if (blocks == 0U) {
        printf ("[raw] READ CAPACITY reported 0 blocks\r\n");
        s.present = 0U;
        s.sectors = 0U;
        return 0U;
    }

    s.sectors     = blocks;
    s.sector_size = (uint16_t)bsize;

    /* INQUIRY is decoration: a failure costs the drive its name in Nextor's
     * device screen and nothing else, so it must not fail the probe. Note
     * that a failed INQUIRY leaves the bulk pipe mid-transfer, which is why
     * the capacity read happens FIRST - it is the one that matters, and
     * putting the optional command after it means a rejected INQUIRY can
     * never cost us the capacity. */
    memset (inq, 0, sizeof (inq));
    if (usb_scsi_inquiry (inq, (uint16_t)sizeof (inq)) == 0U) {
        inq_copy (s.manufacturer, &inq[8],  8U);
        inq_copy (s.model,        &inq[16], 16U);
    } else {
        s.manufacturer[0] = '\0';
        s.model[0]        = '\0';
    }

    s.present = 1U;
    s.changed = 1U;      /* absent -> present: the kernel must re-scan */

    printf ("[raw] USB drive: %lu sectors (%lu KB), vendor='%s' product='%s'\r\n",
            (unsigned long)blocks,
            (unsigned long)(blocks / 2U),
            s.manufacturer, s.model);
    return 1U;
}

/* ------------------------------------------------------------------------- */
/* Cheap queries                                                             */
/* ------------------------------------------------------------------------- */

uint8_t RawDisk_IsPresent (void) {
    return (uint8_t)((s.present != 0U && s.sectors != 0U) ? 1U : 0U);
}

uint32_t RawDisk_SectorCount (void) {
    return s.sectors;
}

uint16_t RawDisk_SectorSize (void) {
    return s.sector_size;
}

void RawDisk_Geometry (uint16_t *cylinders, uint8_t *heads,
                       uint8_t *sectors_per_track) {
    /* 16 heads / 63 sectors per track: the split every PC BIOS uses, so the
     * numbers Nextor displays match what the same stick would report on a
     * real IDE port. Cylinders are the block count divided back down, and
     * CLAMPED - a 64 GB stick needs ~13 million cylinders, and the field is
     * two bytes.
     *
     * The clamp is honest rather than lossy because Nextor does not use
     * these bytes to address anything: this device is not flagged as a
     * floppy (see nextor.c), so the kernel's partition scanner ignores the
     * CHS triple completely and uses the 32-bit sector count. The field
     * exists to give the drive a plausible shape in the device screen. */
    const uint16_t h = 16U;
    const uint16_t spt = 63U;
    uint32_t cyl = s.sectors / ((uint32_t)h * (uint32_t)spt);

    if (cyl > 0xFFFFU) {
        cyl = 0xFFFFU;
    }
    if (cyl == 0U) {
        cyl = 1U;       /* a device smaller than one track still needs one */
    }
    if (cylinders != (uint16_t *)0) {
        *cylinders = (uint16_t)cyl;
    }
    if (heads != (uint8_t *)0) {
        *heads = (uint8_t)h;
    }
    if (sectors_per_track != (uint8_t *)0) {
        *sectors_per_track = (uint8_t)spt;
    }
}

uint8_t RawDisk_MediaState (void) {
    if (RawDisk_IsPresent () == 0U) {
        return RAW_MEDIA_NONE;
    }
    return (s.changed != 0U) ? RAW_MEDIA_CHANGED : RAW_MEDIA_READY;
}

void RawDisk_MediaLatchConsume (void) {
    s.changed = 0U;
}

const char *RawDisk_Manufacturer (void) {
    return s.manufacturer;
}

const char *RawDisk_Model (void) {
    return s.model;
}

/* ------------------------------------------------------------------------- */
/* Data path                                                                 */
/* ------------------------------------------------------------------------- */

/* Both transfer entry points validate before they touch the bus, and both
 * re-validate the LBA against the capacity. Two separate, cheap checks,
 * because the LBA here comes straight off the cart bus from the Z80: an
 * out-of-range request has to come back as an ordinary "no media"/"not
 * ready" answer the kernel already knows how to retry, not as a SCSI error
 * from a stick that is reading past its own end. */
static uint8_t range_ok (uint32_t lba, uint32_t count) {
    if (RawDisk_IsPresent () == 0U) {
        return 0U;
    }
    if (count == 0U) {
        return 1U;               /* nothing to do is not a failure */
    }
    /* lba + count must fit in the device. Written as the subtraction form
     * so a huge lba cannot wrap the sum into a small, apparently valid one. */
    if (lba >= s.sectors) {
        return 0U;
    }
    if (count > (s.sectors - lba)) {
        return 0U;
    }
    return 1U;
}

uint8_t RawDisk_ReadSectors (uint32_t lba, uint32_t count, uint8_t *dst) {
    uint32_t i;

    s_last_failure = "";

    if (dst == (uint8_t *)0) {
        s_last_failure = "null destination buffer";
        return 0U;
    }
    if (RawDisk_IsPresent () == 0U) {
        s_last_failure = "no medium (stick not enumerated, or not probed yet)";
        return 0U;
    }
    if (range_ok (lba, count) == 0U) {
        /* Naming the cause matters: "out of range" against a capacity that
         * is itself wrong (a READ CAPACITY off-by-one, a stick reporting 0
         * blocks) looks identical from the outside to a bad LBA, and this
         * is the only place that can tell them apart. */
        s_last_failure = "LBA out of range (check the reported capacity)";
        return 0U;
    }

    for (i = 0U; i < count; i++) {
        if (usb_scsi_read_sector (lba + i,
                                  dst + (i * RAW_DISK_SECTOR_SIZE),
                                  RAW_DISK_SECTOR_SIZE) != 0U) {
            s_last_failure = "SCSI READ(10) failed (see the USB: lines above)";
            return 0U;
        }
    }
    return 1U;
}

const char *RawDisk_LastFailure (void) {
    return s_last_failure;
}

uint8_t RawDisk_WriteSectors (uint32_t lba, uint32_t count,
                              const uint8_t *src) {
    uint32_t i;

    s_last_failure = "";

    if (src == (const uint8_t *)0) {
        s_last_failure = "null source buffer";
        return 0U;
    }
    if (RawDisk_IsPresent () == 0U) {
        s_last_failure = "no medium (stick not enumerated, or not probed yet)";
        return 0U;
    }
    if (range_ok (lba, count) == 0U) {
        s_last_failure = "LBA out of range (check the reported capacity)";
        return 0U;
    }

    for (i = 0U; i < count; i++) {
        if (usb_scsi_write_sector (lba + i,
                                   src + (i * RAW_DISK_SECTOR_SIZE),
                                   RAW_DISK_SECTOR_SIZE) != 0U) {
            s_last_failure = "SCSI WRITE(10) failed (see the USB: lines above)";
            return 0U;
        }
    }
    return 1U;
}
