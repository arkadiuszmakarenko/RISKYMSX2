/*
 * sunrise_ide.c - Sunrise IDE emulation for the RISKYMSX2 cartridge.
 *
 * Ported from PicoVerse 2040's sunrise_ide.c (cristianoag/msx-picoverse-
 * public). The cart-window decoder (control register, bit-reverse, IDE
 * gating) lives in cart.c::Cart_EXTI0_Sunride_Handler; this file owns
 * the ATA state machine and the FILE backing store.
 *
 * REWORK: the IDE bus is no longer backed by direct SCSI access to the
 * USB stick (that path proved unreliable on real hardware). Instead a
 * fixed-size disk image is kept in a FAT file called nextor.img in the
 * root of the stick, accessed through the well-tested FATFS glue
 * (usb_disk.c + ff.c + diskio.c). Geometry is HARD-CODED below - the
 * file size itself is irrelevant; the firmware extends the file with
 * zeros on first mount and the IDENTIFY response always reports the
 * same capacity.
 *
 * Threading model (real hardware: the MSX drives the cart bus from
 * EXTI0 interrupts):
 *   - IRQ context (CartExti handler): Sunrise_IDE_ReadByte /
 *     Sunrise_IDE_WriteByte only. No FatFs, no printf, no blocking.
 *   - Main loop: Sunrise_IDE_Service does ALL file I/O (mount, open,
 *     extend, f_lseek/f_read/f_write/f_sync for in-flight transfers).
 *     The MSX polls STATUS during this, so BSY must be held until the
 *     file operation completes - exactly how a slow real IDE drive
 *     behaves.
 *
 * See sunrise_ide.h for the cart memory map and overall design.
 */

#include "sunrise_ide.h"
#include "usb_disk.h"
#include "debug.h"
#include <string.h>

#pragma GCC push_options
#pragma GCC optimize("Os")

/* ========================================================================
 * Disk geometry - HARD-CODED, imaginary drive
 * ======================================================================
 *
 * The whole point of this rework: sizes are NOT probed from the stick
 * or from the file. The emulated drive is always exactly
 * ATA_DISK_SECTORS sectors of 512 bytes, addressed with LBA28 (LBA48
 * commands are accepted but any address beyond the capacity fails
 * with IDNF - the capacity is far below 2^28 sectors). The image file
 * nextor.img is zero-extended to exactly this size on first mount;
 * a larger existing file is truncated to it.
 *
 * 32 MiB was chosen because it is FAT16-happy and fast to create, but
 * you can bump ATA_DISK_SECTORS freely - nothing else depends on the
 * value (IDENTIFY, LBA range checks and file size all derive from it).
 */

#define SUNRISE_IMG_PATH       "0:/nextor.img"

#define ATA_DISK_SECTORS       65536UL   /* 32 MiB (LBA28 space) */
#define ATA_DISK_BYTES         (ATA_DISK_SECTORS * 512UL)

/* IDENTIFY CHS geometry - the standard ">8 GB" fake geometry every
 * modern drive reports. Nextor only uses LBA mode; this exists so
 * generic ATA tools see something sane. */
#define ATA_IDENT_CYLINDERS    16383U
#define ATA_IDENT_HEADS        16U
#define ATA_IDENT_SPT          63U

/* Zero-extension budget per main-loop pass (keeps the console and
 * the terminal IRQ service responsive during the one-time image
 * expansion; 16 KiB/pass finishes 32 MiB in ~2K passes). */
#define ATA_IMG_EXTEND_CHUNK   16384U

/* ========================================================================
 * Module state
 * ====================================================================== */

/* Single global state struct. Lives in regular SRAM (no .ramfunc - the
 * cart IRQ handler is the only consumer and it just reads / writes a
 * few fields per cycle, no DMA, no large tables). */
static Sunrise_IDE s_ide;

/* FatFs handle for the image file. Used from main-loop context ONLY
 * (Sunrise_IDE_Service); the IRQ handlers never touch it. */
static FIL     s_img;
static uint8_t s_img_open;

/* Zero-extension cursor (offset into the image already zero-filled). */
static uint32_t s_ext_off;
/* One-shot debug throttle so a failing mount doesn't flood the log. */
static uint8_t  s_open_printed;

/* Scratch buffer for the zero-extension write (bss). */
static uint8_t s_zeros[ATA_IMG_EXTEND_CHUNK];

/* Embedded Nextor kernel image (128 KiB max, served from flash).
 * The actual bytes are produced by tools/rom2c.py from the kernel
 * binary the user provides - see Makefile. */
extern const uint8_t  nextor_rom[];
extern const uint32_t nextor_rom_len;

/* ========================================================================
 * Forward declarations (used internally)
 * ====================================================================== */

static void ide_set_signature (void);
static void ide_set_status (uint8_t status);
static void ide_set_error (uint8_t error);
static uint32_t ide_get_lba (void);
static void ide_set_lba (uint32_t lba);
static void ide_advance_lba (void);
static void ide_execute_command (uint8_t cmd);
static void ide_on_read_sectors (uint8_t ext);
static void ide_on_write_sectors (uint8_t ext);
static void ide_on_read_verify (uint8_t ext);
static void ide_on_identify (void);
static void ide_on_device_reset (void);
static void ide_on_init_dev_params (void);
static void ide_on_set_features (uint8_t features);
static void ide_on_read_buffer (void);
static void ide_start_file_error (void);
static void build_identify_data (void);
static int  usb_state_step (void);
static void store_le16 (uint8_t *p, uint16_t v);
static void store_le32 (uint8_t *p, uint32_t v);

/* ========================================================================
 * Lifecycle / state setup
 * ====================================================================== */

void Sunrise_IDE_Init (void) {
    memset (&s_ide, 0, sizeof (s_ide));
    s_ide.reg_status    = ATA_STATUS_DRDY;
    s_ide.reg_device_ctrl = 0U;
    /* Bank 0 is the kernel entry - pre-select it so the BIOS slot
     * probe finds 'AB' at 0x4000 immediately after the mapper swap.
     * The MSX-side kernel will issue further 0x4104 writes to switch
     * banks; bit-reverse happens inside the handler. */
    s_ide.segment       = 0U;
    s_ide.ide_enabled   = 1U;
    s_ide.sectors_remaining = 0U;
    s_ide.usb_state     = SUNRISE_IDE_USB_INIT;
    /* IDENTIFY data is now fully static (fixed geometry) so build it
     * once here - the PRECHECK path reads it from the IDLE data
     * register and there is no USB capacity probe step anymore. */
    build_identify_data ();
    ide_set_signature ();
}

static void ide_set_signature (void) {
    s_ide.reg_error          = ATA_SIG_ERROR;
    s_ide.reg_sector_count   = ATA_SIG_SECTOR_COUNT;
    s_ide.reg_cylinder_low   = ATA_SIG_CYLINDER_LOW;
    s_ide.reg_cylinder_high  = ATA_SIG_CYLINDER_HIGH;
    s_ide.reg_device_head    = ATA_SIG_DEVICE_HEAD;
    s_ide.reg_status         = ATA_SIG_STATUS;
    /* Also clear the LBA48 shadow regs so an EXT command right after
     * a reset does not inherit stale previous-content bytes. */
    s_ide.hob                = 0U;
    s_ide.sh_sector_count    = 0U;
    s_ide.sh_sector_number   = 0U;
    s_ide.sh_cylinder_low    = 0U;
    s_ide.sh_cylinder_high   = 0U;
}

static void ide_set_status (uint8_t status) {
    /* Straight latch - never clear bits the device is asserting;
     * this is a latch, not an OR-with-zero. */
    s_ide.reg_status = status;
}

static void ide_set_error (uint8_t error) {
    s_ide.reg_error = error;
}

static uint32_t ide_get_lba (void) {
    /* LBA = (DEV_HEAD.LBA[27:24] << 24) | (CYL_HIGH << 16) |
     *       (CYL_LOW << 8) | SECTOR_NUMBER. The LBA bit (0x40) of
     * device_head MUST be set for this to be valid; the kernel sets
     * it on every transaction so we trust it. */
    const uint32_t hi =
        ((uint32_t)(s_ide.reg_device_head & 0x0FU) << 24);
    const uint32_t mid =
        ((uint32_t)s_ide.reg_cylinder_high << 16) |
        ((uint32_t)s_ide.reg_cylinder_low  << 8)  |
        (uint32_t)s_ide.reg_sector_number;
    return hi | mid;
}

static void ide_set_lba (uint32_t lba) {
    s_ide.reg_device_head =
        (uint8_t)((s_ide.reg_device_head & 0xF0U) |
                  ATA_DEV_HEAD_LBA_BIT | (uint8_t)((lba >> 24) & 0x0FU));
    s_ide.reg_cylinder_high = (uint8_t)((lba >> 16) & 0xFFU);
    s_ide.reg_cylinder_low  = (uint8_t)((lba >> 8)  & 0xFFU);
    s_ide.reg_sector_number = (uint8_t)(lba & 0xFFU);
}

static void ide_advance_lba (void) {
    const uint32_t lba = ide_get_lba ();
    if (lba + 1U < ATA_DISK_SECTORS) {
        ide_set_lba (lba + 1U);
    } else {
        /* Past the end of disk: leave LBA unchanged, the next read
         * will return IDNF. The kernel counts sectors via
         * reg_sector_count and stops before the LBA rolls over the
         * edge. */
    }
}

/* ========================================================================
 * File backend helpers (main-loop context ONLY)
 * ====================================================================== */

/* Drop back to the MOUNT step (e.g. FatFs reports a disk error - the
 * stick may have been pulled). Any open file handle is closed first. */
static void file_backend_reset (void) {
    if (s_img_open) {
        (void)f_close (&s_img);
        s_img_open = 0U;
    }
    s_ext_off = 0U;
    s_ide.usb_state = SUNRISE_IDE_USB_ENUM;
}

static uint8_t img_read_sector (uint32_t lba) {
    UINT br = 0U;
    FRESULT fr;
    for (int attempt = 0; attempt < 2; attempt++) {
        fr = f_lseek (&s_img, (FSIZE_t)lba * 512U);
        if (fr != FR_OK) continue;
        fr = f_read (&s_img, s_ide.sector_buffer, 512U, &br);
        if (fr == FR_OK && br == 512U) return 0U;
    }
    printf ("SUNRIDE: read lba=%lu failed fr=%u br=%u\r\n",
            (unsigned long)lba, (unsigned)fr, (unsigned)br);
    return 1U;
}

static uint8_t img_write_sector (uint32_t lba) {
    UINT bw = 0U;
    FRESULT fr;
    for (int attempt = 0; attempt < 2; attempt++) {
        fr = f_lseek (&s_img, (FSIZE_t)lba * 512U);
        if (fr != FR_OK) continue;
        fr = f_write (&s_img, s_ide.sector_buffer, 512U, &bw);
        if (fr == FR_OK && bw == 512U) return 0U;
    }
    printf ("SUNRIDE: write lba=%lu failed fr=%u bw=%u\r\n",
            (unsigned long)lba, (unsigned)fr, (unsigned)bw);
    return 1U;
}

/* ========================================================================
 * Main-loop service: drive the file lifecycle + IDE state machine.
 *
 * Called from main()'s main loop. Cannot block on EXTI0 (no spin
 * loops on the bus side) - but FatFs calls ARE blocking here, which
 * is fine: the MSX-side driver polls STATUS (IRQ-served) and only
 * proceeds when BSY drops, exactly as with a slow real IDE drive.
 * ====================================================================== */

void Sunrise_IDE_Service (void) {
    /* File backend lifecycle: each pass either advances to the next
     * state or stays put (usb_state_step returns 1 if it advanced). */
    (void)usb_state_step ();

    if (s_ide.usb_state != SUNRISE_IDE_USB_READY) {
        /* Image not mounted - nothing else to do this pass. */
        return;
    }

    /* In-flight READ SECTORS: if lba_pending is set, the IRQ handler
     * just queued a file read (ide_on_read_sectors or the
     * multi-sector advance in Sunrise_IDE_ReadByte). Perform the
     * FatFs read here - blocking, but the MSX polls BSY/DRQ in a
     * tight interrupt-driven loop, so the kernel sees BSY drop only
     * after this returns.*/
    if (s_ide.state == SUNRISE_IDE_STATE_READ_BUSY &&
        s_ide.lba_pending != 0U) {
        const uint32_t lba = ide_get_lba ();
        uint8_t rc = 1U;
        if (lba < ATA_DISK_SECTORS) {
            rc = img_read_sector (lba);
        }
        s_ide.lba_pending = 0U;
        s_ide.buffer_index = 0U;
        if (rc != 0U) {
            /* File read failed: assert ERR + ABRT + DF (Drive
             * Fault). The driver checks bit 5 (DF) via AND 0x20 to
             * detect errors, NOT bit 0 (ERR). Without DF set, the
             * driver thinks the read succeeded and LDIRs stale
             * buffer data. */
            ide_start_file_error ();
        } else {
            s_ide.state = SUNRISE_IDE_STATE_READY;
            ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                            ATA_STATUS_DRQ);
        }
    }

    /* In-flight WRITE SECTORS: drain remaining data into the buffer
     * until the MSX has finished the 512-byte PIO write, then commit
     * to the image file. */
    if (s_ide.state == SUNRISE_IDE_STATE_WRITE_BUSY &&
        s_ide.buffer_index >= s_ide.buffer_length) {
        uint8_t ok = 1U;
        if (s_ide.sectors_remaining > 0U) {
            const uint32_t lba = ide_get_lba ();
            if (lba < ATA_DISK_SECTORS) {
                ok = (img_write_sector (lba) == 0U);
            } else {
                ok = 0U;
            }
        }
        if (!ok) {
            ide_start_file_error ();
            (void)f_sync (&s_img);   /* commit whatever did land */
            return;
        }
        ide_advance_lba ();
        if (s_ide.sectors_remaining > 1U) {
            /* More sectors in this transfer: decrement, arm the
             * buffer for the next PIO burst and keep DRQ asserted
             * (spec-correct for WRITE MULTIPLE; for plain WRITE
             * SECTORS the driver re-reads STATUS and waits for DRQ
             * anyway). */
            s_ide.sectors_remaining--;
            s_ide.buffer_index = 0U;
            /* stay SUNRISE_IDE_STATE_WRITE_BUSY, DRQ stays set */
        } else {
            /* Transfer complete: flush the FAT chain / data to the
             * stick so a power cycle can't corrupt the cluster chain
             * of patches the kernel wrote a moment ago. */
            (void)f_sync (&s_img);
            s_ide.state = SUNRISE_IDE_STATE_IDLE;
            s_ide.sectors_remaining = 0U;
            ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        }
    }
}

static void ide_start_file_error (void) {
    ide_set_error (ATA_ERR_ABRT);
    s_ide.state = SUNRISE_IDE_STATE_IDLE;
    s_ide.sectors_remaining = 0U;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                    ATA_STATUS_ERR | ATA_STATUS_DF);
}

/* ========================================================================
 * File-backend lifecycle state machine
 * ====================================================================== */

static int usb_state_step (void) {
    switch (s_ide.usb_state) {
    case SUNRISE_IDE_USB_INIT:
        /* Nothing to do - the host stack was already initialised by
         * USB_Initialization() from main(). Move to MOUNT and let
         * USB_TryEnsureMounted (which itself runs the full enum +
         * f_mount retry ladder) do the work. */
        s_ide.usb_state = SUNRISE_IDE_USB_ENUM;
        return 1;

    case SUNRISE_IDE_USB_ENUM:
        /* MOUNT: poll the root hub, enumerate the stick and mount
         * the FAT volume (all inside USB_TryEnsureMounted, with
         * retries). Only move on when it reports DEF_SUCCESS. */
        if (USB_TryEnsureMounted () != DEF_SUCCESS) {
            return 0;   /* stay in ENUM, retry next pass */
        }
        s_ide.usb_state = SUNRISE_IDE_USB_OPEN;
        return 1;

    case SUNRISE_IDE_USB_OPEN:
        /* Open (or create) the fixed-name image file. */
        {
            FRESULT fr = f_open (&s_img, SUNRISE_IMG_PATH,
                                 FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
            if (fr != FR_OK) {
                if (s_open_printed == 0U) {
                    printf ("SUNRIDE: f_open('%s') failed (%u)\r\n",
                            SUNRISE_IMG_PATH, (unsigned)fr);
                    s_open_printed = 1U;
                }
                /* The volume may have gone away; retry from ENUM. */
                file_backend_reset ();
                return 1;
            }
            s_img_open = 1U;
            s_open_printed = 0U;
            s_ext_off = 0U;
            s_ide.usb_state = SUNRISE_IDE_USB_EXTEND;
            printf ("SUNRIDE: image '%s' opened (size=%lu, need %lu)\r\n",
                    SUNRISE_IMG_PATH, (unsigned long)f_size (&s_img),
                    (unsigned long)ATA_DISK_BYTES);
        }
        return 1;

    case SUNRISE_IDE_USB_EXTEND:
        /* Force the file to exactly ATA_DISK_BYTES: zero-fill
         * sequentially from the current end (positions never exceed
         * EOF, so no undefined gap-fill behaviour), or truncate an
         * oversized file. Chunked per main-loop pass so the firmware
         * stays responsive while a fresh image is created. */
        {
            const uint32_t fsize = (uint32_t)f_size (&s_img);
            if (fsize == ATA_DISK_BYTES) {
                s_ide.usb_state = SUNRISE_IDE_USB_READY;
                printf ("SUNRIDE: image ready (%lu bytes)\r\n",
                        (unsigned long)ATA_DISK_BYTES);
                return 1;
            }
            if (fsize > ATA_DISK_BYTES) {
                /* Oversized image: truncate to the hard-coded size. */
                FRESULT fr = f_lseek (&s_img, ATA_DISK_BYTES);
                if (fr == FR_OK) fr = f_truncate (&s_img);
                if (fr != FR_OK) {
                    printf ("SUNRIDE: f_truncate failed (%u)\r\n",
                            (unsigned)fr);
                    file_backend_reset ();
                    return 1;
                }
                (void)f_sync (&s_img);
                s_ide.usb_state = SUNRISE_IDE_USB_READY;
                printf ("SUNRIDE: image truncated to %lu bytes\r\n",
                        (unsigned long)ATA_DISK_BYTES);
                return 1;
            }
            /* Undersized: zero-extend with a 16 KiB chunk this pass. */
            memset (s_zeros, 0, sizeof (s_zeros));
            FRESULT fr = f_lseek (&s_img, s_ext_off);
            if (fr != FR_OK) {
                printf ("SUNRIDE: extend f_lseek failed (%u)\r\n",
                        (unsigned)fr);
                file_backend_reset ();
                return 1;
            }
            UINT bw = 0U;
            fr = f_write (&s_img, s_zeros, sizeof (s_zeros), &bw);
            if (fr != FR_OK || bw == 0U) {
                printf ("SUNRIDE: extend f_write failed (%u, bw=%u)\r\n",
                        (unsigned)fr, (unsigned)bw);
                file_backend_reset ();
                return 1;
            }
            s_ext_off += bw;
            if (s_ext_off >= ATA_DISK_BYTES) {
                (void)f_sync (&s_img);
                s_ide.usb_state = SUNRISE_IDE_USB_READY;
                printf ("SUNRIDE: image extended to %lu bytes - ready\r\n",
                        (unsigned long)ATA_DISK_BYTES);
            }
        }
        return 1;

    case SUNRISE_IDE_USB_READY:
        /* Stay here - the ATA command layer + Sunrise_IDE_Service's
         * pending-transfer logic handle individual READ/WRITE
         * commands from here on. */
        return 0;

    default:
        s_ide.usb_state = SUNRISE_IDE_USB_INIT;
        return 1;
    }
}

/* ========================================================================
 * build_identify_data - 512-byte ATA IDENTIFY DEVICE response.
 *
 * Fully static: the model / serial strings and the CHS geometry /
 * LBA capacity all come from the hard-coded constants at the top of
 * this file. The Nextor Sunrise driver's PRECHECK routine (0x4440)
 * reads 100 bytes from 0x7C00 and requires byte at offset 0x63
 * (word 49 high byte) to have bit 1 set -> word 49 = 0x0300 carries
 * both the LBA bit (bit 9) and the "DMA supported" bit the kernel
 * probes for (our READ/WRITE DMA commands are accepted and executed
 * as PIO, so the claim is honest).
 * ====================================================================== */

static void build_identify_data (void) {
    memset (s_ide.identify_buf, 0, sizeof (s_ide.identify_buf));

    /* General configuration bit (word 0):
     *   bit 15 = 0  : ATA device (not ATAPI)
     *   bit 7  = 1  : removable media (matches USB stick semantics) */
    store_le16 (&s_ide.identify_buf[0], 0x0080U);

    /* Default CHS geometry (words 1, 3, 6). */
    store_le16 (&s_ide.identify_buf[1 * 2], (uint16_t)ATA_IDENT_CYLINDERS);
    store_le16 (&s_ide.identify_buf[3 * 2], (uint16_t)ATA_IDENT_HEADS);
    store_le16 (&s_ide.identify_buf[6 * 2], (uint16_t)ATA_IDENT_SPT);

    /* Serial number (words 10..19, 20 ASCII chars, byte-swapped
     * pairs per ATA convention). */
    {
        const char *serial = "RISKYMSX2IMG00000000";
        for (int i = 0; i < 20; i++) {
            store_le16 (&s_ide.identify_buf[20 + i * 2], (uint16_t)serial[i]);
        }
    }

    /* Firmware revision (words 23..26, 8 ASCII chars). */
    {
        const char *rev = "1.0    ";
        for (int i = 0; i < 8; i++) {
            store_le16 (&s_ide.identify_buf[23 * 2 + i * 2], (uint16_t)rev[i]);
        }
    }

    /* Model number (words 27..46, 40 ASCII chars). ATA stores words
     * byte-swapped (little-endian), so we copy 2 bytes at a time with
     * each pair swapped (same layout as the serial above). */
    {
        const char *model = "RISKYMSX2 ATA IMAGEDISK            ";
        for (int i = 0; i < 40; i += 2) {
            store_le16 (&s_ide.identify_buf[27 * 2 + i],
                        (uint16_t)((uint8_t)model[i] |
                                   ((uint16_t)(uint8_t)model[i + 1] << 8)));
        }
    }

    /* Word 47: max sectors per Multiple transfer, valid flag 0x80. */
    store_le16 (&s_ide.identify_buf[47 * 2], 0x8010U);

    /* Capabilities (word 49):
     *   bit 9 = 1  : LBA addressing supported
     *   bit 8 = 1  : DMA supported (READ/WRITE DMA are executed as
     *                PIO by the emulation, so the claim is honest) */
    store_le16 (&s_ide.identify_buf[49 * 2], 0x0300U);

    /* Word 50: capabilities 2 (reserved). Word 51/52: PIO timing. */
    store_le16 (&s_ide.identify_buf[51 * 2], 0x0200U);

    /* Word 59: multiple-transfer setting valid flag + current setting. */
    store_le16 (&s_ide.identify_buf[59 * 2],
                (uint16_t)(0x0100U | (uint16_t)s_ide.multi_count));

    /* Total user sectors (LBA28) - words 60..61. */
    store_le32 (&s_ide.identify_buf[60 * 2], ATA_DISK_SECTORS);

    /* Word 64: PIO modes supported (mode 3). */
    store_le16 (&s_ide.identify_buf[64 * 2], 0x0001U);

    /* Word 75: queue depth 1. Word 80: major version (ATA-3..5). */
    store_le16 (&s_ide.identify_buf[80 * 2], 0x00FCU);

    /* Word 82/83/84: command sets supported.
     *   82: bit 3 power mgmt, bit 5 write cache, bit 6 look-ahead,
     *       bit 14 device reset cmd, bit 15 = 1 (tag)
     *   83: bit 10 LBA48, bit 13 flush cache, bit 14 = 1 (tag)
     *   84: bit 14 = 1 (tag) */
    store_le16 (&s_ide.identify_buf[82 * 2], 0xC068U);
    store_le16 (&s_ide.identify_buf[83 * 2], 0x6408U);
    store_le16 (&s_ide.identify_buf[84 * 2], 0x4000U);

    /* Word 85/86/87: enabled mirrors of 82..84 (write cache and
     * look-ahead enabled, LBA48 + flush enabled). */
    store_le16 (&s_ide.identify_buf[85 * 2], 0x0060U);
    store_le16 (&s_ide.identify_buf[86 * 2], 0x6408U);
    store_le16 (&s_ide.identify_buf[87 * 2], 0x4000U);

    /* Words 100..103: 48-bit max user LBA (= sectors - 1). */
    store_le32 (&s_ide.identify_buf[100 * 2], ATA_DISK_SECTORS - 1U);

    s_ide.identify_built = 1U;
}

/* ========================================================================
 * ATA command dispatch
 * ====================================================================== */

static void ide_execute_command (uint8_t cmd) {
    /* Features register (host writes 0x7E01 before SET FEATURES or
     * SMART; the write handler latches it into reg_error). Capture
     * BEFORE the default "error = 0" below. */
    const uint8_t features = s_ide.reg_error;

    /* Default state: command accepted, error cleared, BSY cleared,
     * DRDY + DSC set. Specific commands override (READ/WRITE arm
     * DRQ/BSY; unsupported ones set ERR|ABRT). */
    ide_set_error (0U);

    /* Recalibrate (legacy 0x10..0x1F range): no-op, succeed. */
    if (cmd >= ATA_CMD_RECALIBRATE && cmd <= ATA_CMD_RECALIBRATE_MAX) {
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        return;
    }

    switch (cmd) {
    case ATA_CMD_NOP:
        /* NOP per ATA-5: abort, set ERR|ABRT, clear BSY. */
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        break;

    case ATA_CMD_DEVICE_RESET:
    case ATA_CMD_DEVICE_DIAG:
        ide_on_device_reset ();
        /* STATUS=0x7F is the PRECHECK FAIL path in the driver.
         * Use DRDY (0x40) as the normal post-reset status.
         * CYL_LO/CYL_HI = 0x00 for the PRECHECK success path. */
        ide_set_status (ATA_STATUS_DRDY);
        s_ide.reg_cylinder_low = 0x00U;
        s_ide.reg_cylinder_high = 0x00U;
        break;

    /* --- PIO data-in commands (single + no-retry + multi + EXT +
     *     "DMA" variants, all executed as PIO 512-byte transfers) - */
    case ATA_CMD_READ_SECTORS:       /* 0x20 */
    case ATA_CMD_READ_SECTORS_NR:    /* 0x21 */
    case ATA_CMD_READ_SECTORS_EXT:   /* 0x24 (LBA48) */
    case ATA_CMD_READ_DMA_EXT:       /* 0x25 (LBA48) */
    case ATA_CMD_READ_MULTI:         /* 0xC4 */
    case ATA_CMD_READ_MULTI_EXT:     /* 0x29 (LBA48) */
    case ATA_CMD_READ_DMA:           /* 0xC8 */
    case ATA_CMD_READ_DMA_NR:        /* 0xC9 */
        ide_on_read_sectors ((cmd == ATA_CMD_READ_SECTORS_EXT ||
                              cmd == ATA_CMD_READ_DMA_EXT ||
                              cmd == ATA_CMD_READ_MULTI_EXT) ? 1U : 0U);
        break;

    /* --- PIO write commands (same family as reads) --------------- */
    case ATA_CMD_WRITE_SECTORS:      /* 0x30 */
    case ATA_CMD_WRITE_SECTORS_NR:   /* 0x31 */
    case ATA_CMD_WRITE_SECTORS_EXT:  /* 0x34 (LBA48) */
    case ATA_CMD_WRITE_DMA_EXT:      /* 0x35 (LBA48) */
    case ATA_CMD_WRITE_MULTI:        /* 0xC5 */
    case ATA_CMD_WRITE_MULTI_EXT:    /* 0x39 (LBA48) */
    case ATA_CMD_WRITE_DMA:          /* 0xCA */
    case ATA_CMD_WRITE_DMA_NR:       /* 0xCB */
    case ATA_CMD_WRITE_VERIFY:       /* 0x3C - write, then "verify" =
                                      * rewrite; we treat it as write */
        ide_on_write_sectors ((cmd == ATA_CMD_WRITE_SECTORS_EXT ||
                               cmd == ATA_CMD_WRITE_DMA_EXT ||
                               cmd == ATA_CMD_WRITE_MULTI_EXT) ? 1U : 0U);
        break;

    /* --- Verify-only commands (bounds check, no transfer) -------- */
    case ATA_CMD_READ_VERIFY:        /* 0x40 */
    case ATA_CMD_READ_VERIFY_NR:     /* 0x41 */
    case ATA_CMD_READ_VERIFY_EXT:    /* 0x42 (LBA48) */
        ide_on_read_verify ((cmd == ATA_CMD_READ_VERIFY_EXT) ? 1U : 0U);
        break;

    case ATA_CMD_IDENTIFY:
        ide_on_identify ();
        break;

    case ATA_CMD_INIT_DEV_PARAMS:
        ide_on_init_dev_params ();
        break;

    case ATA_CMD_SET_FEATURES:
        ide_on_set_features (features);
        break;

    case ATA_CMD_SET_MULTI:
        /* SET MULTIPLE MODE: block per READ/WRITE MULTIPLE. Valid
         * counts are 1..256 (0 invalid); we cap the practical count
         * to 16 (anything above just further batches the same
         * per-sector PIO loop). */
        {
            const uint8_t sc = s_ide.reg_sector_count;
            if (sc == 0U) {
                ide_set_error (ATA_ERR_ABRT);
                ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                                ATA_STATUS_ERR);
            } else {
                s_ide.multi_count = sc;
                /* Update IDENTIFY word 59 (valid flag + setting). */
                store_le16 (&s_ide.identify_buf[59 * 2],
                            (uint16_t)(0x0100U | (uint16_t)sc));
                ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
            }
        }
        break;

    case ATA_CMD_READ_BUFFER:
        /* READ BUFFER: PIO-read 512 zero bytes, sector-count ignored
         * (single-sector semantics). */
        ide_on_read_buffer ();
        break;

    case ATA_CMD_WRITE_BUFFER:
        /* WRITE BUFFER: accept one sector of data and discard (no
         * backing store concept for the buffer). */
        s_ide.state = SUNRISE_IDE_STATE_WRITE_BUSY;
        s_ide.buffer_index = 0U;
        s_ide.buffer_length = 512U;
        s_ide.sectors_remaining = 0U;   /* commit path: nothing to write */
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_DRQ);
        break;

    case ATA_CMD_SEEK:
    case ATA_CMD_RECALIBRATE:
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        break;

    case ATA_CMD_STANDBY_IMMEDIATE:
    case ATA_CMD_IDLE_IMMEDIATE:
    case ATA_CMD_STANDBY:
    case ATA_CMD_IDLE:
    case ATA_CMD_MEDIA_LOCK:
    case ATA_CMD_MEDIA_UNLOCK:
    case ATA_CMD_MEDIA_EJECT:
    case ATA_CMD_DEV_CONFIG_FREEZE:
    case ATA_CMD_SECURITY_FREEZE:
        /* No-op power / media / security posture commands: the drive
         * is always "active", never locked. Just return OK. */
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        break;

    case ATA_CMD_CHECK_POWER_MODE:
        /* Power state goes in the sector-count register. We are
         * always active/idle -> 0xFF. */
        s_ide.reg_sector_count = 0xFFU;
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        break;

    case ATA_CMD_SLEEP:
        /* SLEEP mode - nothing to actually power down. Per spec the
         * device stays in sleep until reset; we acknowledge with
         * DRDY and keep serving (a real kernel will reset us). */
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        break;

    case ATA_CMD_FLUSH_CACHE:
    case ATA_CMD_FLUSH_CACHE_EXT:
        /* Make FatFs push cached sectors to the stick. */
        if (s_img_open) {
            (void)f_sync (&s_img);
        }
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        break;

    case ATA_CMD_GET_MEDIA_STATUS:
        /* Media present, no door, no change. Clear error per spec. */
        ide_set_error (0U);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        break;

    case ATA_CMD_PACKET:
    case ATA_CMD_IDENTIFY_PACKET:
    case ATA_CMD_SMART:
    case ATA_CMD_SECURITY_SET_PW:
    case ATA_CMD_SECURITY_UNLOCK:
    case ATA_CMD_SECURITY_ERASE_PRE:
    case ATA_CMD_SECURITY_ERASE:
    case ATA_CMD_SECURITY_DISABLE:
    case ATA_CMD_SET_MAX:
        /* Not supported (ATAPI packet / SMART / security features).
         * None of these are claimed in the IDENTIFY response, and
         * ERR|ABRT is the spec-correct response for them. */
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        break;

    case ATA_CMD_READ_LONG:
    case ATA_CMD_READ_LONG_NR:
    case ATA_CMD_WRITE_LONG:
    case ATA_CMD_WRITE_LONG_NR:
        /* Obsolete ATA-1 LONG commands (512 + ECC bytes): abort. */
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        break;

    default:
        /* Unknown / reserved opcode: signal ABRT per ATA spec. */
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        break;
    }
}

/* Shared xfer-start validation: returns 0 and fills *lba / *count on
 * success; returns 1 and sets ERR|IDNF (or ERR|ABRT when the file
 * backend is not ready) otherwise. `ext` selects LBA48 semantics
 * (16-bit count from the HOB shadow + current regs; addresses beyond
 * our 28-bit capacity are rejected). */
static uint8_t ide_get_transfer (uint8_t ext, uint32_t *lba,
                                 uint32_t *count) {
    if (s_ide.usb_state != SUNRISE_IDE_USB_READY || !s_img_open) {
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        return 1U;
    }
    *lba = ide_get_lba ();
    if (ext) {
        /* LBA48: the high-order 24 bits live in the HOB shadow
         * registers. Our disk is way below 2^28 sectors, so any
         * nonzero high address byte is out of range. (A zero 16-bit
         * sector count is NOT an error - it expands to 65536 below.) */
        if (s_ide.sh_cylinder_low || s_ide.sh_cylinder_high ||
            s_ide.sh_sector_number) {
            ide_set_error (ATA_ERR_IDNF);
            ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                            ATA_STATUS_ERR);
            return 1U;
        }
        uint32_t c = ((uint32_t)s_ide.sh_sector_count << 8) |
                     s_ide.reg_sector_count;
        if (c == 0U) c = 65536U;   /* 0x0000 = 65536 per LBA48 spec */
        *count = c;
    } else {
        *count = s_ide.reg_sector_count ? (uint32_t)s_ide.reg_sector_count
                                        : 256U;
    }
    if (*lba >= ATA_DISK_SECTORS ||
        *count > (ATA_DISK_SECTORS - *lba)) {
        ide_set_error (ATA_ERR_IDNF);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        return 1U;
    }
    return 0U;
}

static void ide_on_read_sectors (uint8_t ext) {
    uint32_t lba, count;
    if (ide_get_transfer (ext, &lba, &count) != 0U) {
        return;
    }
    /* Errors above already set status; success path: */
    s_ide.sectors_remaining = count;
    s_ide.state = SUNRISE_IDE_STATE_READ_BUSY;
    s_ide.buffer_index = 0U;
    s_ide.buffer_length = 512U;
    s_ide.lba_pending = 1U;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC | ATA_STATUS_BSY);
}

static void ide_on_write_sectors (uint8_t ext) {
    uint32_t lba, count;
    if (ide_get_transfer (ext, &lba, &count) != 0U) {
        return;
    }
    s_ide.sectors_remaining = count;
    s_ide.state = SUNRISE_IDE_STATE_WRITE_BUSY;
    s_ide.buffer_index = 0U;
    s_ide.buffer_length = 512U;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                    ATA_STATUS_DRQ);
}

static void ide_on_read_verify (uint8_t ext) {
    uint32_t lba, count;
    if (ide_get_transfer (ext, &lba, &count) != 0U) {
        return;
    }
    if (lba + count > ATA_DISK_SECTORS) {
        ide_set_error (ATA_ERR_IDNF);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        return;
    }
    /* Verify = the data is there (the image read path body proves
     * range validity); nothing to transfer. */
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
}

static void ide_on_read_buffer (void) {
    if (s_ide.usb_state != SUNRISE_IDE_USB_READY || !s_img_open) {
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        return;
    }
    memset (s_ide.sector_buffer, 0, 512U);
    s_ide.state = SUNRISE_IDE_STATE_READY;
    s_ide.buffer_index = 0U;
    s_ide.buffer_length = 512U;
    s_ide.sectors_remaining = 0U;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DRQ);
}

static void ide_on_identify (void) {
    if (s_ide.identify_built == 0U) {
        build_identify_data ();
    }
    /* Copy the IDENTIFY response into the sector buffer and arm the
     * data register for draining. The Nextor Sunrise driver's
     * PRECHECK routine (0x4440) reads 100 bytes from 0x7C00, then
     * checks STATUS: if STATUS==0x7F that's the FAIL path (SCF+RET).
     * The SUCCESS path requires STATUS != 0x7F, CYL_LO=0x00,
     * CYL_HI=0x00, and byte at buffer offset 0x63 bit 1 set.
     * Use DRDY (0x40) as the normal post-command status. */
    memcpy (s_ide.sector_buffer, s_ide.identify_buf, 512U);
    s_ide.state = SUNRISE_IDE_STATE_READY;
    s_ide.buffer_index = 0U;
    s_ide.buffer_length = 512U;
    s_ide.sectors_remaining = 0U;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DRQ);
    /* CYL_LO and CYL_HI must be 0x00 for the PRECHECK success path. */
    s_ide.reg_cylinder_low = 0x00U;
    s_ide.reg_cylinder_high = 0x00U;
}

static void ide_on_device_reset (void) {
    ide_set_signature ();
    s_ide.state = SUNRISE_IDE_STATE_IDLE;
    s_ide.reg_device_ctrl = 0U;
    /* reg_status was set by ide_set_signature() - DRDY. */
}

static void ide_on_init_dev_params (void) {
    /* The kernel uses this to negotiate CHS geometry. We don't
     * actually need CHS - LBA is supported - but acknowledge the
     * command and update sector_count. */
    s_ide.reg_sector_count = (s_ide.reg_sector_count == 0U)
                             ? 0xFFU : s_ide.reg_sector_count;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
}

static void ide_on_set_features (uint8_t sub) {
    /* Subcommand in the features register (latched in reg_error on
     * write). Everything we advertise in IDENTIFY (transfer modes,
     * write cache, look-ahead...) is accepted as a no-op; the IDE
     * enable is virtual, so there is nothing to actually configure. */
    (void)sub;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
}

/* ========================================================================
 * Cart-side handlers - called from Cart_EXTI0_Sunride_Handler.
 *
 * IRQ context. No blocking calls, no printf, no FatFs calls.
 * ====================================================================== */

uint8_t Sunrise_IDE_ReadByte (uint16_t address) {
    /* --- ATA register window ------------------------------------------- */
    if (s_ide.ide_enabled &&
        address >= SUNRIDE_IDE_REG_BASE &&
        address <= SUNRIDE_IDE_REG_END) {
        const uint8_t off = (uint8_t)(address & 0x0FU);
        switch (off) {
        case ATA_REG_ERROR:          return s_ide.reg_error;
        case ATA_REG_SECTOR_COUNT:   return s_ide.reg_sector_count;
        case ATA_REG_SECTOR_NUMBER:  return s_ide.reg_sector_number;
        case ATA_REG_CYLINDER_LOW:   return s_ide.reg_cylinder_low;
        case ATA_REG_CYLINDER_HIGH:  return s_ide.reg_cylinder_high;
        case ATA_REG_DEVICE_HEAD:    return s_ide.reg_device_head;
        case ATA_REG_STATUS:
            s_ide.stat_status_reads++;
            return s_ide.reg_status;
        default:                     return 0xFFU;  /* padding reads 0xFF */
        }
    }

    /* --- ATA data register window -------------------------------------- */
    if (s_ide.ide_enabled &&
        address >= SUNRIDE_IDE_DATA_BASE &&
        address <= SUNRIDE_IDE_DATA_END) {
        /* PIO byte stream. The Nextor Sunrise IDE driver drains the
         * 512-byte sector with LDIR from 0x7C00..0x7DFF (a byte per
         * address, NOT paired even/odd word reads - verified by
         * disassembling the embedded driver bank; the kernel uses
         * `ld hl,7C00h / ld bc,0200h / ldir`, NOT INIR). Each read
         * returns one byte and advances buffer_index by 1. */
        uint8_t v = 0xFFU;
        if (s_ide.state == SUNRISE_IDE_STATE_READY &&
            s_ide.buffer_index < s_ide.buffer_length) {
            v = s_ide.sector_buffer[s_ide.buffer_index];
        } else if (s_ide.state == SUNRISE_IDE_STATE_IDLE) {
            /* Sunrise IDE PRECHECK mode: when the device is idle and
             * no PIO transfer is in progress, the data register
             * (0x7C00..) returns the IDENTIFY-shaped device
             * signature. The Nextor Sunrise driver reads ~100 bytes
             * from 0x7C00 during PRECHECK to detect this signature.
             * Return identify_buf directly; buffer_index is
             * irrelevant here since this read is not part of a PIO
             * drain. */
            if (s_ide.identify_built == 0U) {
                build_identify_data ();
            }
            const uint16_t idx =
                (uint16_t)(address - SUNRIDE_IDE_DATA_BASE);
            if (idx < 512U) {
                v = s_ide.identify_buf[idx];
            }
        }
        /* Advance on every read regardless of address parity. */
        s_ide.buffer_index = (uint16_t)(s_ide.buffer_index + 1U);
        if (s_ide.buffer_index >= s_ide.buffer_length) {
            /* Sector fully drained. If multi-sector and more sectors
             * remaining, kick off the next file read (BSY=1, DRQ=0);
             * otherwise transition to IDLE (BSY=0, DRQ=0, DRDY=1). */
            s_ide.stat_drains++;
            if (s_ide.state == SUNRISE_IDE_STATE_READY &&
                s_ide.sectors_remaining > 1U) {
                s_ide.sectors_remaining--;
                ide_advance_lba ();
                s_ide.state = SUNRISE_IDE_STATE_READ_BUSY;
                s_ide.buffer_index = 0U;
                s_ide.lba_pending = 1U;
                ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                                ATA_STATUS_BSY);
            } else {
                s_ide.state = SUNRISE_IDE_STATE_IDLE;
                s_ide.sectors_remaining = 0U;
                ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
            }
        }
        return v;
    }

    /* --- ROM window ---------------------------------------------------- */
    /* Address is 0x4000..0x7FFF with the IDE/data registers punched
     * out. We masked those already - all remaining addresses are ROM. */
    if (address >= 0x4000U && address < 0x8000U) {
        const uint32_t off = ((uint32_t)s_ide.segment << 14) |
                             (uint32_t)(address & 0x3FFFU);
        if (off < nextor_rom_len) {
            return nextor_rom[off];
        }
        return 0xFFU;
    }

    return 0xFFU;
}

void Sunrise_IDE_WriteByte (uint16_t address, uint8_t value) {
    /* --- Sunrise IDE control register ---------------------------------- */
    if (address == SUNRIDE_CTRL_REG_ADDR) {
        /* Carnivore2 VHDL:
         *   IDEROMADDR <= cReg(5) & cReg(6) & cReg(7) & Addr(13..0)
         * i.e. the hardware maps bit5->bank_bit2, bit6->bank_bit1,
         * bit7->bank_bit0 (reverses bits 7..5 back into bits 0..2 of
         * the bank index). The Nextor chgbnk routine writes the bank
         * number in this bit-reversed form (bank_bit0 goes to bit 7,
         * bank_bit1 to bit 6, bank_bit2 to bit 5), so we must reverse
         * bits 7..5 of the written value to recover the actual page.
         * The IDE enable is in bit 0. Reference implementation:
         * Cristiano's PicoVerse sunrise_ide.c (same Sunrise hardware). */
        const uint8_t raw = (uint8_t)((value >> 5) & 0x07U);
        /* Reverse 3 bits: {bit2,bit1,bit0} -> {bit0,bit1,bit2} */
        s_ide.segment = (uint8_t)(((raw & 0x04U) >> 2) |
                                  (raw & 0x02U) |
                                  ((raw & 0x01U) << 2));
        s_ide.ide_enabled =
            (uint8_t)((value & SUNRIDE_CTRL_IDE_EN) ? 1U : 0U);
        return;
    }

    /* --- ATA register window ------------------------------------------- */
    if (s_ide.ide_enabled &&
        address >= SUNRIDE_IDE_REG_BASE &&
        address <= SUNRIDE_IDE_REG_END) {
        const uint8_t off = (uint8_t)(address & 0x0FU);
        /* HOB selects the "previous content" (shadow) register on
         * task-file writes when LBA48 software is driving us. */
        const uint8_t hob = s_ide.hob;
        switch (off) {
        case ATA_REG_ERROR:           /* write = features */
            /* Latch features for SET FEATURES / SMART dispatch. */
            s_ide.reg_error = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_SECTOR_COUNT:
            if (hob) s_ide.sh_sector_count = value;
            else     s_ide.reg_sector_count = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_SECTOR_NUMBER:
            if (hob) s_ide.sh_sector_number = value;
            else     s_ide.reg_sector_number = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_CYLINDER_LOW:
            if (hob) s_ide.sh_cylinder_low = value;
            else     s_ide.reg_cylinder_low = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_CYLINDER_HIGH:
            if (hob) s_ide.sh_cylinder_high = value;
            else     s_ide.reg_cylinder_high = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_DEVICE_HEAD:
            s_ide.reg_device_head = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_STATUS:          /* write = command */
            s_ide.stat_atacmds++;
            s_ide.stat_last_cmd = value;
            ide_execute_command (value);
            return;
        case ATA_REG_DEVICE_CTRL:
            s_ide.reg_device_ctrl = value;
            s_ide.hob = (uint8_t)(value & ATA_DEVCTRL_HOB);
            if (value & ATA_DEVCTRL_SRST) {
                /* Software reset - mirror the device behaviour. */
                ide_set_signature ();
                s_ide.state = SUNRISE_IDE_STATE_IDLE;
                s_ide.sectors_remaining = 0U;
                /* STATUS=0x7F is the PRECHECK FAIL path in the driver.
                 * Use DRDY (0x40) as normal. The driver's wait loop
                 * checks AND 0xC0 == 0x40 (DRDY set, BSY clear). */
                ide_set_status (ATA_STATUS_DRDY);
                /* CYL_LO/CYL_HI = 0x00 for PRECHECK success path. */
                s_ide.reg_cylinder_low = 0x00U;
                s_ide.reg_cylinder_high = 0x00U;
                /* Pre-arm data register with IDENTIFY data for the
                 * PRECHECK LDIR (100 bytes from 0x7C00). */
                if (s_ide.identify_built == 0U) {
                    build_identify_data ();
                }
                memcpy (s_ide.sector_buffer, s_ide.identify_buf, 512U);
                s_ide.buffer_index = 0U;
                s_ide.buffer_length = 512U;
                s_ide.state = SUNRISE_IDE_STATE_READY;
            }
            return;
        default:
            return;
        }
    }

    /* --- ATA data register window (PIO write) -------------------------- */
    if (s_ide.ide_enabled &&
        address >= SUNRIDE_IDE_DATA_BASE &&
        address <= SUNRIDE_IDE_DATA_END) {
        if (s_ide.state != SUNRISE_IDE_STATE_WRITE_BUSY) return;
        /* PIO byte stream (same LDIR-from-0x7C00 pattern as read). */
        if (s_ide.buffer_index < s_ide.buffer_length) {
            s_ide.sector_buffer[s_ide.buffer_index] = value;
        }
        s_ide.buffer_index = (uint16_t)(s_ide.buffer_index + 1U);
        /* buffer_index / buffer_length check happens in
         * Sunrise_IDE_Service (main-loop context) - that fires
         * the image-file write once all bytes are in. */
        return;
    }

    /* All other writes are ignored. */
}

/* ========================================================================
 * Helpers - little-endian word / dword store into the IDENTIFY buffer.
 * ====================================================================== */

static void store_le16 (uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static void store_le32 (uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

/* Read-only accessor for the live state struct. Used by the main
 * loop's status printer - IRQ-side code must NOT call this (the
 * struct can be modified underfoot). */
const Sunrise_IDE *Sunrise_IDE_GetState (void) {
    return &s_ide;
}
