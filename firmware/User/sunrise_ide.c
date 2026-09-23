/*
 * sunrise_ide.c - Sunrise IDE emulation for the RISKYMSX2 cartridge.
 *
 * Ported from PicoVerse 2040's sunrise_ide.c (cristianoag/msx-picoverse-
 * public). The cart-window decoder (control register, bit-reverse, IDE
 * gating) lives in cart.c::Cart_EXTI0_Sunride_Handler; this file owns
 * the ATA state machine and the USB backing-store bridge.
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
 * Module state
 * ====================================================================== */

/* Single global state struct. Lives in regular SRAM (no .ramfunc - the
 * cart IRQ handler is the only consumer and it just reads / writes a
 * few fields per cycle, no DMA, no large tables). */
static Sunrise_IDE s_ide;

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
static void ide_on_read_sectors (void);
static void ide_on_write_sectors (void);
static void ide_on_identify (void);
static void ide_on_device_reset (void);
static void ide_on_init_dev_params (void);
static void ide_on_set_features (void);
static void build_identify_data (void);
static void store_le16 (uint8_t *p, uint16_t v);
static void store_le32 (uint8_t *p, uint32_t v);
static int usb_state_step (void);
static int usb_do_inquiry (void);
static int usb_do_capacity (void);

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
    ide_set_signature ();
}

static void ide_set_signature (void) {
    s_ide.reg_error          = ATA_SIG_ERROR;
    s_ide.reg_sector_count   = ATA_SIG_SECTOR_COUNT;
    s_ide.reg_cylinder_low   = ATA_SIG_CYLINDER_LOW;
    s_ide.reg_cylinder_high  = ATA_SIG_CYLINDER_HIGH;
    s_ide.reg_device_head    = ATA_SIG_DEVICE_HEAD;
    s_ide.reg_status         = ATA_SIG_STATUS;
}

static void ide_set_status (uint8_t status) {
    /* Bits 0..6 are R/W from the device side; bit 7 (BSY) is also
     * settable. Never clear bits the device is asserting - this is a
     * latch, not an OR-with-zero. */
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
    if (lba + 1U < s_ide.block_count) {
        ide_set_lba (lba + 1U);
    } else {
        /* Past the end of disk: leave LBA unchanged, the next read
         * will return 0xFF and the kernel will see IDNF. The kernel
         * counts sectors via reg_sector_count and stops before the
         * LBA rolls over the edge. */
    }
}

/* ========================================================================
 * Main-loop service: drive the USB lifecycle + IDE state machine.
 *
 * Called from main()'s main loop. Cannot block on EXTI0 (no spin
 * loops). USBH_PreDeal + usb_scsi_* are blocking per the USBHS host
 * API, so this call may take tens of milliseconds during enumeration
 * but the MSX is reset at that point - we're booting the kernel.
 * ====================================================================== */

void Sunrise_IDE_Service (void) {
    /* USB lifecycle: each pass either advances to the next state or
     * stays put. usb_state_step returns 1 if state advanced. */
    (void)usb_state_step ();

    /* In-flight READ SECTORS: if lba_pending is set, the IRQ handler
     * just queued a USB read (ide_on_read_sectors or the multi-sector
     * advance in Sunrise_IDE_ReadByte). Issue the SCSI read here -
     * blocking, but the MSX's PIO loop is polling BSY/DRQ so the
     * kernel sees BSY drop only after this returns. usb_scsi_read_sector
     * typically completes in 1-2 ms for a high-speed stick; the kernel
     * will poll status and see DRQ on the next read of 0x7E07. */
    if (s_ide.state == SUNRISE_IDE_STATE_READ_BUSY &&
        s_ide.lba_pending != 0U) {
        const uint32_t lba = ide_get_lba ();
        uint8_t rc = 0U;
        if (lba < s_ide.block_count) {
            rc = usb_scsi_read_sector (lba, s_ide.sector_buffer,
                                       s_ide.block_size);
        } else {
            rc = 1U;   /* out-of-range - treat as SCSI error */
        }
        s_ide.lba_pending = 0U;
        s_ide.buffer_index = 0U;
        if (rc != 0U) {
            /* SCSI failed: assert ERR + ABRT, leave buffer as-is.
             * The kernel will see ERR on its next status poll and
             * abort the command rather than looping on stale data. */
            ide_set_error (ATA_ERR_ABRT);
            s_ide.state = SUNRISE_IDE_STATE_IDLE;
            s_ide.sectors_remaining = 0U;
            ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                            ATA_STATUS_ERR);
        } else {
            s_ide.state = SUNRISE_IDE_STATE_READY;
            ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                            ATA_STATUS_DRQ);
        }
    }

    /* In-flight WRITE SECTORS: drain remaining data into the buffer
     * until the MSX has finished the 512-byte PIO write, then commit
     * to USB. */
    if (s_ide.state == SUNRISE_IDE_STATE_WRITE_BUSY) {
        if (s_ide.buffer_index >= s_ide.buffer_length) {
            /* All bytes received from the MSX. Issue the USB write. */
            if (s_ide.block_count > 0U) {
                const uint32_t lba = ide_get_lba ();
                if (lba < s_ide.block_count) {
                    (void)usb_scsi_write_sector (lba,
                                                 s_ide.sector_buffer,
                                                 s_ide.block_size);
                }
            }
            ide_advance_lba ();
            s_ide.state = SUNRISE_IDE_STATE_IDLE;
            ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        }
    }
}

static int usb_state_step (void) {
    switch (s_ide.usb_state) {
    case SUNRISE_IDE_USB_INIT:
        /* Nothing to do - the host stack was already initialised by
         * USB_Initialization() from main(). Move to enum and let the
         * host stack detect a device if one is attached. */
        s_ide.usb_state = SUNRISE_IDE_USB_ENUM;
        return 1;

    case SUNRISE_IDE_USB_ENUM:
        /* Skip USBH_PreDeal entirely. Its change-bit gate
         * (PORT_STATUS_CHG & PORT_CONNECT) only fires on a fresh
         * attach-edge. The terminal already enumerated the device
         * via its own disk_initialize path; USBH_PreDeal from this
         * code path returns ROOT_DEV_FAILED=DEF_DEFAULT (0xFF)
         * forever because the change bit was ACK'd long ago. Going
         * straight to INQUIRY (which talks directly to the bulk
         * endpoints) lets SCSI succeed on a device that's already
         * up. If the device isn't enumerated, the SCSI command will
         * return non-zero and we retry. */
        s_ide.usb_state = SUNRISE_IDE_USB_INQUIRY;
        return 1;

    case SUNRISE_IDE_USB_INQUIRY:
        if (usb_do_inquiry () == 0) {
            s_ide.usb_state = SUNRISE_IDE_USB_CAPACITY;
            return 1;
        }
        return 0;

    case SUNRISE_IDE_USB_CAPACITY:
        if (usb_do_capacity () == 0) {
            build_identify_data ();
            s_ide.usb_state = SUNRISE_IDE_USB_READY;
            return 1;
        }
        return 0;

    case SUNRISE_IDE_USB_READY:
        /* Stay here - the IDE state machine (in ide_execute_command +
         * Sunrise_IDE_Service) handles individual READ/WRITE
         * commands from here on. */
        return 0;

    default:
        s_ide.usb_state = SUNRISE_IDE_USB_INIT;
        return 1;
    }
}

static int usb_do_inquiry (void) {
    uint8_t res = usb_scsi_inquiry (s_ide.inquiry_buf,
                                    sizeof (s_ide.inquiry_buf));
    if (res != 0) {
        printf ("SUNRIDE: INQUIRY failed (res=%u) - retrying\r\n", res);
        return -1;
    }
    printf ("SUNRIDE: INQUIRY ok - vendor='%.8s' product='%.16s' "
            "rev='%.4s'\r\n",
            (char *)&s_ide.inquiry_buf[8],
            (char *)&s_ide.inquiry_buf[16],
            (char *)&s_ide.inquiry_buf[32]);
    return 0;
}

static int usb_do_capacity (void) {
    if (usb_scsi_read_capacity (&s_ide.block_count,
                                &s_ide.block_size) != 0) {
        printf ("SUNRIDE: READ CAPACITY failed - retrying\r\n");
        return -1;
    }
    /* Some sticks return block_size != 512. Normalise to 512 - the
     * kernel only speaks 512-byte sectors and any other size would
     * corrupt the LBA math. */
    if (s_ide.block_size != 512U) {
        if (s_ide.block_size == 0U) s_ide.block_size = 512U;
        const uint32_t bsz = s_ide.block_size;
        s_ide.block_count =
            (uint32_t)(((uint64_t)s_ide.block_count * bsz) / 512U);
        s_ide.block_size = 512U;
    }
    /* LBA28 caps at (1<<28)-2 on some kernels - cap at block_count. */
    if (s_ide.block_count > 0x0FFFFFFFU) {
        s_ide.block_count = 0x0FFFFFFFU;
    }
    printf ("SUNRIDE: READ CAPACITY ok - block_count=%lu block_size=%lu\r\n",
            (unsigned long)s_ide.block_count,
            (unsigned long)s_ide.block_size);
    return 0;
}

/* ========================================================================
 * build_identify_data - 512-byte ATA IDENTIFY DEVICE response.
 *
 * Generate enough of the response for the Nextor Sunrise IDE driver to
 * recognise the device and start LBA I/O. Model / serial come from the
 * USB INQUIRY strings (or a default). CHS geometry + LBA capacity
 * come from s_ide.block_count (sourced from READ CAPACITY).
 * ====================================================================== */

static void build_identify_data (void) {
    memset (s_ide.identify_buf, 0, sizeof (s_ide.identify_buf));

    /* General configuration bit (word 0):
     *   bit 15 = 0  : ATA device (not ATAPI)
     *   bit 7  = 1  : removable media (matches USB stick semantics) */
    store_le16 (&s_ide.identify_buf[0], 0x0080U);

    /* Serial number (words 10..19, 20 ASCII chars). "RISKYMSX2        "
     * is the default; the INQUIRY response doesn't carry a serial so
     * we just identify as the firmware. */
    {
        const char *serial = "RISKYMSX2        ";
        for (int i = 0; i < 20; i++) {
            store_le16 (&s_ide.identify_buf[20 + i * 2],
                        (uint16_t)serial[i]);
        }
    }

    /* Model / firmware revision (words 27..46). Use INQUIRY vendor
     * (8 bytes) + product (16 bytes) + "    " (4 bytes) reversed -
     * ATA stores words byte-swapped (little-endian), so we copy
     * 2-byte-at-a-time with each pair swapped. */
    {
        uint8_t model[40];
        memset (model, ' ', sizeof (model));
        memcpy (&model[0],  &s_ide.inquiry_buf[8],  8);
        memcpy (&model[8],  &s_ide.inquiry_buf[16], 16);
        memcpy (&model[24], &s_ide.inquiry_buf[32], 4);
        /* Pad to 40 bytes. */
        for (int i = 0; i < 40; i += 2) {
            const uint8_t a = model[i];
            const uint8_t b = model[i + 1];
            store_le16 (&s_ide.identify_buf[54 + i],
                        (uint16_t)((b << 8) | a));
        }
    }

    /* Capabilities (word 49):
     *   bit 9 = 1  : LBA addressing supported
     *   bit 8 = 1  : DMA supported (we don't actually do DMA but the
     *                kernel reads this for capability detection) */
    store_le16 (&s_ide.identify_buf[49 * 2], 0x0300U);

    /* Total sectors (LBA28) - word 60..61. Cap to 512MB (1M sectors)
     * in the IDENTIFY response to prevent the Nextor kernel from
     * over-allocating RAM on large USB sticks. The actual READ path
     * uses the real block_count for range checking, so the full
     * capacity is still accessible — the kernel just sees a smaller
     * geometry for buffer allocation purposes. */
    uint32_t reported_sectors = s_ide.block_count;
    if (reported_sectors > 0x000FFFFFU) {
        reported_sectors = 0x000FFFFFU;  /* 512MB cap for IDENTIFY */
    }
    store_le32 (&s_ide.identify_buf[60 * 2], reported_sectors);

    /* Multiword DMA / command sets - leave 0 (PIO mode only). The
     * kernel handles this fine. */

    s_ide.identify_built = 1U;
}

/* ========================================================================
 * ATA command dispatch
 * ====================================================================== */

static void ide_execute_command (uint8_t cmd) {
    /* Default state: command accepted, BSY cleared, DRDY + DSC set.
     * Specific commands override the status (e.g. EXECUTE DEVICE
     * DIAGNOSTIC puts the signature back). */
    ide_set_error (0U);

    switch (cmd) {
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

    case ATA_CMD_READ_SECTORS:
    case ATA_CMD_READ_MULTI:
        ide_on_read_sectors ();
        break;

    case ATA_CMD_WRITE_SECTORS:
    case ATA_CMD_WRITE_MULTI:
        ide_on_write_sectors ();
        break;

    case ATA_CMD_IDENTIFY:
        ide_on_identify ();
        break;

    case ATA_CMD_INIT_DEV_PARAMS:
        ide_on_init_dev_params ();
        break;

    case ATA_CMD_SET_FEATURES:
        ide_on_set_features ();
        break;

    case ATA_CMD_STANDBY_IMMEDIATE:
    case ATA_CMD_IDLE_IMMEDIATE:
        /* No-op: drive is always "active" - just return OK. */
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
        break;

    case ATA_CMD_PACKET:
        /* ATAPI not supported - signal ABRT. */
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        break;

    default:
        /* Unknown command: signal ABRT per ATA spec. */
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        break;
    }
}

static void ide_on_read_sectors (void) {
    /* The kernel sets sector_count to the number of sectors it wants
     * (typically 1, but can be up to 256 with the 0x00 = 256 rule).
     * Start the first USB read; subsequent sectors are triggered
     * implicitly as the data register is drained. */
    if (s_ide.block_count == 0U) {
        /* USB stack not ready yet - signal error. The kernel will
         * retry on its own timer. */
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        return;
    }
    const uint32_t lba = ide_get_lba ();
    if (lba >= s_ide.block_count) {
        ide_set_error (ATA_ERR_IDNF);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        return;
    }
    /* Mirror sector_count into our private counter. 0x00 == 256
     * sectors per ATA spec - encode that as 256 in sectors_remaining. */
    s_ide.sectors_remaining =
        (uint16_t)((s_ide.reg_sector_count == 0U)
                   ? 256U : (uint16_t)s_ide.reg_sector_count);
    s_ide.state = SUNRISE_IDE_STATE_READ_BUSY;
    s_ide.buffer_index = 0U;
    s_ide.buffer_length = 512U;
    s_ide.lba_pending = 1U;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC | ATA_STATUS_BSY);
}

static void ide_on_write_sectors (void) {
    if (s_ide.block_count == 0U) {
        ide_set_error (ATA_ERR_ABRT);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        return;
    }
    const uint32_t lba = ide_get_lba ();
    if (lba >= s_ide.block_count) {
        ide_set_error (ATA_ERR_IDNF);
        ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                        ATA_STATUS_ERR);
        return;
    }
    s_ide.sectors_remaining =
        (uint16_t)((s_ide.reg_sector_count == 0U)
                   ? 256U : (uint16_t)s_ide.reg_sector_count);
    s_ide.state = SUNRISE_IDE_STATE_WRITE_BUSY;
    s_ide.buffer_index = 0U;
    s_ide.buffer_length = 512U;
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC |
                    ATA_STATUS_DRQ);
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

static void ide_on_set_features (void) {
    /* Subcommand in reg_error. Most are no-ops for us (write cache,
     * read look-ahead, etc.). Always succeed. */
    ide_set_status (ATA_STATUS_DRDY | ATA_STATUS_DSC);
}

/* ========================================================================
 * Cart-side handlers - called from Cart_EXTI0_Sunride_Handler.
 *
 * IRQ context. No blocking calls, no printf, no long loops.
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
             * remaining, kick off the next USB read (BSY=1, DRQ=0);
             * otherwise transition to IDLE (BSY=0, DRQ=0, DRDY=1). */
            s_ide.stat_drains++;
            if (s_ide.sectors_remaining > 1U) {
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
        switch (off) {
        case ATA_REG_ERROR:           /* write = features */
            /* Features is rarely checked by the kernel; update silently. */
            return;
        case ATA_REG_SECTOR_COUNT:
            /* 0x00 means 256 sectors per ATA spec. */
            s_ide.reg_sector_count = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_SECTOR_NUMBER:
            s_ide.reg_sector_number = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_CYLINDER_LOW:
            s_ide.reg_cylinder_low = value;
            s_ide.stat_tf_writes++;
            return;
        case ATA_REG_CYLINDER_HIGH:
            s_ide.reg_cylinder_high = value;
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
            if (value & ATA_DEVCTRL_SRST) {
                /* Software reset - mirror the device behaviour. */
                ide_set_signature ();
                s_ide.state = SUNRISE_IDE_STATE_IDLE;
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
                s_ide.sectors_remaining = 0U;
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
         * the USB write once all bytes are in. */
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
 * struct is volatile and can be modified underfoot). */
const Sunrise_IDE *Sunrise_IDE_GetState (void) {
    return &s_ide;
}
