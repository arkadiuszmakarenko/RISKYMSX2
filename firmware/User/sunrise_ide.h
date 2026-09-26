/*
 * sunrise_ide.h - Sunrise IDE emulation for the RISKYMSX2 cartridge.
 *
 * This implements the cartridge-side half of the "Sunrise IDE" mapper
 * (the same one Carnivore2 / GR8NET / many other modern MSX IDE
 * cartridges use).  The IDE bus is backed by a FAT file on the USB
 * stick (0:/nextor.img) through FatFs - NOT by raw SCSI sectors.
 *
 * Why a file-backed disk:
 *
 *   Direct SCSI access to the USB stick proved unreliable on real
 *   hardware.  The FAT stack (usb_disk.c + ff.c + diskio.c) is the
 *   well-tested path in this firmware (the ROM loader and terminal
 *   already use it), so the ATA layer is served from a fixed-size
 *   image file named nextor.img in the root of the stick.  The image
 *   geometry is HARD-CODED (see ATA_DISK_SECTORS in sunrise_ide.c) -
 *   the size of the actual file is irrelevant; the firmware extends
 *   the file with zeros on first mount and never reports any other
 *   capacity.
 *
 * Memory map of the Sunrise IDE mapper (Z80 side, 0x4000..0x7FFF):
 *
 *   0x4000..0x40FF  current 16 KiB bank (lower half served from ROM,
 *                   upper half mirrored)
 *   0x4100..0x4103  current 16 KiB bank (continued - mirror)
 *   0x4104          control register (bank bit-reversed + IDE enable)
 *   0x4105..0x7BFF  bank continuation (mirrored)
 *   0x7C00..0x7DFF  IDE data register (PIO mode, byte stream)
 *   0x7E00..0x7EFF  ATA task-file registers (16-byte window)
 *   0x7F00..0x7FFF  bank continuation (mirrored)
 *
 * The 16 KiB bank is chosen by writing to 0x4104. The low three bits
 * of the written byte select one of 8 banks; the high five bits are
 * ignored except bit 7 which acts as the IDE enable flag. The bank
 * index is BIT-REVERSED before being used (Carnivore2 VHDL:
 *   IDEROMADDR <= cReg(5) & cReg(6) & cReg(7) & Addr(13..0);
 * ) so bank 0 lives at 0x4000-0x7FFF when the MSX writes 0x80
 * (binary 10000000 -> reversed = 00000001 = bank 1 in linear order).
 * Bank 0 (kernel's "first bank") is therefore at the highest index
 * the kernel writes (0x81) - see sunrise_ide.c::write_control.
 *
 * IDE register / data window decoding is gated by bit 0 of the last
 * 0x4104 write: when bit 0 = 0 the cart just serves the ROM; when
 * bit 0 = 1 the 0x7C00-0x7EFF window decodes as ATA registers + data
 * register (the rest of the bank is still served from ROM).
 *
 * ATA-side details:
 *
 *   - Data register is drained/filled ONE BYTE per access (the Nextor
 *     Sunrise driver LDIRs from 0x7C00 byte-by-byte, not INIR).
 *   - Task-file registers are mirrored every 16 bytes
 *     (0x7E00..0x7E0F = primary, 0x7E10..0x7E1F = same primary, ...).
 *   - LBA28 addressing: high 4 bits in device/head, mid 8 in cylinder
 *     high, low 8 in cylinder low, low 8 in sector number. LBA48
 *     ("EXT") commands are also accepted using the HOB shadow
 *     registers, but any address above the hard-coded capacity is
 *     rejected with IDNF (our disk is far below 2^28 sectors anyway).
 *   - LBA advances automatically as the MSX drains/fills the 512-byte
 *     sector buffer through the data register.
 *   - IDENTIFY DEVICE (0xEC) returns a fixed 512-byte response with
 *     the hard-coded geometry; byte 0x63 has bit 1 set (LBA) which
 *     the Nextor Sunrise driver's PRECHECK routine requires.
 *
 * Lifecycle:
 *
 *   - Sunrise_IDE_Init(): zero the state struct. Called from main().
 *   - Sunrise_IDE_Service(): main-loop only. Drives the file backend
 *     lifecycle (mount USB -> open/extend nextor.img) and performs
 *     the blocking FatFs reads/writes for in-flight ATA transfers.
 *     The FatFs API may NOT be called from IRQ context - that is why
 *     all file I/O lives here.
 *   - Sunrise_IDE_ReadByte() / Sunrise_IDE_WriteByte(): called by
 *     Cart_EXTI0_Sunride_Handler on each EXTI0 cart cycle. IRQ-safe:
 *     no blocking, no printf, no FatFs calls.
 *
 * The runtime state struct lives in zero-wait-state SRAM (the cart
 * IRQ handler and main loop both touch it).
 */

#ifndef __SUNRISE_IDE_H
#define __SUNRISE_IDE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Sunrise IDE memory map (Z80 side)
 * ====================================================================== */

/* Control register address - writes here select the bank + IDE enable. */
#define SUNRIDE_CTRL_REG_ADDR      0x4104U
/* IDE data register (PIO mode). One byte per access. */
#define SUNRIDE_IDE_DATA_BASE      0x7C00U
#define SUNRIDE_IDE_DATA_END       0x7DFFU
/* ATA task-file register window (mirrored every 16 bytes). */
#define SUNRIDE_IDE_REG_BASE       0x7E00U
#define SUNRIDE_IDE_REG_END        0x7EFFU

/* Sunrise IDE control register bit layout (Carnivore2 VHDL: the
 * Nextor chgbnk routine reverses bits 0..2 of the bank number onto
 * bits 7..5 of the register before writing; the hardware then
 * reverses bits 7..5 back into bits 0..2 of the page index).
 *
 *   bit 0    = IDE registers enable (0 = disabled, 1 = enabled)
 *   bits 7..5 = FlashROM segment number, bit-reversed:
 *               bank_bit0 <- reg_bit7, bank_bit1 <- reg_bit6,
 *               bank_bit2 <- reg_bit5
 *   bits 4..1 = unused / reserved (mask off when reading bits 7..5)
 */
#define SUNRIDE_CTRL_IDE_EN        0x01U
#define SUNRIDE_CTRL_BANK_FIELD    0xE0U  /* bits 7..5 */
#define SUNRIDE_CTRL_BANK_MASK     0x07U  /* bank index after reverse */

/* ========================================================================
 * ATA task-file register offsets (within the 0x7E00..0x7EFF window).
 * ====================================================================== */

#define ATA_REG_DATA               0x00U  /* 16-bit data register */
#define ATA_REG_ERROR              0x01U  /* read: error, write: features */
#define ATA_REG_SECTOR_COUNT       0x02U
#define ATA_REG_SECTOR_NUMBER      0x03U
#define ATA_REG_CYLINDER_LOW       0x04U
#define ATA_REG_CYLINDER_HIGH      0x05U
#define ATA_REG_DEVICE_HEAD        0x06U
#define ATA_REG_STATUS             0x07U  /* read: status, write: command */
#define ATA_REG_DEVICE_CTRL        0x0EU  /* write-only: device control */

/* ATA status register bits. */
#define ATA_STATUS_ERR             0x01U  /* error */
#define ATA_STATUS_IDX             0x02U  /* index (obsolete) */
#define ATA_STATUS_CORR            0x04U  /* corrected (obsolete) */
#define ATA_STATUS_DRQ             0x08U  /* data request - sector ready */
#define ATA_STATUS_DSC             0x10U  /* drive seek complete */
#define ATA_STATUS_DF              0x20U  /* drive fault */
#define ATA_STATUS_DRDY            0x40U  /* drive ready */
#define ATA_STATUS_BSY             0x80U  /* busy */

/* ATA error register bits (when ATA_STATUS_ERR set). */
#define ATA_ERR_ABRT               0x04U  /* command aborted */
#define ATA_ERR_IDNF               0x10U  /* ID not found (LBA out of range) */
#define ATA_ERR_UNC                0x40U  /* uncorrectable data */

/* ATA device/head register bits. */
#define ATA_DEV_HEAD_LBA_BIT       0x40U  /* must be set for LBA mode */
#define ATA_DEV_HEAD_NUM_MASK      0x10U  /* 0 = master, 1 = slave */
#define ATA_DEV_HEAD_HEAD_MASK     0x0FU  /* LBA[27:24] in LBA mode */

/* ATA device control register bits. */
#define ATA_DEVCTRL_NIEN           0x02U  /* disable INTRQ */
#define ATA_DEVCTRL_SRST           0x04U  /* software reset */
#define ATA_DEVCTRL_HOB            0x80U  /* high order byte (LBA48 shadow) */

/* ========================================================================
 * ATA command codes.
 *
 * The emulation implements the whole PIO-era ATA/IDE command set the
 * Nextor kernel (and generic MSX ATA software) may issue. Commands
 * the backing file cannot meaningfully execute (SMART, security
 * erase, ATAPI packet...) abort with ERR|ABRT, which is the spec
 * behaviour for an unsupported command.
 * ====================================================================== */

#define ATA_CMD_NOP                0x00U
#define ATA_CMD_DEVICE_RESET       0x08U
#define ATA_CMD_RECALIBRATE        0x10U  /* 0x10..0x1F all accepted */
#define ATA_CMD_RECALIBRATE_MAX    0x1FU
#define ATA_CMD_READ_SECTORS       0x20U
#define ATA_CMD_READ_SECTORS_NR    0x21U  /* no-retry variant */
#define ATA_CMD_READ_LONG          0x22U  /* obsolete - ABRT */
#define ATA_CMD_READ_LONG_NR       0x23U  /* obsolete - ABRT */
#define ATA_CMD_READ_SECTORS_EXT   0x24U  /* LBA48 */
#define ATA_CMD_READ_DMA_EXT       0x25U  /* LBA48 */
#define ATA_CMD_READ_MULTI_EXT     0x29U  /* LBA48 */
#define ATA_CMD_WRITE_SECTORS      0x30U
#define ATA_CMD_WRITE_SECTORS_NR   0x31U  /* no-retry variant */
#define ATA_CMD_WRITE_LONG         0x32U  /* obsolete - ABRT */
#define ATA_CMD_WRITE_LONG_NR      0x33U  /* obsolete - ABRT */
#define ATA_CMD_WRITE_SECTORS_EXT  0x34U  /* LBA48 */
#define ATA_CMD_WRITE_DMA_EXT      0x35U  /* LBA48 */
#define ATA_CMD_WRITE_MULTI_EXT    0x39U  /* LBA48 */
#define ATA_CMD_WRITE_VERIFY       0x3CU
#define ATA_CMD_READ_VERIFY        0x40U
#define ATA_CMD_READ_VERIFY_NR     0x41U  /* no-retry variant */
#define ATA_CMD_READ_VERIFY_EXT    0x42U  /* LBA48 */
#define ATA_CMD_SEEK               0x70U
#define ATA_CMD_DEVICE_DIAG        0x90U
#define ATA_CMD_INIT_DEV_PARAMS    0x91U
#define ATA_CMD_PACKET             0xA0U  /* ATAPI - ABRT (not a packet dev) */
#define ATA_CMD_IDENTIFY_PACKET    0xA1U  /* ATAPI - ABRT (not a packet dev) */
#define ATA_CMD_SMART              0xB0U  /* ABRT (not claimed in IDENTIFY) */
#define ATA_CMD_DEV_CONFIG_FREEZE  0xB1U
#define ATA_CMD_READ_MULTI         0xC4U
#define ATA_CMD_WRITE_MULTI        0xC5U
#define ATA_CMD_SET_MULTI          0xC6U
#define ATA_CMD_READ_DMA           0xC8U
#define ATA_CMD_READ_DMA_NR        0xC9U
#define ATA_CMD_WRITE_DMA          0xCAU
#define ATA_CMD_WRITE_DMA_NR       0xCBU
#define ATA_CMD_GET_MEDIA_STATUS   0xDAU
#define ATA_CMD_MEDIA_LOCK         0xDEU
#define ATA_CMD_MEDIA_UNLOCK       0xDFU
#define ATA_CMD_STANDBY_IMMEDIATE  0xE0U
#define ATA_CMD_IDLE_IMMEDIATE     0xE1U
#define ATA_CMD_STANDBY            0xE2U
#define ATA_CMD_IDLE               0xE3U
#define ATA_CMD_CHECK_POWER_MODE   0xE5U
#define ATA_CMD_SLEEP              0xE6U
#define ATA_CMD_FLUSH_CACHE        0xE7U
#define ATA_CMD_READ_BUFFER        0xE8U
#define ATA_CMD_WRITE_BUFFER       0xE9U
#define ATA_CMD_FLUSH_CACHE_EXT    0xEAU  /* LBA48 */
#define ATA_CMD_IDENTIFY           0xECU
#define ATA_CMD_MEDIA_EJECT        0xEDU
#define ATA_CMD_SET_FEATURES       0xEFU
#define ATA_CMD_SECURITY_SET_PW    0xF1U  /* ABRT */
#define ATA_CMD_SECURITY_UNLOCK    0xF2U  /* ABRT */
#define ATA_CMD_SECURITY_ERASE_PRE 0xF3U  /* ABRT */
#define ATA_CMD_SECURITY_ERASE     0xF4U  /* ABRT */
#define ATA_CMD_SECURITY_FREEZE    0xF5U
#define ATA_CMD_SECURITY_DISABLE   0xF6U  /* ABRT */
#define ATA_CMD_SET_MAX            0xF9U  /* ABRT */

/* ========================================================================
 * PATA device signature (set on IDENTIFY / DEVICE RESET / power-on).
 * ====================================================================== */

#define ATA_SIG_ERROR              0x01U
#define ATA_SIG_SECTOR_COUNT       0x01U
#define ATA_SIG_CYLINDER_LOW       0x00U
#define ATA_SIG_CYLINDER_HIGH      0x00U
#define ATA_SIG_DEVICE_HEAD        0x00U
#define ATA_SIG_STATUS             ATA_STATUS_DRDY

/* ========================================================================
 * Sector buffer / state machine
 * ====================================================================== */

/* Sub-state of an in-flight PIO transfer. */
typedef enum {
    SUNRISE_IDE_STATE_IDLE       = 0,    /* no command in progress */
    SUNRISE_IDE_STATE_READ_BUSY,         /* READ - file fetch in flight */
    SUNRISE_IDE_STATE_READY,             /* sector buffer ready, draining */
    SUNRISE_IDE_STATE_WRITE_BUSY,        /* WRITE - buffering, then file */
} Sunrise_IDE_State;

/* File-backend lifecycle (background state machine - driven from the
 * main loop; mounts the USB stick, opens nextor.img, extends it to
 * the hard-coded size, then serves ATA I/O). */
typedef enum {
    SUNRISE_IDE_USB_INIT = 0,   /* waiting to start */
    SUNRISE_IDE_USB_ENUM,       /* mounting the USB stick via FatFs */
    SUNRISE_IDE_USB_OPEN,       /* opening 0:/nextor.img */
    SUNRISE_IDE_USB_EXTEND,     /* zero-extending the image (chunked) */
    SUNRISE_IDE_USB_READY,      /* image ready - serving ATA I/O */
} Sunrise_IDE_USB_State;

/* Main runtime state struct. Lives in regular SRAM. */
typedef struct {
    /* ROM bank state. */
    uint8_t  segment;             /* current 16 KiB bank (after bit-reverse) */
    uint8_t  ide_enabled;         /* 0 = pure ROM, 1 = IDE window open */

    /* ATA task-file register file (current content). */
    uint8_t  reg_error;           /* read: error / write: features latch */
    uint8_t  reg_sector_count;
    uint8_t  reg_sector_number;
    uint8_t  reg_cylinder_low;
    uint8_t  reg_cylinder_high;
    uint8_t  reg_device_head;
    uint8_t  reg_status;          /* last latched status (BSY/DRQ/DRDY/...) */
    uint8_t  reg_device_ctrl;     /* device control: SRST, NIEN, HOB */

    /* HOB shadow registers ("previous content" for LBA48 EXT
     * commands). Written when Device Control bit 7 (HOB) is set. */
    uint8_t  hob;                 /* mirror of DEVCTRL bit 7 */
    uint8_t  sh_sector_count;
    uint8_t  sh_sector_number;
    uint8_t  sh_cylinder_low;
    uint8_t  sh_cylinder_high;

    /* SET MULTIPLE MODE block size (accepted; transfers still run one
     * 512-byte sector at a time, which is ATA-legal). */
    uint8_t  multi_count;

    /* Sector buffer + state. */
    Sunrise_IDE_State  state;
    uint8_t  sector_buffer[512];
    uint16_t buffer_index;       /* next byte offset to serve (0..511) */
    uint16_t buffer_length;      /* always 512 */
    uint8_t  lba_pending;        /* 1 = next call to Sunrise_IDE_Service
                                    should fetch the sector from the file;
                                    cleared once the read completes */
    uint32_t sectors_remaining;  /* Sectors left in the current transfer
                                    (16-bit counts are possible with
                                    LBA48: 0x0000 means 65536). */

    /* File-backend lifecycle. */
    Sunrise_IDE_USB_State usb_state;

    /* 512-byte IDENTIFY DEVICE response (fixed, built at init). */
    uint8_t  identify_buf[512];
    uint8_t  identify_built;      /* 1 once identify_buf is populated */

    /* Diagnostic counters (incremented in IRQ context, read in main loop). */
    volatile uint32_t stat_drains;        /* PIO sectors fully drained */
    volatile uint32_t stat_atacmds;       /* ATA command writes (0x7E07) */
    volatile uint32_t stat_tf_writes;     /* task-file register writes */
    volatile uint32_t stat_status_reads;  /* STATUS register reads (0x7E07) */
    volatile uint8_t  stat_last_cmd;      /* last ATA command byte */
} Sunrise_IDE;

/* ========================================================================
 * Public API
 * ====================================================================== */

/* Initialise the state struct. Safe to call any number of times. Does
 * NOT touch the USB stack - USB_Initialization() must run first. */
void Sunrise_IDE_Init (void);

/* Main-loop service: drives the file-backend lifecycle (mount USB ->
 * open/extend nextor.img) and services in-flight READ/WRITE requests
 * with blocking FatFs calls. Must be called from main(), never from
 * IRQ context. */
void Sunrise_IDE_Service (void);

/* Read handler: returns the byte the MSX should see on a cart read.
 * Called from Cart_EXTI0_Sunride_Handler (IRQ context). MUST be
 * IRQ-safe - no blocking, no printf, no FatFs calls. */
uint8_t Sunrise_IDE_ReadByte (uint16_t address);

/* Write handler: applies a cart write from the MSX. Same constraints
 * as the read handler. */
void Sunrise_IDE_WriteByte (uint16_t address, uint8_t value);

/* Read-only accessor for the live state struct (diagnostic counters,
 * file lifecycle, etc.). Used by the main-loop status printer. */
const Sunrise_IDE *Sunrise_IDE_GetState (void);

#ifdef __cplusplus
}
#endif

#endif /* __SUNRISE_IDE_H */
