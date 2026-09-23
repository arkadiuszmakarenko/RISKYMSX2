/*
 * sunrise_ide.h - Sunrise IDE emulation for the RISKYMSX2 cartridge.
 *
 * This implements the cartridge-side half of the "Sunrise IDE" mapper
 * (the same one Carnivore2 / GR8NET / many other modern MSX IDE
 * cartridges use), with the IDE bus backed by the CH32V407's USBHS
 * host (USB stick = "HDD0"). Reference implementation ported from
 * the PicoVerse 2040 sunrise_ide.c (cristianoag/msx-picoverse-public).
 *
 * Why Sunrise IDE instead of an ASCII16 + mailbox protocol:
 *
 *   Sunrise IDE is the *standard* MSX ATA mapper. The Nextor kernel
 *   ships a "Sunrise IDE Master Only" driver whose ATA register /
 *   data / control window matches the Carnivore2 / Sunrise IDE
 *   reference exactly. No MSX-side porting work is needed - you drop
 *   in the stock NEXTOR.ROM and it boots straight into a USB-stick-
 *   backed Nextor.
 *
 * Memory map of the Sunrise IDE mapper (Z80 side, 0x4000..0x7FFF):
 *
 *   0x4000..0x40FF  current 16 KiB bank (lower half served from ROM,
 *                   upper half mirrored)
 *   0x4100..0x4103  current 16 KiB bank (continued - mirror)
 *   0x4104          control register (bank bit-reversed + IDE enable)
 *   0x4105..0x7BFF  bank continuation (mirrored)
 *   0x7C00..0x7DFF  IDE data register (16-bit latch, PIO mode)
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
 * IDE register / data window decoding is gated by bit 7 of the last
 * 0x4104 write: when bit 7 = 0 the cart just serves the ROM; when bit
 * 7 = 1 the 0x7C00-0x7EFF window decodes as ATA registers + data
 * register (the rest of the bank is still served from ROM).
 *
 * ATA-side details:
 *
 *   - 16-bit data register latches on writes and reads from the
 *     512-byte sector buffer (low byte first, then high byte).
 *   - Task-file registers are mirrored every 16 bytes
 *     (0x7E00..0x7E0F = primary, 0x7E10..0x7E1F = same primary, ...).
 *   - LBA addressing is the 28-bit CHS-LBA form: high 4 bits in
 *     device/head, mid 8 in cylinder high, low 8 in cylinder low,
 *     sector count in sector count, sector in sector number. LBA is
 *     rebuilt on every register write.
 *   - LBA increments automatically as the MSX drains the 512-byte
 *     sector buffer through the data register.
 *   - IDENTIFY DEVICE (0xEC) returns a 512-byte response generated
 *     from the USB INQUIRY data + a fake CHS geometry so the kernel's
 *     LBA math succeeds.
 *
 * Lifecycle:
 *
 *   - sunrise_ide_init(): zero the state struct. Called from main().
 *   - sunrise_ide_service(): poll the USB stack, kick off INQUIRY /
 *     READ CAPACITY / IDENTIFY generation once the stick is up.
 *     Called every main-loop pass.
 *   - sunrise_ide_handle_read() / sunrise_ide_handle_write(): called
 *     by Cart_EXTI0_Sunride_Handler on each EXTI0 cart cycle.
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
/* IDE data register (16-bit, PIO mode). Two byte reads / writes
 * per 16-bit word (low byte first per ATA convention). */
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

/* ATA device/head register bits. */
#define ATA_DEV_HEAD_LBA_BIT       0x40U  /* must be set for LBA mode */
#define ATA_DEV_HEAD_NUM_MASK      0x10U  /* 0 = master, 1 = slave */
#define ATA_DEV_HEAD_HEAD_MASK     0x0FU  /* LBA[27:24] in LBA mode */

/* ATA device control register bits. */
#define ATA_DEVCTRL_NIEN           0x02U  /* disable INTRQ */
#define ATA_DEVCTRL_SRST           0x04U  /* software reset */
#define ATA_DEVCTRL_HOB            0x80U  /* high order byte (obsolete) */

/* ATA command codes. */
#define ATA_CMD_DEVICE_RESET       0x08U
#define ATA_CMD_READ_SECTORS       0x20U
#define ATA_CMD_WRITE_SECTORS      0x30U
#define ATA_CMD_DEVICE_DIAG        0x90U
#define ATA_CMD_IDENTIFY           0xECU
#define ATA_CMD_SET_FEATURES       0xEFU
#define ATA_CMD_READ_MULTI         0xC4U
#define ATA_CMD_WRITE_MULTI        0xC5U
#define ATA_CMD_STANDBY_IMMEDIATE  0xE0U
#define ATA_CMD_IDLE_IMMEDIATE     0xE1U
#define ATA_CMD_INIT_DEV_PARAMS    0x91U
#define ATA_CMD_PACKET             0xA0U  /* ATAPI - not supported */

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
    SUNRISE_IDE_STATE_READ_BUSY,         /* READ SECTORS - USB fetch in flight */
    SUNRISE_IDE_STATE_READY,             /* sector buffer ready, draining */
    SUNRISE_IDE_STATE_WRITE_BUSY,        /* WRITE SECTORS - USB write in flight */
} Sunrise_IDE_State;

/* USB lifecycle (background state machine - drives the identify /
 * capacity fetch once a stick is enumerated). */
typedef enum {
    SUNRISE_IDE_USB_INIT = 0,
    SUNRISE_IDE_USB_ENUM,
    SUNRISE_IDE_USB_INQUIRY,
    SUNRISE_IDE_USB_CAPACITY,
    SUNRISE_IDE_USB_READY,
} Sunrise_IDE_USB_State;

/* Main runtime state struct. Lives in regular SRAM. */
typedef struct {
    /* ROM bank state. */
    uint8_t  segment;             /* current 16 KiB bank (after bit-reverse) */
    uint8_t  ide_enabled;         /* 0 = pure ROM, 1 = IDE window open */

    /* ATA task-file register file. */
    uint8_t  reg_error;
    uint8_t  reg_sector_count;
    uint8_t  reg_sector_number;
    uint8_t  reg_cylinder_low;
    uint8_t  reg_cylinder_high;
    uint8_t  reg_device_head;
    uint8_t  reg_status;          /* last latched status (BSY/DRQ/DRDY/...) */
    uint8_t  reg_device_ctrl;     /* device control: SRST, NIEN */

    /* Currently latched LBA (rebuilt from the four LBA registers on
     * every register write). Used for READ/WRITE SECTORS commands. */
    uint32_t current_lba;

    /* Sector buffer + state. */
    Sunrise_IDE_State  state;
    uint8_t  sector_buffer[512];
    uint16_t buffer_index;       /* next byte offset to serve (0..511) */
    uint16_t buffer_length;      /* typically 512, smaller for short LBA tail */
    uint8_t  lba_pending;        /* 1 = next call to sunrise_service should
                                   * kick off a new USB read; cleared once
                                   * the read completes */
    uint16_t sectors_remaining;  /* Sectors left in a multi-sector xfer
                                   * (0 means "not in a multi-sector
                                   * transfer"). Mirrors reg_sector_count
                                   * semantics (0 == 256 per ATA spec) but
                                   * kept private so we don't have to
                                   * mutate reg_sector_count (the kernel
                                   * reads it back to verify geometry). */

    /* USB lifecycle. */
    Sunrise_IDE_USB_State usb_state;
    uint8_t  inquiry_buf[36];     /* SCSI INQUIRY response (vendor+product) */
    uint32_t block_count;        /* READ CAPACITY result (LBA28 max + 1) */
    uint32_t block_size;         /* typically 512 */

    /* 512-byte IDENTIFY DEVICE response, generated lazily. */
    uint8_t  identify_buf[512];
    uint8_t  identify_built;      /* 1 once identify_buf is populated */

    /* Error tracking. */
    uint8_t  last_cmd_status;    /* 0 = OK, 1 = error (sets ATA_STATUS_ERR) */

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

/* Main-loop service: drives the USB lifecycle (enumerate -> INQUIRY ->
 * READ CAPACITY -> build IDENTIFY) and services in-flight READ/WRITE
 * requests. Must be called from main(), never from IRQ context. */
void Sunrise_IDE_Service (void);

/* Read handler: returns the byte the MSX should see on a cart read.
 * Called from Cart_EXTI0_Sunride_Handler (IRQ context). MUST be
 * IRQ-safe - no blocking, no printf, no long loops. */
uint8_t Sunrise_IDE_ReadByte (uint16_t address);

/* Write handler: applies a cart write from the MSX. Same constraints
 * as the read handler. */
void Sunrise_IDE_WriteByte (uint16_t address, uint8_t value);

/* Read-only accessor for the live state struct (diagnostic counters,
 * USB lifecycle, etc.). Used by the main-loop status printer. */
const Sunrise_IDE *Sunrise_IDE_GetState (void);

#ifdef __cplusplus
}
#endif

#endif /* __SUNRISE_IDE_H */
