/********************************** (C) COPYRIGHT *******************************
 * File Name          : nextor.h
 * Description        : Nextor mailbox engine for the RISKYMSX2.
 *
 *                      The Nextor kernel (nextor_rom[], embedded in flash,
 *                      served by the CART_MAP_NEXTOR mapper) runs a Z80
 *                      disk driver that talks to this firmware through the
 *                      cart-bus mailbox at 0x7FF0..0x7FFF. The EXTI0
 *                      handler (cart.c) latches commands and moves mailbox
 *                      bytes; Nextor_Service() (main loop) executes them -
 *                      raw SCSI reads/writes against the USB stick. No
 *                      FatFs in this path: the kernel owns the filesystem.
 *
 *                      Call Nextor_Service() from the main loop alongside
 *                      Loader_Service()/Terminal_Service(). It only acts
 *                      while a full mailbox command is pending.
 *********************************************************************************/

#ifndef __NEXTOR_H
#define __NEXTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mailbox register offsets within the cart window 0x7FF0..0x7FFF.
 * MUST match MSXSoftware/NextorDriver/driver.asm. */
#define NEXTOR_MB_STATUS     0x7FF0U  /* r: READY|DONE|ERR|RX_AVAIL    */
#define NEXTOR_MB_CMD        0x7FF0U  /* w: command byte               */
#define NEXTOR_MB_DATA       0x7FF1U  /* rw: result pop / arg push     */
#define NEXTOR_MB_RXCL       0x7FF2U  /* r: result count low           */
#define NEXTOR_MB_RXCH       0x7FF3U  /* r: result count high          */
#define NEXTOR_MB_ERR        0x7FF4U  /* r: last error code            */
#define NEXTOR_MB_VER        0x7FF5U  /* r: firmware version byte      */

/* STATUS bits (MUST match driver.asm). */
#define NEXTOR_ST_READY      0x80U    /* firmware alive                */
#define NEXTOR_ST_DONE       0x40U    /* last command completed        */
#define NEXTOR_ST_ERR        0x20U    /* command completed with error  */
#define NEXTOR_ST_RXAVL      0x10U    /* result bytes pending          */

/* Firmware version byte served in the handshake + at mailbox 0x7FF5
 * (shared by nextor.c and the cart.c EXTI0 handler). */
#define NEXTOR_FW_VERSION    0x01U

/* Commands (MUST match driver.asm). */
#define NEXTOR_CMD_HANDSHAKE 0x00U    /* -> "RNX2"+ver (5 bytes)       */
#define NEXTOR_CMD_CAPACITY  0x01U    /* -> blk count + blk size (8B)  */
#define NEXTOR_CMD_STATUS    0x02U    /* -> media status (1 byte,
                                       *    consumes the change latch) */
#define NEXTOR_CMD_READ      0x03U    /* <- LBA(4) -> 512 bytes        */
#define NEXTOR_CMD_WRITE     0x04U    /* <- LBA(4)+512 -> DONE         */
#define NEXTOR_CMD_ABORT     0x05U    /* cancel/reset engine           */
#define NEXTOR_CMD_STAPEEK   0x06U    /* -> media status, does NOT
                                       *    consume the change latch
                                       *    (protocol extension for the
                                       *    device availability query) */

/* Mailbox ERR port codes. */
#define NEXTOR_ERR_NONE      0x00U
#define NEXTOR_ERR_NOMEDIA   0x01U
#define NEXTOR_ERR_SCSI      0x02U
#define NEXTOR_ERR_TMO       0x03U

/* Sector payload size (fixed in v1; the driver drains/fills exactly
 * this much per command - docs/NEXTOR_PLAN.md D5). */
#define NEXTOR_SECTOR_SIZE   512U

/* ------------------------------------------------------------------ */
/* Cart-side memory-mapper emulation                                     */
/* ------------------------------------------------------------------ */
/* The Nextor kernel scans EVERY slot at page 2 for an MSX-standard
 * memory mapper (I/O ports 0xFC..0xFF = page registers + memory at
 * 0x8000-0xBFFF for page 2 / 0xC000-0xFFFF for page 3) and adopts the
 * largest one it finds as its primary mapper - then moves the system
 * RAM slot to it. With this emulation our cart slot becomes the DOS
 * system RAM on mapper-less machines (TFMSX). Segments live in PSRAM:
 * 32 x 16 KiB = 512 KiB, far away from the game image (bottom) and
 * the mailbox sector buffers (top 4 KiB). Registers are I/O-decoded
 * by Cart_EXTI95_IORQ_Handler (/IORQ is wired to PE8). */
#define NEXTOR_MAPPER_RAM_OFF   0x700000UL  /* PSRAM offset of segment 0 */
#define NEXTOR_MAPPER_SEGS      32U         /* 32 x 16 KiB = 512 KiB */
#define NEXTOR_MAPPER_SEG_MASK  0x1FU       /* high regs mirror (real HW) */
#define NEXTOR_MAPPER_SEGSIZE   16384U

typedef struct {
    volatile uint8_t  page_reg[4];   /* ports 0xFC..0xFF latched values */
    volatile uint32_t ram_base;      /* PSRAM address of segment 0 */
    volatile uint8_t  armed;         /* mapper emulation live */
} NextorMapperState;

extern NextorMapperState g_nx_mapper;

/* Debug/activity counter: incremented by the cart IRQ handler every
 * time a mailbox command is latched. The main loop watches it and
 * prints on change - it tells us whether the kernel's driver is
 * actually talking to the mailbox (the driver phase would otherwise
 * be silent in both the firmware log and, possibly, on screen). */
extern volatile uint32_t g_nx_cmd_count;
extern volatile uint32_t g_nx_cmd_attempts;  /* raw 0x7FF0 WRITE count
                                             * regardless of have_cmd
                                             * dedup; shows whether the
                                             * driver is writing CMD
                                             * bytes at all */
extern volatile uint32_t g_nx_stat_reads;   /* raw 0x7FF0 READ count
                                            * (STATUS polls the driver
                                            * does while waiting for
                                            * DONE) */
extern volatile uint32_t g_nx_irq_entry;    /* every EXTI0 IRQ entry */
extern volatile uint32_t g_nx_irq_late;     /* late-entry bail (SLTSL
                                            * already high) */
extern volatile uint32_t g_nx_irq_zero;     /* RD&WR both high bail */
extern volatile uint32_t g_nx_addr_counts[16]; /* address high-nibble
                                              * histogram - each IRQ
                                              * entry adds to the slot
                                              * for its top 12 bits */
extern volatile uint32_t g_nx_last_addr;   /* last seen address */
extern volatile uint32_t g_nx_reads;       /* RD=0 cycles served */
extern volatile uint32_t g_nx_writes;      /* WR=0 cycles served */

/* Debug/activity counter: incremented by the cart IRQ handler every
 * time the kernel writes the page-1 bank-select register at 0x6000
 * (ASCII16 CHGBNK). Watching this fires whenever the kernel decides
 * to page in bank 7 (the driver bank) - the driver probe doesn't
 * have to complete a full handshake for this to tick. With
 * g_nx_cmd_count staying at 0 + g_nx_bank_writes ticking on bank=7
 * we know the driver was selected but is silent (signalling that the
 * driver is paged in but the kernel hasn't reached its first disk
 * request yet - typical when sitting at the MSX-BASIC prompt). */
extern volatile uint32_t g_nx_bank_writes;
extern volatile uint8_t  g_nx_last_bank;   /* bank number of last 0x6000 write */
extern volatile uint32_t g_nx_bank7_selects;  /* counts writes of 7 to 0x6000
                                               * (driver bank select) */

/* Debug/activity counter: incremented by Nextor_MediaPoll on every
 * USB-stick insertion/removal transition. With this staying at 0
 * the kernel never even sees a media change - either the stick is
 * not enumerated or the kernel hasn't requested a status query. */
extern volatile uint32_t g_nx_media_changes;
extern volatile uint8_t  g_nx_media_now;   /* current media_ok */

/* Arm the mapper (called when the NEXTOR mapper installs). */
void Nextor_MapperInit (void);

/* PSRAM placement of the mailbox sector buffers: the very top of the
 * 8 MiB window, well clear of the game image (which lives at the bottom,
 * optionally shifted by CART_GAME_BASE). RX = results/sector reads,
 * TX = sector writes. */
#define NEXTOR_SECTOR_BUF_BASE  (PSRAM_CART_BASE + PSRAM_CART_SIZE - 0x1000U)
#define NEXTOR_RX_BUF           (NEXTOR_SECTOR_BUF_BASE)
#define NEXTOR_TX_BUF           (NEXTOR_SECTOR_BUF_BASE + NEXTOR_SECTOR_SIZE)

/* Mailbox state (shared between the EXTI0 IRQ handler in cart.c and
 * Nextor_Service() here). All fields touched by BOTH contexts are
 * single-writer where it matters:
 *   - cmd/args/arg_n/have_cmd: IRQ writes, main loop reads+clears.
 *   - status/err/rx_count: main loop writes, IRQ reads.
 *   - rx_idx/tx_n: IRQ advances (one byte per bus cycle), main loop
 *     resets at the start of each transfer phase.
 *   - change_latch/capacity: main loop only. */
typedef struct {
    volatile uint8_t  cmd;        /* latched command byte            */
    volatile uint8_t  arg_n;      /* LBA bytes still expected        */
    volatile uint8_t  have_cmd;   /* command complete, not yet served */
    volatile uint8_t  status;     /* NEXTOR_ST_* bits (IRQ serves it) */
    volatile uint8_t  err;        /* ERR code for the last command   */
    volatile uint32_t lba;        /* latched LBA (arg bytes, LE)     */
    volatile uint16_t rx_n;       /* result bytes ready to drain     */
    volatile uint16_t rx_idx;     /* DATA pop index into RX buffer   */
    volatile uint16_t tx_n;       /* DATA pushes collected for WRITE */
    volatile uint8_t  state;      /* NextorState                     */
    volatile uint8_t  change_latch; /* media changed since last STATUS */
    volatile uint8_t  media_ok;   /* stick was enumerated at least once */
    volatile uint32_t cap_blocks; /* block count from READ CAPACITY  */
    volatile uint32_t cap_size;   /* block size (512)                */
    volatile uint8_t  cap_valid;  /* capacity cache is live          */
} NextorMailbox;

extern NextorMailbox g_nextor_mbox;

/* States (state machine in nextor.c: IDLE -> ARGS -> XFER -> DONE). */
typedef enum {
    NEXTOR_IDLE = 0,   /* no command in flight                      */
    NEXTOR_ARGS,       /* collecting LBA bytes for READ/WRITE       */
    NEXTOR_WRITE_XFER, /* collecting 512 data bytes for WRITE       */
} NextorState;

/* Reset the mailbox (install-time; called from Cart_SetMapper). */
void Nextor_Reset (void);

/* Main-loop service: runs one pending mailbox command against the USB
 * stack (SCSI READ(10)/WRITE(10)/READ CAPACITY). Returns immediately
 * when no command is pending. */
void Nextor_Service (void);

/* Boot into the flash-resident Nextor kernel: installs CART_MAP_NEXTOR
 * (no PSRAM copy, no ~RESET drive - the MSX-side caller performs the
 * soft reset that makes the BIOS re-probe the cart). */
void Boot_Nextor (void);

#ifdef __cplusplus
}
#endif

#endif /* __NEXTOR_H */
