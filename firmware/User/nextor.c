/********************************** (C) COPYRIGHT *******************************
 * File Name          : nextor.c
 * Description        : Nextor mailbox engine (see nextor.h).
 *
 *                      Layering (docs/NEXTOR_PLAN.md section 2):
 *                        Nextor kernel (flash)  - filesystem, FAT, DOS API
 *                        driver.asm (Z80, cart) - Nextor v3 contract,
 *                                                 mailbox client
 *                        cart.c EXTI0 handler   - mailbox byte pump
 *                        nextor.c (this file)   - command engine, raw SCSI
 *                        USBHS host stack       - SCSI READ(10)/WRITE(10)
 *
 *                      The firmware NEVER interprets partition tables or
 *                      FAT structures here: the kernel sees the whole
 *                      medium. This mirrors the layering of every real
 *                      Nextor block driver.
 *********************************************************************************/

#include "nextor.h"
#include "cart.h"
#include "usb_disk.h"
#include "ch32v4x7.h"
#include <stdio.h>
#include <string.h>

/* Firmware version byte served in the handshake + at mailbox 0x7FF5. */
#define NEXTOR_FW_VERSION  0x01U

NextorMailbox g_nextor_mbox;

/* ------------------------------------------------------------------ */
/* Mailbox lifecycle                                                    */
/* ------------------------------------------------------------------ */

void Nextor_Reset (void) {
    memset ((void *)&g_nextor_mbox, 0, sizeof (g_nextor_mbox));
    g_nextor_mbox.state = NEXTOR_IDLE;
    g_nextor_mbox.status = NEXTOR_ST_READY;
    Nextor_MapperInit ();
}

/* ------------------------------------------------------------------ */
/* Cart-side memory-mapper emulation (see nextor.h)                     */
/* ------------------------------------------------------------------ */

NextorMapperState g_nx_mapper;
volatile uint32_t g_nx_cmd_count = 0U;

void Nextor_MapperInit (void) {
    g_nx_mapper.page_reg[0] = 0U;
    g_nx_mapper.page_reg[1] = 0U;
    g_nx_mapper.page_reg[2] = 0U;
    g_nx_mapper.page_reg[3] = 0U;
    g_nx_mapper.ram_base = PSRAM_CART_BASE + NEXTOR_MAPPER_RAM_OFF;
    g_nx_mapper.armed = 1U;
}

/* Compose the STATUS byte the IRQ serves at mailbox 0x7FF0:
 *   READY always (the firmware answers), DONE when the pending command
 *   completed, ERR bit when err != 0, RX_AVAIL when result bytes wait. */
static void nextor_compose_status (void) {
    uint8_t s = NEXTOR_ST_READY;
    if (g_nextor_mbox.err != NEXTOR_ERR_NONE) {
        s |= NEXTOR_ST_ERR;
    }
    if (g_nextor_mbox.rx_n != 0U) {
        s |= NEXTOR_ST_RXAVL;
    }
    g_nextor_mbox.status = s;
}

/* ------------------------------------------------------------------ */
/* Media presence + change latch                                        */
/* ------------------------------------------------------------------ */

/* USB stick presence: poll the root hub once per main-loop pass.
 * USBH_PreDeal() returns 0 (enumerated) or 0xFF (nothing new to do but
 * a device is present) while a stick is plugged in - same test the
 * FatFS disk_initialize uses. Anything else = no device. */
static uint8_t nextor_media_present (void) {
    uint8_t res = USBH_PreDeal ();
    return (res == 0U || res == 0xFFU) ? 1U : 0U;
}

/* Update the media-change latch:
 *   absent -> present : latch "changed" + mark media available (covers
 *                       both "stick was inserted" and the boot-time race
 *                       where enumeration completed after driver init).
 *   present -> absent : latch "changed", drop the capacity cache. */
static void Nextor_MediaPoll (void) {
    static uint8_t s_last_present = 0U;   /* starts "no media" */

    uint8_t present = nextor_media_present ();
    if (present != s_last_present) {
        g_nextor_mbox.change_latch = 1U;
        s_last_present = present;
    }
    if (present) {
        g_nextor_mbox.media_ok = 1U;
    } else {
        g_nextor_mbox.media_ok = 0U;
        g_nextor_mbox.cap_valid = 0U;
    }
}

/* ------------------------------------------------------------------ */
/* Main-loop command engine                                             */
/* ------------------------------------------------------------------ */

/* Serve one pending command. Called with have_cmd == 1; on return the
 * status byte carries DONE (+ERR / RX_AVAIL). */
static void Nextor_RunCommand (void) {
    switch (g_nextor_mbox.cmd) {

    case NEXTOR_CMD_HANDSHAKE: {
        /* "RNX2" + firmware version, little-endian byte order. */
        volatile uint8_t *rx = (volatile uint8_t *)NEXTOR_RX_BUF;
        rx[0] = (uint8_t)'R';
        rx[1] = (uint8_t)'N';
        rx[2] = (uint8_t)'X';
        rx[3] = (uint8_t)'2';
        rx[4] = NEXTOR_FW_VERSION;
        g_nextor_mbox.rx_n = 5U;
        break;
    }

    case NEXTOR_CMD_CAPACITY: {
        uint32_t last_lba = 0U, bsize = NEXTOR_SECTOR_SIZE;
        if (usb_scsi_read_capacity (&last_lba, &bsize) != 0U) {
            /* No medium / not enumerated: zero-capacity device (the
             * kernel treats total sectors 0 as "unknown capacity"). */
            last_lba = 0U;
            bsize    = NEXTOR_SECTOR_SIZE;
            g_nextor_mbox.cap_valid = 0U;
        } else {
            g_nextor_mbox.cap_blocks = last_lba + 1U;   /* READ CAPACITY
                                                        * returns LAST lba */
            g_nextor_mbox.cap_size   = bsize;
            g_nextor_mbox.cap_valid  = 1U;
        }
        volatile uint8_t *rx = (volatile uint8_t *)NEXTOR_RX_BUF;
        {
            uint32_t blocks = g_nextor_mbox.cap_blocks;
            rx[0] = (uint8_t)(blocks);
            rx[1] = (uint8_t)(blocks >> 8);
            rx[2] = (uint8_t)(blocks >> 16);
            rx[3] = (uint8_t)(blocks >> 24);
            rx[4] = (uint8_t)(bsize);
            rx[5] = (uint8_t)(bsize >> 8);
            rx[6] = (uint8_t)(bsize >> 16);
            rx[7] = (uint8_t)(bsize >> 24);
        }
        g_nextor_mbox.rx_n = 8U;
        break;
    }

    case NEXTOR_CMD_STATUS:
    case NEXTOR_CMD_STAPEEK: {
        /* 0 = no media, 1 = ready, 2 = changed. STATUS consumes the
         * latch (the kernel's geometry refresh relies on exactly-one
         * "changed" report); STAPEEK leaves it alone so the device
         * availability query does not suppress the next change report. */
        uint8_t b = 0U;
        if (g_nextor_mbox.media_ok) {
            b = g_nextor_mbox.change_latch ? 2U : 1U;
            if (g_nextor_mbox.cmd == NEXTOR_CMD_STATUS) {
                g_nextor_mbox.change_latch = 0U;   /* consumed */
            }
        }
        volatile uint8_t *rx = (volatile uint8_t *)NEXTOR_RX_BUF;
        rx[0] = b;
        g_nextor_mbox.rx_n = 1U;
        break;
    }

    case NEXTOR_CMD_READ: {
        /* The IRQ already latched the LBA and parked state == IDLE. */
        if (usb_scsi_read_sector (g_nextor_mbox.lba,
                                  (uint8_t *)NEXTOR_RX_BUF,
                                  NEXTOR_SECTOR_SIZE) != 0U) {
            g_nextor_mbox.err = NEXTOR_ERR_SCSI;
            g_nextor_mbox.rx_n = 0U;
        } else {
            g_nextor_mbox.rx_n = NEXTOR_SECTOR_SIZE;
        }
        break;
    }

    case NEXTOR_CMD_WRITE: {
        /* Runs after the IRQ collected all 512 bytes into the TX buffer. */
        if (usb_scsi_write_sector (g_nextor_mbox.lba,
                                   (const uint8_t *)NEXTOR_TX_BUF,
                                   NEXTOR_SECTOR_SIZE) != 0U) {
            g_nextor_mbox.err = NEXTOR_ERR_SCSI;
        }
        g_nextor_mbox.rx_n = 0U;   /* write: no result bytes to drain */
        break;
    }

    case NEXTOR_CMD_ABORT:
        /* Cancel: drop any half-collected transfer state. */
        g_nextor_mbox.state = NEXTOR_IDLE;
        g_nextor_mbox.rx_n = 0U;
        g_nextor_mbox.tx_n = 0U;
        break;

    default:
        g_nextor_mbox.err = NEXTOR_ERR_SCSI;
        break;
    }

    /* Compose the in-flight status, then mark the command completed:
     * the driver's poll loop watches the DONE bit. */
    nextor_compose_status ();
    g_nextor_mbox.status |= NEXTOR_ST_DONE;
}

void Nextor_Service (void) {
    Nextor_MediaPoll ();

    if (!g_nextor_mbox.have_cmd) {
        return;
    }
    g_nextor_mbox.have_cmd = 0U;

    /* Log the low-frequency commands (handshake/capacity/status) -
     * these trace the driver bring-up on USART. Per-sector READ/WRITE
     * commands are deliberately not logged (they would flood the log
     * during disk I/O). */
    if (g_nextor_mbox.cmd != NEXTOR_CMD_READ
        && g_nextor_mbox.cmd != NEXTOR_CMD_WRITE) {
        printf ("NEXTOR: cmd=0x%02X lba=%u\r\n",
                (unsigned)g_nextor_mbox.cmd,
                (unsigned)g_nextor_mbox.lba);
    }

    Nextor_RunCommand ();
}

/* ------------------------------------------------------------------ */
/* Boot path                                                            */
/* ------------------------------------------------------------------ */

void Boot_Nextor (void) {
    printf ("NEXTOR: boot - installing CART_MAP_NEXTOR (flash-served)\r\n");
    /* No PSRAM copy: the kernel lives in nextor_rom[] (.cartrom). No
     * ~RESET drive: the MSX-side caller (terminal GRAPH gate stub /
     * loader CMD_BOOT_NEXTOR) performs the soft reset that re-probes
     * the slots once the new handler is live. */
    (void)Cart_SetMapper_Safe (CART_MAP_NEXTOR);
}
