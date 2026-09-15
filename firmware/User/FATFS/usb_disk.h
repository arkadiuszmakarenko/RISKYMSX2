/********************************** (C) COPYRIGHT  *******************************
 * File Name          : usb_disk.h
 * Description        : Glue between the CH32V407 USBHS host stack and
 *                      the FatFs module.  Based on
 *                      v303Firmware/User/FATFS/usb_disk.h, ported from
 *                      USBFS to USBHS:
 *                        - USBFSH_*  ->  USBHSH_*
 *                        - data-toggle is uint16_t (USBHS keeps 2 bits)
 *                        - USBHS auto-detects speed in EnableRootHubPort,
 *                          so the explicit SetSelfSpeed() call from v303
 *                          is gone
 *                        - bulk max packet size is 512 (USBHS) instead of
 *                          64 (USBFS)
 *                        - sector-buffer + descriptor scratch buffers live
 *                          in PSRAM (0x80000000+) instead of internal SRAM
 *                          - the v407 silicon only has ~135 KB of SRAM
 *                          and that was the bottleneck for v303's USB
 *                          path; PSRAM has 8 MB and is on the HB bus at
 *                          core speed.
 *
 *                      Application layer (CLI, XLOAD) uses the public
 *                      API below to enumerate the USB stick and stream
 *                      files into PSRAM via DMA.
 *********************************************************************************/

#ifndef __USB_DISK_H
#define __USB_DISK_H

#include "debug.h"
#include "stdio.h"
#include "string.h"
#include "ch32v4x7_conf.h"
#include "ch32v407_usbhs_host.h"
#include "usb_host_config.h"
#include "ff.h"

/* Status Definitions */
#define DEF_SUCCESS                0x00
#define DEF_DEFAULT                0xFF
#define DEF_ERR_DETECT             0xF1   /* USB device not detected */
#define DEF_ERR_ENUM               0xF2   /* Host enumeration failure */
#define DEF_ERR_FILE               0xF3   /* File name incorrect / not found */
#define DEF_ERR_FLASH              0xF4   /* Flash operation failure */
#define DEF_ERR_VERIFY             0xF5   /* Flash data verify error */
#define DEF_ERR_LENGTH             0xF6   /* Flash data length verify error */
#define DEF_ERR_PSRAM_DMA          0xF7   /* PSRAM DMA copy failed */

/* USBHS bulk-IN max packet size (high-speed bulk endpoint). v303 used 64
 * for full-speed; USBHS high-speed bulk is 512. Many sticks report only
 * one of the two values, but capping the per-transfer read at 512 keeps
 * the bulk loop happy on every USBHS-attached MSC I've measured. */
#define USBHS_BULK_MAX_PACKET      512U

/* --- Public USB-host enumeration / SCSI API ------------------------------- */

/* Initialise the USBHS host controller + bookkeeping. Safe to call once
 * before main loop. */
void USB_Initialization (void);

/* USBH_EnumRootDevice: enumerate the device on the root hub.
 * Returns ERR_SUCCESS on success, ERR_USB_DISCON if no device, etc. */
uint8_t USBH_EnumRootDevice (uint8_t usb_port);

/* USBH_PreDeal: poll the root hub, kick off enumeration if a new device
 * appeared, or return a status code if the device went away. */
uint8_t USBH_PreDeal (void);

/* SCSI commands used by the disk layer */
uint8_t usb_scsi_read_capacity (uint32_t *block_count, uint32_t *block_size);
uint8_t usb_scsi_read_sector (uint32_t lba, uint8_t *buf, uint32_t block_size);
uint8_t usb_scsi_request_sense (uint8_t *buf, uint16_t len);
uint8_t msc_mass_storage_reset (void);

/* Clear enumerator state (called by disk_initialize before re-enum). */
void ClearUSB (void);

/* Endpoint addresses found during enumeration.  Exposed here so the
 * CLI status printout can show them without an extra accessor. */
extern uint8_t  usb_in_ep;
extern uint8_t  usb_out_ep;

/* Single FATFS volume handle (mounted once after first successful
 * enumeration). */
extern FATFS g_fatfs;

/* --- High-level helpers used by the CLI ------------------------------- */

/* Read `len` bytes of `path` into the PSRAM buffer at `psram_addr`.
 * Uses FATFS to seek/open/read and PSRAMDMAWrite to push the sector
 * into PSRAM. `len` is capped at the available cart-image space.
 * Returns number of bytes actually copied, or 0 on error. */
uint32_t USB_FileToPSRAM (const char *path, uint32_t psram_addr, uint32_t len);

/* Print the first `max` directory entries of `path` to the UART.
 * Used by the LS CLI command. */
void USB_ListDir (const char *path, uint16_t max);

/* Lazy-mount helper: poll the root hub, enumerate if needed, f_mount()
 * if not already mounted.  Returns DEF_SUCCESS on success, DEF_ERR_*
 * otherwise.  Call before any FatFs API (f_open, f_opendir, etc.) so a
 * missed `USB` step doesn't silently produce FR_NO_FILESYSTEM. */
uint8_t USB_TryEnsureMounted (void);

/* Verbose enumeration diagnostic.  Walks the full USBHS host +
 * enumeration + SCSI READ CAPACITY pipeline, printing every step +
 * REQUEST SENSE data after each failure.  Does NOT call f_mount, so
 * this is safe to run even if the stick refuses to enumerate. */
void USB_DiagEnumerate (void);

#endif /* __USB_DISK_H */