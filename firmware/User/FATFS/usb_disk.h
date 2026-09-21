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
uint8_t usb_scsi_write_sector (uint32_t lba, const uint8_t *buf, uint32_t block_size);
uint8_t usb_scsi_request_sense (uint8_t *buf, uint16_t len);
uint8_t usb_scsi_inquiry (uint8_t *buf, uint16_t len);
uint8_t usb_scsi_test_unit_ready (void);
uint8_t msc_mass_storage_reset (void);

/* Clear enumerator state (called by disk_initialize before re-enum). */
void ClearUSB (void);

/* Endpoint addresses found during enumeration.  Exposed here so the
 * CLI status printout can show them without an extra accessor. */
extern uint8_t  usb_in_ep;
extern uint8_t  usb_out_ep;

/* Descriptor scratch buffers used by the enumeration diagnostic
 * (usb_tests.c reads them).  DevDesc_Buf holds the 18-byte device
 * descriptor; Com_Buffer holds the configuration descriptor set. */
extern __attribute__ ((aligned (4))) uint8_t DevDesc_Buf[18];
extern __attribute__ ((aligned (4))) uint8_t Com_Buffer[];

/* Publish the bulk endpoint pair discovered during enumeration into
 * the global usb_in_ep/usb_out_ep (and reset the data toggles).  The
 * enumeration diagnostic and USBH_EnumRootDevice both call this once
 * the MSC interface has been parsed. */
void USB_PublishEndpoints (uint8_t in_ep, uint8_t out_ep);

/* Single-shot SCSI READ CAPACITY helpers (used by the enumeration
 * diagnostic).  usb_scsi_read_capacity below is the retrying version
 * used by production paths. */
uint8_t USB_ScsiReadCapacityOnce  (uint32_t *block_count,
                                   uint32_t *block_size);
uint8_t USB_ScsiReadCapacity16Once (uint32_t *block_count,
                                    uint32_t *block_size);

/* Single FATFS volume handle (mounted once after first successful
 * enumeration). */
extern FATFS g_fatfs;

/* --- High-level helpers used by the CLI ------------------------------- */

/* Optional progress callback, invoked by USB_FileToPSRAM once per
 * chunk (every s_sector_buf bytes). Lets the caller draw a progress
 * bar / percentage on the MSX terminal while a multi-megabyte ROM is
 * being streamed into PSRAM (8 MiB takes 10-20 s over USB FS - long
 * enough that a static "Loading" line looks like a crash).
 *
 * Set to NULL to disable. Called with (bytes_copied_so_far,
 * total_bytes_planned). Keep the body SHORT - it runs inside the
 * read loop and slows the copy down. */
extern void (*USB_ProgressCB) (uint32_t done, uint32_t total);

/* Read `len` bytes of `path` into the PSRAM buffer at `psram_addr`.
 * If `len` is 0, the file size is used. Uses FATFS to seek/open/read
 * and PSRAM DMA to push each chunk into PSRAM. `len` is capped at the
 * available cart-image space. Returns number of bytes actually copied,
 * or 0 on error. */
uint32_t USB_FileToPSRAM (const char *path, uint32_t psram_addr, uint32_t len);

/* Print the first `max` directory entries of `path` to the UART.
 * Used by the LS CLI command. */
void USB_ListDir (const char *path, uint16_t max);

/* Lazy-mount helper: poll the root hub, enumerate if needed, f_mount()
 * if not already mounted.  Returns DEF_SUCCESS on success, DEF_ERR_*
 * otherwise.  Call before any FatFs API (f_open, f_opendir, etc.) so a
 * missed `USB` step doesn't silently produce FR_NO_FILESYSTEM. */
uint8_t USB_TryEnsureMounted (void);

/* NOTE: the standalone tests (verbose enumeration diagnostic, SCSI
 * read test, FAT integration test) now live in USB_Host/usb_tests.{c,h}
 * - see USB_Tests_Enumerate / USB_Tests_Scsi / USB_Tests_Fat /
 * USB_Tests_RunAll.  This module is purely driver glue. */

#endif /* __USB_DISK_H */