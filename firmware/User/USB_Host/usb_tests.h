/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_tests.h
 * Description        : Standalone USB + FAT integration tests for the
 *                      RISKYMSX2 firmware.  All diagnostic and test
 *                      entry points that are NOT part of the runtime
 *                      USB stack live here, so usb_disk.c stays purely
 *                      the driver glue (enumeration glue, SCSI, FatFs
 *                      volume helpers).
 *
 *                      Tests included:
 *                        USB_Tests_Enumerate  - verbose enumeration
 *                                              diagnostic (USBD)
 *                        USB_Tests_Scsi      - low-level SCSI read test
 *                                              (UREAD)
 *                        USB_Tests_Fat       - FAT integration test:
 *                                              mount, list, create a
 *                                              text file, read it
 *                                              back, verify (FAT)
 *                        USB_Tests_RunAll    - all of the above in
 *                                              order, with retries
 *********************************************************************************/

#ifndef __USB_TESTS_H
#define __USB_TESTS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Verbose enumeration diagnostic.  Walks the full USBHS host +
 * enumeration + SCSI READ CAPACITY pipeline, printing every step +
 * REQUEST SENSE data after each failure.  Does NOT call f_mount, so
 * this is safe to run even if the stick refuses to enumerate.
 * Publishes usb_in_ep/usb_out_ep/RootHubDev state on success so the
 * later tests can use the bulk pipe. */
void USB_Tests_Enumerate (void);

/* Low-level USB drive read test. Walks the full SCSI command set
 * (INQUIRY, TEST UNIT READY, READ CAPACITY(10), READ(10)) and prints
 * the stick's identity, capacity, MBR partition table, spot reads and
 * a multi-sector throughput number.  Does NOT touch FATFS.
 * Requires a successfully enumerated device (run Enumerate first). */
void USB_Tests_Scsi (void);

/* Full FAT integration test: f_mount, list root directory, create
 * 0:/RISKYMSX2.TXT with a few lines of text, read it back, verify
 * byte-for-byte, and hexdump the first 256 bytes of the first regular
 * file in /.  Returns 0 on success.  Requires an enumerated device
 * (run Enumerate first). */
uint8_t USB_Tests_Fat (void);

/* Run the whole suite with cold-start retries: settle delay, then up
 * to `attempts` rounds of (re-init controller -> enumerate -> SCSI
 * test -> FAT test).  Stops after the first fully successful round.
 * Used by main() at boot; each step can also be driven by hand via
 * the CLI (USBD / UREAD / FAT). */
void USB_Tests_RunAll (int attempts);

#ifdef __cplusplus
}
#endif

#endif /* __USB_TESTS_H */