/********************************** (C) COPYRIGHT *******************************
 * File Name          : usb_tests.c
 * Description        : Standalone USB + FAT integration tests for the
 *                      RISKYMSX2 firmware (see usb_tests.h).  Split out
 *                      of usb_disk.c so the driver glue stays clean:
 *                      everything that prints a "diagnostic report"
 *                      rather than serving a runtime request lives here.
 *
 *                      The tests call the public usb_disk API
 *                      (USBHSH_* host primitives, usb_scsi_* commands,
 *                      FatFs f_* via g_fatfs) exactly like production
 *                      code does - they are just another consumer.
 *********************************************************************************/

#include "usb_tests.h"
#include "usb_disk.h"
#include "ch32v4x7.h"
#include "ch32v407_usbhs_host.h"
#include "usb_host_config.h"
#include "ff.h"
#include "debug.h"
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------------ */
/* Descriptor / SCSI pretty-printers                                        */
/* ------------------------------------------------------------------------ */

/* Print one row of a USB device descriptor field. */
static void print_desc_field (const char *name, uint16_t val) {
    printf ("    %-12s = 0x%04x (%u)\r\n", name, val, val);
}

/* Print the parsed device descriptor in human-readable form. */
static void dump_dev_descriptor (const uint8_t *buf) {
    PUSB_DEV_DESCR d = (PUSB_DEV_DESCR)buf;
    printf ("  Device Descriptor:\r\n");
    printf ("    bLength         = %u\r\n", d->bLength);
    printf ("    bDescriptorType = 0x%02x\r\n", d->bDescriptorType);
    print_desc_field ("bcdUSB",     d->bcdUSB);
    printf ("    bDeviceClass    = 0x%02x\r\n", d->bDeviceClass);
    printf ("    bDeviceSubClass = 0x%02x\r\n", d->bDeviceSubClass);
    printf ("    bDeviceProtocol = 0x%02x\r\n", d->bDeviceProtocol);
    printf ("    bMaxPacketSize0 = %u\r\n", d->bMaxPacketSize0);
    print_desc_field ("idVendor",   d->idVendor);
    print_desc_field ("idProduct",  d->idProduct);
    print_desc_field ("bcdDevice",  d->bcdDevice);
    printf ("    iManufacturer   = %u\r\n", d->iManufacturer);
    printf ("    iProduct        = %u\r\n", d->iProduct);
    printf ("    iSerialNumber   = %u\r\n", d->iSerialNumber);
    printf ("    bNumConfigurations = %u\r\n", d->bNumConfigurations);
}

/* Print the contents of a SCSI REQUEST SENSE reply. The first byte is
 * the response code (0x70 = current error, 0x71 = deferred), bytes
 * 2..3 are the sense key + additional sense code (ASC).  This is the
 * raw data that tells us why a SCSI command failed. */
static void dump_request_sense (const uint8_t *s, uint8_t n) {
    if (n < 4) {
        printf ("USB: REQUEST SENSE returned only %u bytes\r\n",
                (unsigned)n);
        return;
    }
    uint8_t resp_code    = s[0] & 0x7F;
    uint8_t sense_key    = s[2] & 0x0F;
    uint8_t asc          = s[12];
    uint8_t ascq         = s[13];
    const char *skname =
        (sense_key == 0x00) ? "NO SENSE"          :
        (sense_key == 0x01) ? "RECOVERED ERROR"   :
        (sense_key == 0x02) ? "NOT READY"         :
        (sense_key == 0x03) ? "MEDIUM ERROR"      :
        (sense_key == 0x04) ? "HARDWARE ERROR"    :
        (sense_key == 0x05) ? "ILLEGAL REQUEST"   :
        (sense_key == 0x06) ? "UNIT ATTENTION"    :
        (sense_key == 0x07) ? "DATA PROTECT"      :
        (sense_key == 0x0B) ? "ABORTED COMMAND"   :
        (sense_key == 0x0D) ? "VOLUME OVERFLOW"   :
                              "RESERVED";
    printf ("USB: REQUEST SENSE resp=%02x key=0x%02x (%s) "
            "asc=0x%02x ascq=0x%02x\r\n",
            (unsigned)resp_code, (unsigned)sense_key, skname,
            (unsigned)asc, (unsigned)ascq);
}

/* Pretty-print `len` bytes of `buf` as 16-byte rows of hex + ASCII. */
static void uread_hexdump (const uint8_t *buf, uint32_t len) {
    const uint32_t MAX = 512U;
    if (len > MAX) len = MAX;
    for (uint32_t off = 0; off < len; off += 16U) {
        printf ("    %04lx:", (unsigned long)off);
        uint32_t row = (len - off > 16U) ? 16U : (len - off);
        for (uint32_t i = 0; i < row; i++) {
            printf (" %02x", buf[off + i]);
        }
        for (uint32_t i = row; i < 16U; i++) printf ("   ");
        printf ("  ");
        for (uint32_t i = 0; i < row; i++) {
            uint8_t c = buf[off + i];
            printf ("%c", (c >= 0x20 && c < 0x7F) ? c : '.');
        }
        printf ("\r\n");
    }
}

/* ------------------------------------------------------------------------ */
/* Test 1: verbose enumeration diagnostic (USBD)                             */
/* ------------------------------------------------------------------------ */

void USB_Tests_Enumerate (void) {
    uint8_t port = DEF_USB_PORT;

    printf ("USBD: USBHS diagnostic enumeration\r\n");
    printf ("      Root hub state: status=%u dev_addr=%u ep0_max=%u speed=%u\r\n",
            (unsigned)RootHubDev[port].bStatus,
            (unsigned)RootHubDev[port].bAddress,
            (unsigned)RootHubDev[port].bEp0MaxPks,
            (unsigned)RootHubDev[port].bSpeed);

    /* Force a re-enumeration by clearing the root hub state. */
    ClearUSB ();
    printf ("USBD: cleared root hub state, polling port...\r\n");

    /* Step 1: poll the port.  At cold start the device may not have
     * signalled connect yet - retry for up to 3 s. */
    uint8_t port_status = ROOT_DEV_DISCONNECT;
    for (int poll = 0; poll < 30; poll++) {
        port_status = USBHSH_CheckRootHubPortStatus (RootHubDev[port].bStatus);
        if (port_status == ROOT_DEV_CONNECTED) break;
        if (port_status == ROOT_DEV_DISCONNECT) {
            /* Clear the connect-change flag and keep waiting. */
            Delay_Ms (100);
            continue;
        }
        break;
    }
    printf ("USBD: step 1 - port status = %u "
            "(0=DISCONNECT 1=CONNECTED 2=FAILED 3=SUCCESS)\r\n",
            (unsigned)port_status);

    if (port_status != ROOT_DEV_CONNECTED) {
        printf ("USBD: nothing plugged in or port error - "
                "connect a USB stick and try again\r\n");
        return;
    }

    /* Step 1b: bus reset.  A USB device must see SE0 signalling
     * (port reset) before it will respond to any control transfer.
     * Also clear UH_SOF_EN so EnableRootHubPort will read the speed
     * (CheckRootHubPortEnable returns SOF_EN; if it's already set,
     * speed detection is skipped and we get speed=0xFF). */
    printf ("USBD: step 1b - resetting root hub port...\r\n");
    USBHSH->CFG &= ~USBHS_UH_SOF_EN;
    USBHSH_ResetRootHubPort ();
    Delay_Ms (2);

    /* Step 2: enable the port + detect speed. */
    uint8_t speed = 0xFF;
    uint8_t rc_e = USBHSH_EnableRootHubPort (&speed);
    printf ("USBD: step 2 - EnableRootHubPort rc=%02x speed=%u ",
            (unsigned)rc_e, (unsigned)speed);
    if (rc_e == ERR_SUCCESS) {
        const char *sname =
            (speed == USB_HIGH_SPEED) ? "HIGH-SPEED" :
            (speed == USB_FULL_SPEED) ? "FULL-SPEED" :
            (speed == USB_LOW_SPEED)  ? "LOW-SPEED"  : "?";
        printf ("(%s)\r\n", sname);
    } else {
        printf ("\r\nUSBD: port enable failed (rc=%02x). "
                "Check cable/power.\r\n", (unsigned)rc_e);
        return;
    }

    /* Step 3: GET DESCRIPTOR (device, 18 bytes).  At cold start the
     * device may NAK the first request while it's still initialising -
     * retry with a short bus reset between attempts. */
    uint8_t ep0_size = 0;
    uint8_t rc_d = ERR_USB_TRANSFER;
    for (int desc_try = 0; desc_try < 3; desc_try++) {
        rc_d = USBHSH_GetDeviceDescr (&ep0_size, DevDesc_Buf);
        if (rc_d == ERR_SUCCESS) break;
        printf ("USBD: step 3 - GET_DESCRIPTOR(DEVICE) rc=%02x "
                "(attempt %d)\r\n", (unsigned)rc_d, desc_try + 1);
        if (desc_try < 2) {
            USBHSH->CFG &= ~USBHS_UH_SOF_EN;
            USBHSH_ResetRootHubPort ();
            Delay_Ms (5);
            USBHSH_EnableRootHubPort (&speed);
            Delay_Ms (50);
        }
    }
    printf ("USBD: step 3 - GET_DESCRIPTOR(DEVICE) rc=%02x ep0=%u\r\n",
            (unsigned)rc_d, (unsigned)ep0_size);
    if (rc_d != ERR_SUCCESS) {
        printf ("USBD: device descriptor fetch failed; "
                "stick may be in a weird state. Try replugging.\r\n");
        return;
    }
    dump_dev_descriptor (DevDesc_Buf);

    /* VID:PID sanity print. */
    {
        uint16_t vid = ((uint16_t)DevDesc_Buf[8])
                     | ((uint16_t)DevDesc_Buf[9] << 8);
        uint16_t pid = ((uint16_t)DevDesc_Buf[10])
                     | ((uint16_t)DevDesc_Buf[11] << 8);
        printf ("USBD: VID:PID = %04x:%04x\r\n", vid, pid);
    }

    /* Step 4: SET_ADDRESS. */
    uint8_t addr = (uint8_t)(port + USB_DEVICE_ADDR);
    uint8_t rc_a = USBHSH_SetUsbAddress (ep0_size, addr);
    printf ("USBD: step 4 - SET_ADDRESS(%u) rc=%02x\r\n",
            (unsigned)addr, (unsigned)rc_a);
    if (rc_a != ERR_SUCCESS) {
        printf ("USBD: SET_ADDRESS failed; stick rejected the address.\r\n");
        return;
    }
    Delay_Ms (5);

    /* Step 5: GET_DESCRIPTOR(CONFIG). */
    uint16_t cfglen = 0;
    uint8_t rc_c = USBHSH_GetConfigDescr (ep0_size, Com_Buffer,
                                          DEF_COM_BUF_LEN, &cfglen);
    printf ("USBD: step 5 - GET_DESCRIPTOR(CONFIG) rc=%02x cfg_len=%u\r\n",
            (unsigned)rc_c, (unsigned)cfglen);
    if (rc_c != ERR_SUCCESS || cfglen < 4) {
        printf ("USBD: config descriptor fetch failed.\r\n");
        return;
    }
    uint8_t cfg_val = ((PUSB_CFG_DESCR)Com_Buffer)->bConfigurationValue;
    printf ("USBD:   bConfigurationValue = %u, total length = %u\r\n",
            (unsigned)cfg_val, (unsigned)cfglen);

    /* Step 6: SET_CONFIGURATION. */
    uint8_t rc_sc = USBHSH_SetUsbConfig (ep0_size, cfg_val);
    printf ("USBD: step 6 - SET_CONFIGURATION(%u) rc=%02x\r\n",
            (unsigned)cfg_val, (unsigned)rc_sc);
    if (rc_sc != ERR_SUCCESS) {
        printf ("USBD: SET_CONFIGURATION failed.\r\n");
        return;
    }

    /* Step 7: scan the config descriptor for the MSC interface + bulk
     * endpoints.  Endpoints are only accepted after the MSC interface
     * is seen (composite sticks have other interfaces with their own
     * bulk endpoints that we must not steal). */
    uint8_t in_ep = 0, out_ep = 0;
    {
        uint8_t *p   = Com_Buffer;
        uint8_t *end = Com_Buffer + cfglen;
        uint8_t ifcnt = 0;
        uint8_t msc_found = 0;
        while (p < end) {
            if (p[1] == 0x04) {  /* INTERFACE */
                ifcnt++;
                uint8_t cls = p[5], sub = p[6], proto = p[7];
                printf ("USBD:   interface %u: class=0x%02x subclass=0x%02x "
                        "proto=0x%02x\r\n",
                        (unsigned)p[2], (unsigned)cls,
                        (unsigned)sub, (unsigned)proto);
                if (cls == 0x08 && sub == 0x06 && proto == 0x50) {
                    msc_found = 1;
                }
            } else if (p[1] == 0x05 && msc_found) {  /* ENDPOINT */
                uint8_t ep_addr = p[2];
                uint8_t ep_attr = p[3];
                uint16_t ep_size = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
                const char *tname =
                    (ep_attr & 0x03) == 0x00 ? "CTRL" :
                    (ep_attr & 0x03) == 0x01 ? "ISOC" :
                    (ep_attr & 0x03) == 0x02 ? "BULK" :
                                              "INTR";
                printf ("USBD:     endpoint 0x%02x %s maxpktsize=%u\r\n",
                        (unsigned)ep_addr, tname, (unsigned)ep_size);
                if ((ep_attr & 0x03) == 0x02) {
                    if (ep_addr & 0x80) in_ep  = ep_addr & 0x0F;
                    else                 out_ep = ep_addr & 0x0F;
                }
            }
            if (p[0] == 0) break;
            p += p[0];
        }
        printf ("USBD:   found %u interface(s), bulk IN=0x%02x OUT=0x%02x\r\n",
                (unsigned)ifcnt, (unsigned)in_ep, (unsigned)out_ep);
    }
    if (in_ep == 0 || out_ep == 0) {
        printf ("USBD: no MSC bulk endpoints found - stick isn't MSC?\r\n");
        return;
    }

    /* Publish the discovered endpoints + device state into the globals
     * so that the SCSI/FAT layers use the right bulk pipe.  Without
     * this the globals stay 0 and every CBW goes to endpoint 0. */
    USB_PublishEndpoints (in_ep, out_ep);
    RootHubDev[port].bEp0MaxPks = ep0_size;
    RootHubDev[port].bSpeed     = speed;
    RootHubDev[port].bAddress   = addr;
    RootHubDev[port].bStatus    = ROOT_DEV_SUCCESS;

    /* Step 8: bulk-only mass storage reset (some sticks need this
     * before the first SCSI command). */
    uint8_t msc_rst = msc_mass_storage_reset ();
    printf ("USBD: step 8 - mass_storage_reset rc=%u (0=ok)\r\n",
            (unsigned)msc_rst);

    /* Step 9: SCSI READ CAPACITY (10) sanity check. */
    printf ("USBD: step 9 - SCSI READ CAPACITY (10):\r\n");
    uint32_t bc = 0, bs = 0;
    for (int attempt = 1; attempt <= 3; attempt++) {
        printf ("USBD:   attempt %d...\r\n", attempt);
        uint8_t r = USB_ScsiReadCapacityOnce (&bc, &bs);
        if (r == 0) {
            printf ("USBD:   OK last_lba=%u (=%u blocks) block_size=%u\r\n",
                    (unsigned)bc, (unsigned)(bc + 1U), (unsigned)bs);
            printf ("USBD: enumeration + READ CAPACITY succeeded!\r\n");
            return;
        }
        printf ("USBD:   attempt %d failed at stage %u\r\n",
                (unsigned)attempt, (unsigned)r);
        uint8_t sense[18] = {0};
        uint16_t slen = sizeof (sense);
        if (usb_scsi_request_sense (sense, slen) == 0) {
            dump_request_sense (sense, (uint8_t)slen);
        } else {
            printf ("USBD:   REQUEST SENSE also failed\r\n");
        }
        Delay_Ms (100);
    }

    /* Step 10: READ CAPACITY(16) for >2TiB sticks. */
    printf ("USBD: step 10 - SCSI READ CAPACITY (16):\r\n");
    {
        uint8_t r16 = USB_ScsiReadCapacity16Once (&bc, &bs);
        if (r16 == 0) {
            printf ("USBD:   OK last_lba=%u block_size=%u\r\n",
                    (unsigned)bc, (unsigned)bs);
            return;
        }
        printf ("USBD:   CAP16 also failed (rc=%u)\r\n", (unsigned)r16);
    }
    printf ("USBD: full diagnostic finished - stick not usable.\r\n");
}

/* ------------------------------------------------------------------------ */
/* Test 2: low-level SCSI read test (UREAD)                                  */
/* ------------------------------------------------------------------------ */

void USB_Tests_Scsi (void) {
    printf ("UREAD: low-level USB drive test\r\n");

    /* Step 1: the enumerate test must have run first (it publishes the
     * endpoints + ROOT_DEV_SUCCESS). */
    if (RootHubDev[DEF_USB_PORT].bStatus != ROOT_DEV_SUCCESS) {
        printf ("UREAD: ERR device not enumerated (run USBD first)\r\n");
        return;
    }
    uint8_t sp = RootHubDev[DEF_USB_PORT].bSpeed;
    const char *spname =
        (sp == USB_HIGH_SPEED) ? "HIGH-SPEED" :
        (sp == USB_FULL_SPEED) ? "FULL-SPEED" :
        (sp == USB_LOW_SPEED)  ? "LOW-SPEED"  : "?";
    printf ("UREAD: enumerated at %s (in_ep=0x%02x out_ep=0x%02x)\r\n",
            spname, (unsigned)usb_in_ep, (unsigned)usb_out_ep);

    /* Step 2: INQUIRY. */
    uint8_t inq[36] = {0};
    uint8_t ri = usb_scsi_inquiry (inq, sizeof(inq));
    if (ri != 0) {
        printf ("UREAD: ERR INQUIRY failed (rc=%u) - try `USBD`\r\n",
                (unsigned)ri);
        return;
    }
    uint8_t pqual = (inq[0] >> 5) & 0x07;
    uint8_t dtype = inq[0] & 0x1F;
    uint8_t rmb   = (inq[1] >> 7) & 1;
    printf ("UREAD: INQUIRY OK\r\n");
    printf ("  peripheral qualifier = %u (0=connected)\r\n", (unsigned)pqual);
    printf ("  device type          = %u "
            "(0=disk, 5=CD-ROM, 7=optical, 8=LB tape, 0x1F=no device)\r\n",
            (unsigned)dtype);
    printf ("  removable            = %u\r\n", (unsigned)rmb);
    printf ("  version              = 0x%02x\r\n", (unsigned)inq[2]);
    char vendor[9]   = {0}; memcpy (vendor,   &inq[8],  8);
    char product[17] = {0}; memcpy (product, &inq[16], 16);
    char rev[5]      = {0}; memcpy (rev,      &inq[32], 4);
    for (int i = 7; i >= 0 && vendor[i] == ' '; i--) vendor[i] = 0;
    for (int i = 15; i >= 0 && product[i] == ' '; i--) product[i] = 0;
    for (int i = 3; i >= 0 && rev[i] == ' '; i--) rev[i] = 0;
    printf ("  vendor   = '%s'\r\n", vendor);
    printf ("  product  = '%s'\r\n", product);
    printf ("  revision = '%s'\r\n", rev);

    /* Step 3: TEST UNIT READY. */
    uint8_t rt = usb_scsi_test_unit_ready ();
    if (rt != 0) {
        uint8_t sense[18] = {0};
        (void)usb_scsi_request_sense (sense, sizeof(sense));
        printf ("UREAD: WARN TEST UNIT READY failed (rc=%u); "
                "sense key=0x%02x asc=0x%02x ascq=0x%02x\r\n",
                (unsigned)rt, (unsigned)sense[2],
                (unsigned)sense[12], (unsigned)sense[13]);
        /* Continue anyway - many sticks return UNIT ATTENTION on the
         * first TEST UNIT READY after INQUIRY, and READ CAPACITY still
         * works. */
    } else {
        printf ("UREAD: TEST UNIT READY OK\r\n");
    }

    /* Step 4: READ CAPACITY(10). */
    uint32_t bc = 0, bs = 0;
    uint8_t rc = usb_scsi_read_capacity (&bc, &bs);
    if (rc != 0) {
        printf ("UREAD: ERR READ CAPACITY failed (rc=%u)\r\n", (unsigned)rc);
        return;
    }
    uint64_t total_bytes = (uint64_t)(bc + 1U) * (uint64_t)bs;
    printf ("UREAD: READ CAPACITY OK\r\n");
    printf ("  last LBA        = %lu (0x%lx)\r\n",
            (unsigned long)bc, (unsigned long)bc);
    printf ("  block size      = %lu bytes\r\n", (unsigned long)bs);
    printf ("  total blocks    = %lu\r\n", (unsigned long)(bc + 1U));
    printf ("  total capacity  = %llu bytes (%llu MiB)\r\n",
            (unsigned long long)total_bytes,
            (unsigned long long)(total_bytes / (1024ULL * 1024ULL)));

    /* Step 5: read LBA 0 (MBR). */
    uint8_t sector[512];
    uint8_t rs = usb_scsi_read_sector (0, sector, bs);
    if (rs != 0) {
        printf ("UREAD: ERR READ(10) LBA 0 failed (rc=%u)\r\n",
                (unsigned)rs);
        return;
    }
    printf ("UREAD: READ(10) LBA 0 OK (%lu bytes)\r\n", (unsigned long)bs);
    uread_hexdump (sector, 256);

    if (sector[510] == 0x55 && sector[511] == 0xAA) {
        printf ("UREAD: MBR signature 0x55 0xAA present "
                "(valid partition table)\r\n");
        for (int pe = 0; pe < 4; pe++) {
            const uint8_t *p = &sector[446 + pe * 16];
            uint8_t status = p[0];
            uint8_t type   = p[4];
            uint32_t lba_start =
                ((uint32_t)p[8])  | ((uint32_t)p[9]  << 8) |
                ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
            uint32_t lba_count =
                ((uint32_t)p[12]) | ((uint32_t)p[13] << 8) |
                ((uint32_t)p[14] << 16) | ((uint32_t)p[15] << 24);
            if (type == 0) continue;
            const char *fname =
                (type == 0x07) ? "NTFS/exFAT" :
                (type == 0x0B) ? "FAT32 CHS"   :
                (type == 0x0C) ? "FAT32 LBA"   :
                (type == 0x0E) ? "FAT16 LBA"   :
                (type == 0x83) ? "Linux ext"   :
                "other";
            printf ("    Part %d: status=0x%02x type=0x%02x (%s) "
                    "LBA %lu..%lu (%lu blocks)\r\n",
                    pe, (unsigned)status, (unsigned)type, fname,
                    (unsigned long)lba_start,
                    (unsigned long)(lba_start + lba_count - 1),
                    (unsigned long)lba_count);
        }
    } else {
        printf ("UREAD: no MBR signature at [510:512] = 0x%02x 0x%02x "
                "(superfloppy / GPT / unpartitioned)\r\n",
                (unsigned)sector[510], (unsigned)sector[511]);
    }

    /* Step 6: spot reads at 4 LBAs across the volume. */
    if (bc >= 1) {
        uint32_t samples[4] = {
            1U,
            (bc + 3U) / 4U,
            (bc + 1U) / 2U,
            (3U * bc + 3U) / 4U
        };
        printf ("UREAD: spot-reading 4 sectors across the volume...\r\n");
        for (int i = 0; i < 4; i++) {
            uint32_t lba = samples[i];
            if (lba > bc) lba = bc;
            uint8_t rso = usb_scsi_read_sector (lba, sector, bs);
            if (rso != 0) {
                printf ("  LBA %lu: ERR (rc=%u)\r\n",
                        (unsigned long)lba, (unsigned)rso);
            } else {
                printf ("  LBA %lu: OK, first 16 bytes =",
                        (unsigned long)lba);
                for (int j = 0; j < 16; j++) {
                    printf (" %02x", sector[j]);
                }
                printf ("\r\n");
            }
        }
    }

    /* Step 7: contiguous read throughput (64 sectors). */
    printf ("UREAD: throughput test (64 sectors = %lu KiB)...\r\n",
            (unsigned long)((64U * bs) / 1024U));
    {
        /* Free-running SysTick snapshot for timing (HCLK cycle
         * counter), then a 64-sector contiguous read. */
        SysTick->SR   = 0;
        SysTick->CNT  = 0;
        SysTick->CMP  = 0xFFFFFFFFU;
        SysTick->CTLR = 0x1DU;   /* STE|STCLK(HCLK)|up-count, no reload */

        uint32_t t0 = SysTick->CNT;
        uint8_t ok = 1;
        for (int s = 0; s < 64; s++) {
            if (usb_scsi_read_sector ((uint32_t)s, sector, bs) != 0) {
                ok = 0;
                break;
            }
        }
        uint32_t t1 = SysTick->CNT;
        SysTick->CTLR = 0;       /* stop the counter */

        if (ok) {
            uint32_t cycles = t1 - t0;
            /* bytes/s = 64*bs * HCLK / cycles; guard div-by-zero. */
            if (cycles > 0) {
                uint64_t bytes_per_s = ((uint64_t)64U * bs
                                       * (uint64_t)SystemCoreClock)
                                      / (uint64_t)cycles;
                printf ("UREAD: 64 sectors in %lu HCLK cycles "
                        "(~%llu bytes/s incl. per-sector overhead)\r\n",
                        (unsigned long)cycles,
                        (unsigned long long)bytes_per_s);
            }
        } else {
            printf ("UREAD: throughput read FAILED mid-way\r\n");
        }
    }
    printf ("UREAD: done\r\n");
}

/* ------------------------------------------------------------------------ */
/* Test 3: FAT integration test (FAT)                                        */
/* ------------------------------------------------------------------------ */

uint8_t USB_Tests_Fat (void) {
    FRESULT fr;

    printf ("\r\nFAT: --- FAT integration test ---\r\n");

    /* Step 1: mount (opt=1 = immediate; runs disk_initialize). */
    fr = f_mount (&g_fatfs, "0:", 1);
    if (fr != FR_OK) {
        printf ("FAT: f_mount failed (%u)\r\n", (unsigned)fr);
        return 1;
    }
    printf ("FAT: mounted 0:\r\n");

    /* Step 2: list the root directory. */
    printf ("FAT: root directory:\r\n");
    {
        DIR dir;
        FILINFO fi;
        uint16_t n = 0;
        fr = f_opendir (&dir, "0:/");
        if (fr != FR_OK) {
            printf ("FAT: f_opendir failed (%u)\r\n", (unsigned)fr);
            return 2;
        }
        while (n < 20) {
            fr = f_readdir (&dir, &fi);
            if (fr != FR_OK || fi.fname[0] == 0) break;
            const char *tag = (fi.fattrib & AM_DIR) ? "d " : "f ";
            printf ("FAT:   %s %10u  %s\r\n", tag,
                    (unsigned)fi.fsize, fi.fname);
            n++;
        }
        f_closedir (&dir);
        printf ("FAT: %u entries listed\r\n", (unsigned)n);
    }

    /* The exact content written in step 3 and verified in step 4.
     * One shared constant so write + verify can't drift. */
    static const char msg[] =
        "RISKYMSX2 USB write test.\r\n"
        "Created by the CH32V407 USBHS host stack.\r\n"
        "If you can read this file, SCSI WRITE(10) works.\r\n";
    const char *test_path = "0:/RISKYMSX2.TXT";

    /* Step 3: create a text file. */
    {
        FIL fp;
        fr = f_open (&fp, test_path, FA_WRITE | FA_CREATE_ALWAYS);
        if (fr != FR_OK) {
            printf ("FAT: f_open('%s', CREATE) failed (%u)\r\n",
                    test_path, (unsigned)fr);
            return 3;
        }
        UINT bw = 0;
        fr = f_write (&fp, msg, sizeof (msg) - 1, &bw);
        if (fr != FR_OK || bw != (sizeof (msg) - 1)) {
            printf ("FAT: f_write failed (fr=%u bw=%u/%u)\r\n",
                    (unsigned)fr, (unsigned)bw,
                    (unsigned)(sizeof (msg) - 1));
            f_close (&fp);
            return 4;
        }
        fr = f_close (&fp);
        if (fr != FR_OK) {
            printf ("FAT: f_close failed (%u)\r\n", (unsigned)fr);
            return 5;
        }
        printf ("FAT: wrote %u bytes to %s\r\n",
                (unsigned)bw, test_path);
    }

    /* Step 4: read it back and verify the FULL content. */
    {
        FIL fp;
        fr = f_open (&fp, test_path, FA_READ);
        if (fr != FR_OK) {
            printf ("FAT: f_open('%s', READ) failed (%u)\r\n",
                    test_path, (unsigned)fr);
            return 6;
        }
        char rb[128] = {0};
        UINT br = 0;
        fr = f_read (&fp, rb, sizeof (rb) - 1, &br);
        if (fr != FR_OK) {
            printf ("FAT: f_read failed (%u)\r\n", (unsigned)fr);
            f_close (&fp);
            return 7;
        }
        rb[br] = 0;
        f_close (&fp);
        printf ("FAT: read back %u bytes:\r\n", (unsigned)br);
        /* Print each line. */
        char *line = rb;
        while ((size_t)(line - rb) < br) {
            char *nl = line;
            while (*nl && *nl != '\r' && *nl != '\n') nl++;
            printf ("FAT:   | %.*s\r\n", (int)(nl - line), line);
            line = (*nl == '\r') ? nl + 2 : nl + 1;
            if (line - rb > (int)br) break;
        }
        /* Byte-for-byte verify (including the trailing CRLF - the
         * earlier tail-only compare was off by exactly that). */
        if (br != sizeof (msg) - 1
            || memcmp (rb, msg, br) != 0) {
            printf ("FAT: VERIFY FAILED - content mismatch "
                    "(read %u, expected %u)\r\n",
                    (unsigned)br, (unsigned)(sizeof (msg) - 1));
            return 8;
        }
        printf ("FAT: verify OK\r\n");
    }

    /* Step 5: hexdump the first 256 bytes of the first regular file
     * in / (proves the read path on a pre-existing file). */
    {
        DIR dir;
        FILINFO fi;
        char first[64] = {0};
        fr = f_opendir (&dir, "0:/");
        if (fr == FR_OK) {
            while (1) {
                fr = f_readdir (&dir, &fi);
                if (fr != FR_OK || fi.fname[0] == 0) break;
                if (!(fi.fattrib & AM_DIR) && fi.fsize > 0) {
                    if (strlen (fi.fname) < sizeof (first) - 4) {
                        snprintf (first, sizeof (first), "0:/%s",
                                  fi.fname);
                        break;
                    }
                }
            }
            f_closedir (&dir);
        }
        if (first[0] != 0) {
            printf ("FAT: reading first 256 bytes of %s:\r\n", first);
            FIL fp;
            fr = f_open (&fp, first, FA_READ);
            if (fr == FR_OK) {
                uint8_t hdr[256];
                UINT br = 0;
                fr = f_read (&fp, hdr, sizeof (hdr), &br);
                if (fr == FR_OK) {
                    for (UINT off = 0; off < br; off += 16) {
                        printf ("FAT:   %04x:", (unsigned)off);
                        UINT row = (br - off > 16) ? 16 : (br - off);
                        for (UINT j = 0; j < row; j++) {
                            printf (" %02x", hdr[off + j]);
                        }
                        printf ("\r\n");
                    }
                }
                f_close (&fp);
            } else {
                printf ("FAT: f_open('%s') failed (%u)\r\n",
                        first, (unsigned)fr);
            }
        }
    }

    printf ("FAT: --- FAT integration test PASSED ---\r\n");
    return 0;
}

/* ------------------------------------------------------------------------ */
/* Test 4: full suite with cold-start retries                                */
/* ------------------------------------------------------------------------ */

void USB_Tests_RunAll (int attempts) {
    /* At cold start (power-on reset) the USB flash drive and the USBHS
     * PHY both need extra time to stabilise:
     *   - The USB stick's internal controller needs 2-3 s to boot its
     *     flash translation layer from zero power.
     *   - The UTMI PHY calibration converges more slowly when the chip
     *     just powered up.
     * After a hot (pin) reset, both are already warm and 1 s is plenty.
     * Use 3 s before the first attempt, then retry with re-init. */
    printf ("\r\n=== USB drive test starting in 3 s "
            "(cold-start settle) ===\r\n");
    Delay_Ms (3000);

    for (int usb_attempt = 1; usb_attempt <= attempts; usb_attempt++) {
        printf ("\r\n--- USB attempt %d/%d ---\r\n",
                usb_attempt, attempts);

        /* Re-init the USBHS controller on retries to reset the SIE/PHY
         * state - a failed enumeration can leave the SIE in a bad
         * state that prevents the next attempt from working. */
        if (usb_attempt > 1) {
            USB_Initialization ();
            Delay_Ms (500);
        }

        printf ("\r\n--- USBD (verbose enumeration) ---\r\n");
        USB_Tests_Enumerate ();

        if (RootHubDev[DEF_USB_PORT].bStatus != ROOT_DEV_SUCCESS) {
            printf ("\r\nUSB enumeration failed on attempt %d; "
                    "retrying...\r\n", usb_attempt);
            Delay_Ms (1000);
            continue;
        }

        printf ("\r\n--- UREAD (low-level SCSI read test) ---\r\n");
        USB_Tests_Scsi ();

        /* FAT integration test: mount + list + create a text file
         * + read it back + verify.  Exercises the WRITE(10) path. */
        USB_Tests_Fat ();
        break;
    }

    printf ("\r\n=== USB drive test complete ===\r\n");
}