/********************************** (C) COPYRIGHT  *******************************
 * File Name          : usb_disk.c
 * Description        : USBHS host enumeration + SCSI for the
 *                      RISKYMSX2 firmware.  Ported from
 *                      v303Firmware/User/FATFS/usb_disk.c, which used
 *                      the CH32V303 USBFS host SIE.
 *
 *                      Migration notes vs the v303 version:
 *                        - USBFSH_* calls -> USBHSH_*
 *                        - data-toggle is uint16_t (USBHS keeps 2 bits,
 *                          the lower one is what USBFS uses; the second
 *                          bit is the high-bandwidth ping-pong toggle
 *                          that gets cleared by HALT-clear anyway, so
 *                          we still treat it as a single byte of toggle)
 *                        - DEF_USB_PORT_FS (0) -> DEF_USB_PORT (1).
 *                          Only one root hub (USBHS1).
 *                        - USBFSH_SetSelfSpeed() removed.  USBHS
 *                          reports its own speed in
 *                          USBHSH_EnableRootHubPort's *pspeed out-param.
 *                        - USBFSH_ResetRootHubPort(0) -> USBHSH_ResetRootHubPort()
 *                          (no mode arg in the USBHS variant).
 *                        - USBFSH_PortDetect() removed; USBHS uses
 *                          USBHSH_CheckRootHubPortEnable().
 *                        - bulk max packet 64 -> 512 (USBHS high-speed
 *                          bulk endpoint size).
 *
 *                      All buffer space (descriptor scratch + sector
 *                      buffer + the actual file content) lives in PSRAM
 *                      (0x80000000+) so we don't burn any of the 135 KB
 *                      internal SRAM.  PSRAM DMA is used to land the
 *                      final file copy.
 *********************************************************************************/

#include "usb_disk.h"
#include "cart.h"
#include "psram.h"
#include "ch32v4x7.h"
#include "ch32v4x7_psram.h"
#include <string.h>

/* ------------------------------------------------------------------------ */
/* Buffer placement (PSRAM)                                                 */
/* ------------------------------------------------------------------------ */

/* PSRAM layout (8 MB total @ 0x80000000):
 *
 *   0x80000000 - 0x80000020  : DevDesc_Buf     18 bytes
 *   0x80000020 - 0x8000011F  : Com_Buffer      255 bytes
 *   0x80001000 - 0x800FF000  : psram sector buffer for SCSI read (1 MiB,
 *                              enough for a 512-byte sector + slack)
 *
 * NB: the cart image window PSRAM_CART_BASE is at the START of PSRAM (set
 * by cart.h).  The buffers above overlap the cart image window. That's
 * OK for the buffers themselves, but it means while a USB read is in
 * progress the cart window contents are being overwritten - the cart
 * mapper is left at NONE during USB operations (see main.c).
 */

/* Force 4-byte alignment; the WCH USBHS host SIE requires the
 * descriptor setup-data pointer to be 4-byte aligned. */
__attribute__ ((aligned (4))) uint8_t Com_Buffer[DEF_COM_BUF_LEN];
__attribute__ ((aligned (4))) uint8_t DevDesc_Buf[18];

struct _ROOT_HUB_DEVICE RootHubDev[DEF_TOTAL_ROOT_HUB];
struct __HOST_CTL        HostCtl[DEF_TOTAL_ROOT_HUB * DEF_ONE_USB_SUP_DEV_TOTAL];

/* The single FATFS volume handle used by the firmware (mounted once
 * USBH_PreDeal reports success).  Declared here so the disk glue +
 * the loader share the same instance. */
FATFS g_fatfs;

/* The PSRAM-based buffers must be exposed via their PSRAM addresses too
 * (the on-core USBHS SIE reads/writes via the system bus; both internal
 * SRAM and PSRAM are reachable). For the descriptor-stage transfers the
 * WCH driver uses the small RxBuffer/TxBuffer (64 B each) defined inside
 * ch32v407_usbhs_host.c, so Com_Buffer/DevDesc_Buf just need to be
 * reachable - internal SRAM addresses are fine. */

/* Bulk IN/OUT endpoint addresses; set during enumeration. */
uint8_t  usb_out_ep;          /* OUT endpoint address                       */
uint16_t out_tog;             /* OUT endpoint toggle (USBHS: 16-bit field)   */
uint8_t  usb_in_ep;           /* IN endpoint address                         */
uint16_t in_tog;              /* IN endpoint toggle                          */
static uint8_t msc_interface_number = 0;

/* ------------------------------------------------------------------------ */
/* Class-specific control-request helper (over EP0)                          */
/* ------------------------------------------------------------------------ */

static uint8_t msc_class_request (uint8_t bmRequestType, uint8_t bRequest,
                                  uint16_t wValue, uint16_t wIndex,
                                  uint8_t *buf, uint16_t *plen) {
    /* USBHSH setup-data lives in TxBuffer (the WCH driver doesn't expose
     * a pUSBHS_SetupRequest pointer the way the v303 driver did, so we
     * build the setup packet manually and call CtrlTransfer with NULL
     * pbuf to do the SETUP stage only - then a follow-up GetEndpData to
     * pull the data stage).
     *
     * Actually, the v303 path was: write setup into TxBuffer, call
     * USBFSH_CtrlTransfer(ep0, buf, plen).  The WCH v407 driver exposes
     * the same function with the same signature, so it Just Works. */
    pUSBHS_SetupRequest->bRequestType = bmRequestType;
    pUSBHS_SetupRequest->bRequest      = bRequest;
    pUSBHS_SetupRequest->wValue        = wValue;
    pUSBHS_SetupRequest->wIndex        = wIndex;
    pUSBHS_SetupRequest->wLength       = (plen != NULL && buf != NULL) ? *plen : 0;
    uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
    return USBHSH_CtrlTransfer (ep0, buf, plen);
}

/* Mass Storage: Bulk-Only Mass Storage Reset (bRequest=0xFF, bmRequestType=00100001b) */
uint8_t msc_mass_storage_reset (void) {
    uint8_t rc = msc_class_request (0x21, 0xFF, 0x0000, msc_interface_number,
                                    NULL, NULL);
    if (rc == ERR_SUCCESS) {
        uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
        (void)USBHSH_ClearEndpStall (ep0, (uint8_t)(0x80 | usb_in_ep));
        Delay_Ms (2);
        (void)USBHSH_ClearEndpStall (ep0, usb_out_ep);
        in_tog = 0;
        out_tog = 0;
        Delay_Ms (20);
        return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------------ */
/* CBW / CSW structs                                                        */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint32_t dCBWSignature;
    uint32_t dCBWTag;
    uint32_t dCBWDataTransferLength;
    uint8_t  bmCBWFlags;
    uint8_t  bCBWLUN;
    uint8_t  bCBWCBLength;
    uint8_t  CBWCB[16];
} __attribute__ ((packed)) CBW_t;

typedef struct {
    uint32_t dCSWSignature;
    uint32_t dCSWTag;
    uint32_t dCSWDataResidue;
    uint8_t  bCSWStatus;
} __attribute__ ((packed)) CSW_t;

/* ------------------------------------------------------------------------ */
/* Bulk transport helpers (USBHS)                                            */
/* ------------------------------------------------------------------------ */

static uint8_t usb_send_cbw (const CBW_t *cbw) {
    uint16_t plen = sizeof (CBW_t);
    uint8_t  res;
    for (int tries = 0; tries < 40; tries++) {
        res = USBHSH_SendEndpData (usb_out_ep, &out_tog,
                                   (uint8_t *)cbw, plen);
        if (res == ERR_SUCCESS) return res;
        if ((tries % 10) == 9) {
            uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
            (void)USBHSH_ClearEndpStall (ep0, usb_out_ep);
            out_tog = 0;
            Delay_Ms (2);
        }
        Delay_Ms (1);
    }
    return res;
}

static uint8_t usb_recv_csw (CSW_t *csw) {
    uint16_t plen = sizeof (CSW_t);
    uint8_t  res;
    for (int tries = 0; tries < 60; tries++) {
        res = USBHSH_GetEndpData (usb_in_ep, &in_tog,
                                  (uint8_t *)csw, &plen);
        if (res == ERR_SUCCESS) return res;
        if ((tries % 10) == 9) {
            uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
            (void)USBHSH_ClearEndpStall (ep0, (uint8_t)(0x80 | usb_in_ep));
            in_tog = 0;
            Delay_Ms (2);
        }
        Delay_Ms (1);
    }
    return res;
}

static uint8_t bulk_in_read_retry (uint8_t *buf, uint16_t *plen,
                                   int max_retries) {
    /* The CH32V407 USBHS controller reports ERR_SUCCESS with RX_LEN=0
     * when a bulk-IN handshake completes but no data was returned --
     * i.e. the stick's ZLP response to an IN while it is still
     * processing the previous OUT (CBW). On most sticks this lasts
     * 100-1000 us; retrying transparently without touching the data
     * toggle is the right thing.
     *
     * We retry up to `max_retries` times on plen=0, with a tiny delay
     * between attempts. If we exhaust retries AND the controller
     * reported an error, return that. If the controller always said
     * ERR_SUCCESS but we never got data, the caller can decide. */
    uint8_t res;
    int tries = 0;
    int zlp_retries = 0;
    do {
        res = USBHSH_GetEndpData (usb_in_ep, &in_tog, buf, &plen[0]);
        if (res == ERR_SUCCESS && plen[0] > 0) return res;
        /* ZLP-style empty response: stick is busy. Don't reset toggles
         * (the toggle bit is now set on DATA1, the stick expects DATA0
         * next). Just back off and try again. Cap ZLP retries
         * independently of hard-error retries. */
        if (res == ERR_SUCCESS && plen[0] == 0) {
            if (++zlp_retries >= 20) {
                /* Give up -- tell caller we got no data. */
                return res;
            }
            Delay_Us (100);
            continue;
        }
        if ((tries % 10) == 9) {
            uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
            (void)USBHSH_ClearEndpStall (ep0, (uint8_t)(0x80 | usb_in_ep));
            in_tog = 0;
            Delay_Ms (2);
        }
        Delay_Ms (1);
        tries++;
    } while (tries < max_retries);
    return res;
}

/* ------------------------------------------------------------------------ */
/* USB controller init                                                      */
/* ------------------------------------------------------------------------ */

void USB_Initialization (void) {
    /* USBHS driver handles its own RCC init via USBHS_Host_Init(ENABLE)
     * (which calls USBHS_RCC_Init internally). */
    USBHS_Host_Init (ENABLE);
    memset (&RootHubDev[DEF_USB_PORT].bStatus, 0, sizeof (struct _ROOT_HUB_DEVICE));
    memset (&HostCtl[DEF_USB_PORT].InterfaceNum, 0, sizeof (struct __HOST_CTL));
}

/* ------------------------------------------------------------------------ */
/* Root-device enumeration                                                  */
/* ------------------------------------------------------------------------ */

uint8_t USBH_EnumRootDevice (uint8_t usb_port) {
    uint8_t  s;
    uint8_t  enum_cnt;
    uint8_t  cfg_val;
    uint16_t i;
    uint16_t len;
    uint8_t  msc_interface_found = 0;
    uint8_t  msc_in_ep_found     = 0;
    uint8_t  msc_out_ep_found    = 0;

    enum_cnt = 0;
ENUM_START:
    Delay_Ms (100);
    enum_cnt++;
    Delay_Ms (8 << enum_cnt);

    /* USBHS has no "begin/end reset" mode arg; it just resets. */
    USBHSH_ResetRootHubPort ();
    for (i = 0, s = 0; i < DEF_RE_ATTACH_TIMEOUT; i++) {
        if (USBHSH_EnableRootHubPort (&RootHubDev[usb_port].bSpeed)
                == ERR_SUCCESS) {
            i = 0;
            s++;
            if (s > 6) break;
        }
        Delay_Ms (1);
    }
    if (i) {
        if (enum_cnt <= 5) goto ENUM_START;
        return ERR_USB_DISCON;
    }

    /* USBHS already populated RootHubDev[usb_port].bSpeed via the call
     * above; no separate SetSelfSpeed call needed (unlike v303 USBFS). */

    s = USBHSH_GetDeviceDescr (&RootHubDev[usb_port].bEp0MaxPks, DevDesc_Buf);
    if (s != ERR_SUCCESS) {
        if (enum_cnt <= 5) goto ENUM_START;
        return DEF_DEV_DESCR_GETFAIL;
    }

    RootHubDev[usb_port].bAddress = (uint8_t)(DEF_USB_PORT + USB_DEVICE_ADDR);
    s = USBHSH_SetUsbAddress (RootHubDev[usb_port].bEp0MaxPks,
                              RootHubDev[usb_port].bAddress);
    if (s != ERR_SUCCESS) {
        if (enum_cnt <= 5) goto ENUM_START;
        return DEF_DEV_ADDR_SETFAIL;
    }
    Delay_Ms (5);

    s = USBHSH_GetConfigDescr (RootHubDev[usb_port].bEp0MaxPks,
                               Com_Buffer, DEF_COM_BUF_LEN, &len);
    if (s != ERR_SUCCESS) {
        if (enum_cnt <= 5) goto ENUM_START;
        return DEF_CFG_DESCR_GETFAIL;
    }
    cfg_val = ((PUSB_CFG_DESCR)Com_Buffer)->bConfigurationValue;

    s = USBHSH_SetUsbConfig (RootHubDev[usb_port].bEp0MaxPks, cfg_val);
    if (s != ERR_SUCCESS) {
        if (enum_cnt <= 5) goto ENUM_START;
        return ERR_USB_UNSUPPORT;
    }

    /* Parse configuration descriptor for MSC interface + endpoints */
    {
        uint8_t *p   = Com_Buffer;
        uint8_t *end = Com_Buffer + len;
        uint8_t in_ep_found = 0, out_ep_found = 0;
        msc_interface_found = 0;
        uint8_t interface_class = 0, interface_subclass = 0, interface_protocol = 0;

        while (p < end) {
            if (p[1] == 0x04) {  /* INTERFACE descriptor */
                interface_class    = p[5];
                interface_subclass = p[6];
                interface_protocol = p[7];
                if (interface_class == 0x08 && interface_subclass == 0x06
                    && interface_protocol == 0x50) {
                    msc_interface_found = 1;
                    msc_interface_number = p[2];
                }
            }
            if (p[1] == 0x05 && msc_interface_found) {  /* ENDPOINT descriptor */
                uint8_t ep_addr = p[2];
                uint8_t ep_attr = p[3];
                if ((ep_attr & 0x03) == 0x02) {  /* BULK */
                    if (ep_addr & 0x80) {
                        usb_in_ep = ep_addr & 0x0F;
                        in_ep_found = 1;
                    } else {
                        usb_out_ep = ep_addr & 0x0F;
                        out_ep_found = 1;
                    }
                }
            }
            if (p[0] == 0) break;
            p += p[0];
        }

        if (msc_interface_found && in_ep_found && out_ep_found) {
            /* Publish + reset toggles in one place (shared with the
             * verbose enumeration diagnostic in usb_tests.c). */
            USB_PublishEndpoints (usb_in_ep, usb_out_ep);
        } else {
            usb_in_ep = 0;
            usb_out_ep = 0;
            return ERR_USB_UNSUPPORT;
        }
    }

    return ERR_SUCCESS;
}

uint8_t USBH_PreDeal (void) {
    uint8_t usb_port = DEF_USB_PORT;
    uint8_t index;
    uint8_t ret;

    /* For USBHS we ask "is the port enabled?" (driver clears it on
     * disconnect) and "is a device attached?". */
    ret = USBHSH_CheckRootHubPortStatus (RootHubDev[usb_port].bStatus);

    if (ret == ROOT_DEV_CONNECTED) {
        if (RootHubDev[usb_port].bStatus != ROOT_DEV_SUCCESS) {
            RootHubDev[usb_port].bStatus      = ROOT_DEV_CONNECTED;
            RootHubDev[usb_port].bDeviceIndex = usb_port * DEF_ONE_USB_SUP_DEV_TOTAL;

            ret = USBH_EnumRootDevice (usb_port);
            if (ret == ERR_SUCCESS) {
                RootHubDev[usb_port].bStatus = ROOT_DEV_SUCCESS;
                return DEF_SUCCESS;
            } else {
                RootHubDev[usb_port].bStatus = ROOT_DEV_FAILED;
                return DEF_ERR_ENUM;
            }
        }
        return DEF_SUCCESS;
    } else if (ret == ROOT_DEV_DISCONNECT) {
        index = RootHubDev[usb_port].bDeviceIndex;
        memset (&RootHubDev[usb_port].bStatus, 0, sizeof (struct _ROOT_HUB_DEVICE));
        memset (&HostCtl[index].InterfaceNum, 0, sizeof (struct __HOST_CTL));
        return DEF_ERR_DETECT;
    }

    return DEF_DEFAULT;
}

void ClearUSB (void) {
    uint8_t usb_port = DEF_USB_PORT;
    uint8_t index;
    index = RootHubDev[usb_port].bDeviceIndex;
    memset (&RootHubDev[usb_port].bStatus, 0, sizeof (struct _ROOT_HUB_DEVICE));
    memset (&HostCtl[index].InterfaceNum, 0, sizeof (struct __HOST_CTL));
}

/* ------------------------------------------------------------------------ */
/* SCSI commands (READ CAPACITY, READ, REQUEST SENSE)                        */
/* ------------------------------------------------------------------------ */

static uint8_t scsi_read_capacity10_once (uint32_t *block_count,
                                         uint32_t *block_size) {
    CBW_t cbw;
    CSW_t csw;
    uint8_t  cap_buf[8];
    uint8_t  res;
    uint16_t plen;

    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature        = 0x43425355;
    cbw.dCBWTag              = 0x11223344;
    cbw.dCBWDataTransferLength = 8;
    cbw.bmCBWFlags           = 0x80;  /* IN */
    cbw.bCBWLUN              = 0;
    cbw.bCBWCBLength         = 10;
    cbw.CBWCB[0]             = 0x25;  /* READ CAPACITY(10) */

    res = usb_send_cbw (&cbw);
    if (res != ERR_SUCCESS) {
        printf ("USB: CAP10 CBW send failed r=%02x\r\n", (unsigned)res);
        return 1;
    }

    Delay_Ms (2);
    plen = 8;
    res = bulk_in_read_retry (cap_buf, &plen, 40);
    if (res != ERR_SUCCESS) {
        printf ("USB: CAP10 data stage failed r=%02x plen=%u\r\n",
                (unsigned)res, (unsigned)plen);
        return 2;
    }
    if (plen != 8) {
        printf ("USB: CAP10 short read plen=%u (expected 8)\r\n",
                (unsigned)plen);
        return 2;
    }
    Delay_Ms (2);
    res = usb_recv_csw (&csw);
    if (res != ERR_SUCCESS) {
        printf ("USB: CAP10 CSW recv failed r=%02x\r\n", (unsigned)res);
        return 3;
    }
    if (csw.bCSWStatus != 0) {
        printf ("USB: CAP10 CSW status=%u residue=%u sig=%08x\r\n",
                (unsigned)csw.bCSWStatus,
                (unsigned)csw.dCSWDataResidue,
                (unsigned)csw.dCSWSignature);
        return 3;
    }
    Delay_Ms (1);
    *block_count = ((uint32_t)cap_buf[0] << 24) | ((uint32_t)cap_buf[1] << 16)
                 | ((uint32_t)cap_buf[2] << 8)  |  (uint32_t)cap_buf[3];
    *block_size  = ((uint32_t)cap_buf[4] << 24) | ((uint32_t)cap_buf[5] << 16)
                 | ((uint32_t)cap_buf[6] << 8)  |  (uint32_t)cap_buf[7];
    return 0;
}

static uint8_t scsi_read_capacity16_once (uint32_t *block_count,
                                          uint32_t *block_size) {
    CBW_t cbw; CSW_t csw; uint8_t res; uint16_t plen = 32; uint8_t cap_buf[32];
    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature        = 0x43425355;
    cbw.dCBWTag              = 0x11223366;
    cbw.dCBWDataTransferLength = 32;
    cbw.bmCBWFlags           = 0x80;
    cbw.bCBWLUN              = 0;
    cbw.bCBWCBLength         = 16;
    cbw.CBWCB[0]  = 0x9E;  /* READ CAPACITY(16) */
    cbw.CBWCB[1]  = 0x10;  /* service action */
    cbw.CBWCB[13] = 0x20;  /* alloc length 32 */
    res = usb_send_cbw (&cbw);
    if (res != ERR_SUCCESS) return 1;
    Delay_Ms (1);
    res = bulk_in_read_retry (cap_buf, &plen, 40);
    if (res != ERR_SUCCESS || plen < 12) return 2;
    Delay_Ms (1);
    res = usb_recv_csw (&csw);
    if (res != ERR_SUCCESS || csw.bCSWStatus != 0) return 3;
    {
        uint32_t lba_hi = ((uint32_t)cap_buf[0] << 24)
                        | ((uint32_t)cap_buf[1] << 16)
                        | ((uint32_t)cap_buf[2] << 8)
                        |  (uint32_t)cap_buf[3];
        uint32_t lba_lo = ((uint32_t)cap_buf[4] << 24)
                        | ((uint32_t)cap_buf[5] << 16)
                        | ((uint32_t)cap_buf[6] << 8)
                        |  (uint32_t)cap_buf[7];
        *block_count = lba_hi ? 0xFFFFFFFFU : lba_lo;
        *block_size  = ((uint32_t)cap_buf[8] << 24)
                     | ((uint32_t)cap_buf[9] << 16)
                     | ((uint32_t)cap_buf[10] << 8)
                     |  (uint32_t)cap_buf[11];
    }
    return 0;
}

/* Single-shot READ CAPACITY(10). Public wrapper for the enumeration
 * diagnostic in usb_tests.c (the retrying production version is
 * usb_scsi_read_capacity below). */
uint8_t USB_ScsiReadCapacityOnce (uint32_t *block_count,
                                  uint32_t *block_size) {
    return scsi_read_capacity10_once (block_count, block_size);
}

/* Single-shot READ CAPACITY(16) for >2 TiB sticks. Public wrapper for
 * the enumeration diagnostic. */
uint8_t USB_ScsiReadCapacity16Once (uint32_t *block_count,
                                    uint32_t *block_size) {
    return scsi_read_capacity16_once (block_count, block_size);
}

/* Publish the discovered bulk endpoint pair into the globals and reset
 * the data toggles. Called by USBH_EnumRootDevice and by the verbose
 * enumeration diagnostic in usb_tests.c once the MSC interface has
 * been parsed. */
void USB_PublishEndpoints (uint8_t in_ep, uint8_t out_ep) {
    usb_in_ep  = in_ep;
    usb_out_ep = out_ep;
    in_tog     = 0;
    out_tog    = 0;
}

uint8_t usb_scsi_read_capacity (uint32_t *block_count, uint32_t *block_size) {
    /* Try READ CAPACITY(10) several times before giving up.  Many
     * sticks need a moment after enumeration to spin up their flash
     * translation layer; the first attempt often fails with a phase
     * error or short packet.  A bare REQUEST SENSE + retry (NO MSC
     * reset) usually works; the bulk-only mass-storage reset from the
     * v303 path actually makes things worse on a freshly-enumerated
     * stick because it drops the just-negotiated bulk pipe state. */
    for (int attempt = 0; attempt < 5; attempt++) {
        uint8_t r = scsi_read_capacity10_once (block_count, block_size);
        if (r == 0) return 0;
        uint8_t sense[18] = {0};
        (void)usb_scsi_request_sense (sense, sizeof(sense));
        Delay_Ms (50);     /* let the stick finish settling */
    }
    /* Last-resort: 16-byte READ CAPACITY for >2 TiB sticks (we won't
     * actually address that much, but some sticks refuse CAP10 and only
     * answer CAP16). */
    uint8_t r16 = scsi_read_capacity16_once (block_count, block_size);
    if (r16 == 0) return 0;
    return 3;   /* still failing */
}

/* Read a single 512-byte (or whatever the stick's block size is) sector
 * into `buf`.  For high-speed sticks the bulk pipe is a 512-byte
 * multi-packet stream; we loop on the per-packet size (512) until
 * `block_size` bytes have been received. */
static uint8_t scsi_read_sector_once (uint32_t lba, uint8_t *buf,
                                      uint32_t block_size) {
    CBW_t cbw;
    CSW_t csw;
    uint8_t  res;
    uint16_t plen;
    uint32_t bytes_received = 0;
    uint32_t transfer_len   = block_size;

    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature          = 0x43425355;
    cbw.dCBWTag                = 0xCAFEBABE;
    cbw.dCBWDataTransferLength = transfer_len;
    cbw.bmCBWFlags             = 0x80;  /* IN */
    cbw.bCBWLUN                = 0;
    cbw.bCBWCBLength           = 10;
    cbw.CBWCB[0] = 0x28;  /* READ(10) */
    cbw.CBWCB[2] = (lba >> 24) & 0xFF;
    cbw.CBWCB[3] = (lba >> 16) & 0xFF;
    cbw.CBWCB[4] = (lba >> 8)  & 0xFF;
    cbw.CBWCB[5] = (lba)       & 0xFF;
    cbw.CBWCB[7] = 0;          /* 1 block */
    cbw.CBWCB[8] = 1;

    int cbw_send_retries = 0;
    do {
        res = usb_send_cbw (&cbw);
        if (res == ERR_SUCCESS) break;
        Delay_Ms (1);
        cbw_send_retries++;
    } while (cbw_send_retries < 20);
    if (res != ERR_SUCCESS) return 1;

    while (bytes_received < block_size) {
        plen = block_size - bytes_received;
        if (plen > USBHS_BULK_MAX_PACKET) plen = USBHS_BULK_MAX_PACKET;
        int nak_retries = 0;
        do {
            res = USBHSH_GetEndpData (usb_in_ep, &in_tog,
                                      buf + bytes_received, &plen);
            if (res != ERR_SUCCESS) {
                Delay_Ms (1);
                nak_retries++;
                if (nak_retries >= 20) return 2;
            }
        } while (res != ERR_SUCCESS);
        if (res != ERR_SUCCESS || plen == 0) return 2;
        bytes_received += plen;
    }

    int csw_retries = 0;
    do {
        res = usb_recv_csw (&csw);
        if (res == ERR_SUCCESS && csw.bCSWStatus == 0) break;
        Delay_Ms (1);
        csw_retries++;
    } while (csw_retries < 40);
    if (res != ERR_SUCCESS || csw.bCSWStatus != 0) return 3;
    return 0;
}

uint8_t usb_scsi_read_sector (uint32_t lba, uint8_t *buf, uint32_t block_size) {
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t r = scsi_read_sector_once (lba, buf, block_size);
        if (r == 0) return 0;
        uint8_t sense[18] = {0};
        (void)usb_scsi_request_sense (sense, sizeof(sense));
        (void)msc_mass_storage_reset ();
        Delay_Ms (5);
    }
    return 3;
}

/* Write a single block with SCSI WRITE(10).  Mirrors scsi_read_sector_once:
 * CBW (OUT, data-out) -> DATA-OUT packets on the bulk OUT endpoint ->
 * CSW (IN).  The data stage sends `block_size` bytes in 512-byte
 * (USBHS_BULK_MAX_PACKET) chunks. */
static uint8_t scsi_write_sector_once (uint32_t lba, const uint8_t *buf,
                                       uint32_t block_size) {
    CBW_t cbw;
    CSW_t csw;
    uint8_t  res;
    uint32_t bytes_sent = 0;

    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature          = 0x43425355;
    cbw.dCBWTag                = 0xCAFEBA5E;
    cbw.dCBWDataTransferLength = block_size;
    cbw.bmCBWFlags             = 0x00;  /* OUT - host to device */
    cbw.bCBWLUN                = 0;
    cbw.bCBWCBLength           = 10;
    cbw.CBWCB[0] = 0x2A;  /* WRITE(10) */
    cbw.CBWCB[2] = (lba >> 24) & 0xFF;
    cbw.CBWCB[3] = (lba >> 16) & 0xFF;
    cbw.CBWCB[4] = (lba >> 8)  & 0xFF;
    cbw.CBWCB[5] = (lba)       & 0xFF;
    cbw.CBWCB[7] = 0;          /* 1 block (MSB) */
    cbw.CBWCB[8] = 1;          /* 1 block */

    res = usb_send_cbw (&cbw);
    if (res != ERR_SUCCESS) {
        printf ("USB: WR10 CBW send failed r=%02x\r\n", (unsigned)res);
        return 1;
    }

    /* Data stage: stream the block out on the bulk OUT endpoint in
     * 512-byte chunks. */
    while (bytes_sent < block_size) {
        uint16_t chunk = (uint16_t)((block_size - bytes_sent)
                                    > USBHS_BULK_MAX_PACKET)
                       ? (uint16_t)USBHS_BULK_MAX_PACKET
                       : (uint16_t)(block_size - bytes_sent);
        int out_retries = 0;
        do {
            res = USBHSH_SendEndpData (usb_out_ep, &out_tog,
                                       buf + bytes_sent, chunk);
            if (res == ERR_SUCCESS) break;
            Delay_Ms (1);
            if (++out_retries >= 20) return 2;
        } while (res != ERR_SUCCESS);
        bytes_sent += chunk;
    }

    Delay_Ms (1);
    res = usb_recv_csw (&csw);
    if (res != ERR_SUCCESS || csw.bCSWStatus != 0) {
        printf ("USB: WR10 CSW rc=%02x status=%u\r\n",
                (unsigned)res, (unsigned)csw.bCSWStatus);
        return 3;
    }
    return 0;
}

uint8_t usb_scsi_write_sector (uint32_t lba, const uint8_t *buf,
                               uint32_t block_size) {
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t r = scsi_write_sector_once (lba, buf, block_size);
        if (r == 0) return 0;
        uint8_t sense[18] = {0};
        (void)usb_scsi_request_sense (sense, sizeof(sense));
        (void)msc_mass_storage_reset ();
        Delay_Ms (5);
    }
    return 3;
}

uint8_t usb_scsi_request_sense (uint8_t *buf, uint16_t len) {
    CBW_t cbw; CSW_t csw; uint8_t res; uint16_t plen = len;
    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature        = 0x43425355;
    cbw.dCBWTag              = 0x53454E53;
    cbw.dCBWDataTransferLength = len;
    cbw.bmCBWFlags           = 0x80;
    cbw.bCBWLUN              = 0;
    cbw.bCBWCBLength         = 6;
    cbw.CBWCB[0] = 0x03;
    cbw.CBWCB[4] = (uint8_t)len;
    res = usb_send_cbw (&cbw);
    if (res != ERR_SUCCESS) return 1;
    Delay_Ms (1);
    res = bulk_in_read_retry (buf, &plen, 40);
    if (res != ERR_SUCCESS) return 2;
    Delay_Ms (1);
    res = usb_recv_csw (&csw);
    if (res != ERR_SUCCESS || csw.bCSWStatus != 0) return 3;
    return 0;
}

/* SCSI INQUIRY (opcode 0x12). Reads the standard 36-byte response into
 * `buf`. Used by the UREAD CLI test to confirm the device speaks SCSI
 * and to print the vendor/product/revision strings without going
 * through FATFS. `len` should be at least 36. */
uint8_t usb_scsi_inquiry (uint8_t *buf, uint16_t len) {
    CBW_t cbw; CSW_t csw; uint8_t res; uint16_t plen = len;
    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature          = 0x43425355;
    cbw.dCBWTag                = 0xCAFEBAB0;
    cbw.dCBWDataTransferLength = len;
    cbw.bmCBWFlags             = 0x80;   /* IN */
    cbw.bCBWLUN                = 0;
    cbw.bCBWCBLength           = 6;
    cbw.CBWCB[0] = 0x12;                  /* INQUIRY */
    cbw.CBWCB[4] = (uint8_t)len;
    res = usb_send_cbw (&cbw);
    if (res != ERR_SUCCESS) return 1;
    Delay_Ms (1);
    res = bulk_in_read_retry (buf, &plen, 40);
    if (res != ERR_SUCCESS) return 2;
    Delay_Ms (1);
    res = usb_recv_csw (&csw);
    if (res != ERR_SUCCESS || csw.bCSWStatus != 0) return 3;
    return 0;
}

/* SCSI TEST UNIT READY (opcode 0x00). No data phase. Returns 0 on
 * success, non-zero on failure (CSW status, request-sense may then
 * tell us why). */
uint8_t usb_scsi_test_unit_ready (void) {
    CBW_t cbw; CSW_t csw; uint8_t res;
    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature          = 0x43425355;
    cbw.dCBWTag                = 0xCAFEBAB1;
    cbw.dCBWDataTransferLength = 0;
    cbw.bmCBWFlags             = 0x80;   /* IN but zero-length */
    cbw.bCBWLUN                = 0;
    cbw.bCBWCBLength           = 6;
    cbw.CBWCB[0] = 0x00;                  /* TEST UNIT READY */
    res = usb_send_cbw (&cbw);
    if (res != ERR_SUCCESS) return 1;
    Delay_Ms (1);
    res = usb_recv_csw (&csw);
    if (res != ERR_SUCCESS) return 2;
    return (csw.bCSWStatus == 0) ? 0 : 3;
}

/* ------------------------------------------------------------------------ */
/* PSRAM DMA helper (one-shot copy from SRAM scratch to PSRAM)              */
/* ------------------------------------------------------------------------ */

/* The WCH PSRAM DMA controller expects a 32-bit-aligned source buffer
 * (must be in SRAM, NOT in PSRAM - the PSRAM controller reads from the
 * system bus).  `sram_src` points at a 4-byte-aligned SRAM region of
 * `byte_count` bytes (must be a multiple of 4). */
static uint8_t psram_dma_copy (const uint8_t *sram_src, uint32_t psram_addr,
                               uint32_t byte_count) {
    if ((byte_count & 3U) != 0U) return 1;        /* must be 4-byte aligned */
    if (((uint32_t)sram_src & 3U) != 0U) return 1;
    if (byte_count == 0U) return 0;

    PSRAMDMATypeDef dmas = {0};
    dmas.PSRAM_DMA_DAT_DIR      = DMA_DIR_PSRAM;   /* SRAM -> PSRAM */
    dmas.PSRAM_DMA_MEMORY_SIZE  = PSRAM_DMA_SIZE_32;
    dmas.PSRAM_DMA_DMA_ONEBL    = DMA_BRST_NUM8;
    dmas.PSRAM_DMA_TOWBL_TOUT   = DMA_PAUSE_TIM4;
    dmas.PSRAM_DMA_DATA_NUM     = byte_count / 4U;

    PSRAMDMASet ((uint32_t *)sram_src, psram_addr, &dmas);
    PSRAM_DMA_Cmd (ENABLE);
    while ((PSRAM->ISR & 0x2U) == 0U) {            /* wait DMATF */
        /* could WFI here but the DMA finishes fast (1 MiB / 8-word burst) */
    }
    PSRAM_ClearITPendingBit (PSRAM_DMATF);
    PSRAM_DMA_Cmd (DISABLE);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* High-level: file -> PSRAM                                                */
/* ------------------------------------------------------------------------ */

/* Sector-sized SRAM scratch buffer used as the bulk-IN target before
 * PSRAM DMA.  One sector (typically 512 B); aligned to 4.  Internal
 * SRAM is only ~135 KB but a single 512-byte buffer is fine. */
__attribute__ ((aligned (4))) static uint8_t s_sector_buf[512];

/* Set by USB_TryEnsureMounted() once f_mount() has succeeded.  Cleared
 * on every disconnect (so the next file op re-mounts).  Keeps the
 * "am I mounted?" check out of every f_opendir / f_open. */
static uint8_t s_fatfs_mounted = 0U;

/* Attempt to (re-)enumerate the USB device and mount the FAT volume.
 * Returns 0 on success, non-zero (DEF_ERR_*) on failure.
 *
 * Use this from any CLI/file op BEFORE calling f_open / f_opendir so a
 * missed `USB` step (or a stick unplug/replug) doesn't silently fail
 * with FR_NO_FILESYSTEM (12).  Idempotent: cheap to call repeatedly. */
uint8_t USB_TryEnsureMounted (void) {
    /* 1. Poll the root hub.  Up to 5 attempts: freshly plugged sticks
     *    often need 200-500 ms to finish their post-enumeration
     *    housekeeping (especially flash translation layer init).
     *    USBH_PreDeal() itself does multiple enumeration retries
     *    internally, so this loop is mostly for the f_mount side. */
    uint8_t r = DEF_ERR_ENUM;
    for (int tries = 0; tries < 5; tries++) {
        r = USBH_PreDeal ();
        if (r == DEF_ERR_DETECT) {
            if (s_fatfs_mounted) {
                (void)f_mount (NULL, "0:", 0);
                s_fatfs_mounted = 0U;
                printf ("USB: stick removed, volume unmounted\r\n");
            }
            return DEF_ERR_DETECT;
        }
        if (r == DEF_SUCCESS || r == DEF_DEFAULT) break;
        Delay_Ms (100);
    }
    if (r == DEF_ERR_ENUM) {
        return DEF_ERR_ENUM;
    }
    if (r != DEF_SUCCESS && r != DEF_DEFAULT) {
        return r;
    }

    /* 2. If enumeration just succeeded (or we were already enumerated),
     *    make sure f_mount is in sync.  Idempotent: f_mount() on an
     *    already-mounted volume is fine.  Retry f_mount a few times
     *    for the same settling reason - disk_initialize -> SCSI READ
     *    CAPACITY often fails on the first try. */
    if (!s_fatfs_mounted) {
        for (int tries = 0; tries < 3; tries++) {
            FRESULT fr = f_mount (&g_fatfs, "0:", 1);
            if (fr == FR_OK) {
                s_fatfs_mounted = 1U;
                printf ("FAT: mounted on 0:\r\n");
                return DEF_SUCCESS;
            }
            /* Translate common FR_* into something the user can act on.
             * FR_NOT_READY (3) = disk_initialize failed, typically
             * because the SCSI READ CAPACITY (10) returned a non-512
             * block size or failed entirely. */
            const char *why =
                (fr ==  3) ? "no medium / SCSI READ CAPACITY failed"
                : (fr == 11) ? "FAT volume not recognised (try FAT12/16/32 stick)"
                : (fr == 12) ? "no FAT filesystem on stick"
                :               "see FatFs docs";
            printf ("FAT: f_mount failed (%u = %s)\r\n", (unsigned)fr, why);
            if (tries < 2) {
                Delay_Ms (200);
                /* disk_initialize caches the block size; force a reset
                 * so the next f_mount re-runs it (and gives the stick
                 * another chance to settle before the next SCSI burst). */
                ClearUSB ();
                (void)USBH_PreDeal ();
            }
        }
        return DEF_ERR_ENUM;
    }
    return DEF_SUCCESS;
}

/* Read `len` bytes of `path` (a FatFS path like "0:/GAMES/ROM1.ROM")
 * and write them into PSRAM at `psram_addr`.  `psram_addr` is the
 * ABSOLUTE PSRAM bus address (e.g. PSRAM_CART_BASE 0x80000000, or
 * PSRAM_CART_BASE+offset for a sub-region); the bounds check below
 * subtracts the bus base to compare against the window-relative size.
 * If `len` is 0, the file size is used.  Returns the number of bytes
 * actually copied, or 0 on error. */
uint32_t USB_FileToPSRAM (const char *path, uint32_t psram_addr,
                          uint32_t len) {
    /* Lazy-mount: if the user skipped `USB` (or replugged after a
     // disconnect), get the stick enumerated and the volume mounted
     // before we try to open the file. */
    if (USB_TryEnsureMounted () != DEF_SUCCESS) return 0U;

    FIL     fp;
    FRESULT fr;
    UINT    br;
    uint32_t total = 0U;

    fr = f_open (&fp, path, FA_READ);
    if (fr != FR_OK) {
        printf ("USB: f_open('%s') failed (%u)\r\n", path, (unsigned)fr);
        return 0U;
    }

    /* Bounds check: psram_addr is the absolute PSRAM bus address
     * (always >= PSRAM_CART_BASE 0x80000000). Compare the offset
     * within the cart window against PSRAM_CART_SIZE. The previous
     * version compared the absolute address against PSRAM_CART_SIZE
     * directly, which always tripped (0x80000000 >= 0x00800000) and
     * made the loader's "copy 0 bytes -> PSRAM" silent. */
    if (psram_addr < PSRAM_CART_BASE
        || psram_addr - PSRAM_CART_BASE >= PSRAM_CART_SIZE) {
        f_close (&fp);
        printf ("USB: PSRAM address 0x%08lx out of range\r\n",
                (unsigned long)psram_addr);
        return 0U;
    }

    if (len == 0U) {
        len = (uint32_t)f_size (&fp);
    }

    uint32_t remaining = PSRAM_CART_SIZE - (psram_addr - PSRAM_CART_BASE);
    if (len > remaining) {
        len = remaining;
    }

    while (total < len) {
        uint32_t chunk = len - total;
        if (chunk > sizeof (s_sector_buf)) chunk = sizeof (s_sector_buf);

        fr = f_read (&fp, s_sector_buf, chunk, &br);
        if (fr != FR_OK || br == 0U) break;
        if (psram_dma_copy (s_sector_buf, psram_addr + total, (uint32_t)br) != 0U) {
            printf ("USB: PSRAM DMA copy failed at offset %u\r\n",
                    (unsigned)total);
            break;
        }
        total += (uint32_t)br;
        if (br < chunk) break;     /* EOF */
    }

    f_close (&fp);
    return total;
}

/* ------------------------------------------------------------------------ */
/* CLI helper: directory listing (LS command)                                */
/* ------------------------------------------------------------------------ */

void USB_ListDir (const char *path, uint16_t max) {
    /* Lazy-mount: see USB_FileToPSRAM for the rationale. */
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        printf ("LS: no USB stick (run `USB` after plugging one in)\r\n");
        return;
    }

    DIR     dir;
    FRESULT fr = f_opendir (&dir, path);
    if (fr != FR_OK) {
        printf ("LS: f_opendir('%s') failed (%u)\r\n",
                path, (unsigned)fr);
        return;
    }
    FILINFO fi;
    uint16_t n = 0;
    printf ("LS %s:\r\n", path);
    while (n < max) {
        fr = f_readdir (&dir, &fi);
        if (fr != FR_OK || fi.fname[0] == 0) break;
        const char *tag = (fi.fattrib & AM_DIR) ? "d " : "f ";
        printf ("  %s %10u  %s\r\n", tag,
                (unsigned)fi.fsize, fi.fname);
        n++;
    }
    f_closedir (&dir);
}
