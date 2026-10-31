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
#include "debug.h"
#include "cart.h"
#include "psram.h"
#include "nextor.h"
#include "ch32v4x7.h"
#include "ch32v4x7_psram.h"
#include <string.h>

#ifndef RISKY_DEBUG
#define RISKY_DEBUG 0
#endif

/* PSRAM load path selector:
 *   0 = PSRAM peripheral DMA burst engine (the default, faster)
 *   1 = synchronous CPU word copy (fallback)
 *
 * The PSRAM DMA burst engine is the preferred path: it frees the CPU
 * for other work during the burst and the PSRAM controller's DMA
 * hardware has higher throughput than the CPU's per-word stores at
 * the 400 MHz DDR device clock. The CPU path is kept as a fallback
 * for silicon revisions or environmental factors (board temperature,
 * PSRAM device lot) where the DMA engine misbehaves.
 *
 * The DMA engine's known fragility (DMATF flag not asserting under
 * some conditions at HCLK 200 MHz / 400 MHz DDR device clock) is
 * handled inside psram_dma_copy() by:
 *   - a bounded completion wait (~10 ms) per attempt,
 *   - a controller reset (PSRAMDeInit) between attempts,
 *   - up to PSRAM_DMA_RETRIES retries before failing,
 *   - a first/last word verify readback after each successful
 *     transfer to detect silent corruption that would otherwise pass
 *     DMATF but land wrong bytes in PSRAM.
 *
 * The CPU path is ~13 us per 512 B chunk (~210 ms for an 8 MiB image)
 * and keeps the same per-chunk ~WAIT hold structure. Set to 1 to
 * fall back to it. */
#ifndef USB_PSRAM_SYNC_COPY
#define USB_PSRAM_SYNC_COPY 0
#endif

/* Maximum number of retry attempts per chunk before failing the
 * whole copy. Each retry resets the PSRAM DMA controller via
 * PSRAMDeInit() so the next attempt starts from a clean state.
 * Three is enough in practice: the wedge is a one-shot per chunk
 * and on this hardware the second attempt has not been observed to
 * fail when the first succeeded. */
#ifndef PSRAM_DMA_RETRIES
#define PSRAM_DMA_RETRIES  3
#endif

/* ------------------------------------------------------------------------ */
/* Buffer placement (PSRAM)                                                 */
/* ------------------------------------------------------------------------ */

/* PSRAM layout (8 MB total @ 0x80000000, see cart.h for the full
 * partition):
 *
 *   0x80000000 + 0                 .. +CART_GAME_MAX_BYTES
 *                                          game image window (default 4 MiB)
 *   0x80000000 + CART_GAME_BASE    .. +CART_GAME_BASE + 0
 *                                          (== PSRAM_CART_BASE)
 *                                          loaded game image (variable,
 *                                          <= CART_GAME_MAX_BYTES)
 *   0x80000000 + CART_SLOTTED_BANK_BASE_OFFSET
 *                                .. +PSRAM_CART_SIZE
 *                                          sub-slot 1 mapper RAM
 *                                          (CART_SLOTTED_BANK_COUNT x
 *                                          CART_SLOTTED_BANK_SIZE = 4 MiB)
 *
 *   0x80000000 - 0x80000020  : DevDesc_Buf     18 bytes  (overlap warning)
 *   0x80000020 - 0x8000011F  : Com_Buffer      255 bytes  (overlap warning)
 *
 * NB: DevDesc_Buf and Com_Buffer live in the cart-image window and
 * corrupt the loaded image if read at the wrong moment. They are
 * only touched during USB enumeration (the very first few SCSI
 * exchanges after a stick is plugged), which never overlaps a ROM
 * load - the SCSI read path itself writes into the SRAM scratch
 * buffer (s_sector_buf) and not into PSRAM, so this is safe in
 * practice. The 0x80001000+ 1 MiB "psram sector buffer" mentioned
 * in earlier revisions is stale: scsi_read_sector_once targets the
 * caller's buf pointer, which is always SRAM for FatFS. */

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
    printf ("USB: MSC reset (control transfer)\r\n");
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

/* USBHSH_Transact folds the received token PID into its error code, and
 * the two interesting codes need OPPOSITE recovery:
 *   0x2A = ERR_USB_TRANSFER|USB_PID_NAK   "busy, no data yet" -> be patient
 *   0x2E = ERR_USB_TRANSFER|USB_PID_STALL "endpoint halted"    -> clear halt
 * Clearing the halt of a merely-busy endpoint is actively harmful: the
 * extra control transfer disturbs a flash controller that is committing
 * a WRITE, and the clear can resync the bulk data toggles out of step. */
#define MSC_ERR_STALL   ((uint8_t)(ERR_USB_TRANSFER | USB_PID_STALL))

static uint8_t usb_send_cbw (const CBW_t *cbw) {
    uint16_t plen = sizeof (CBW_t);
    uint8_t  res;
    for (int tries = 0; tries < 40; tries++) {
        res = USBHSH_SendEndpData (usb_out_ep, &out_tog,
                                   (uint8_t *)cbw, plen);
        if (res == ERR_SUCCESS) return res;
        if (res == MSC_ERR_STALL) {
            /* A halted OUT endpoint stays halted until CLEAR_FEATURE;
             * retrying into it is futile. Both sides return to DATA0. */
            uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
            (void)USBHSH_ClearEndpStall (ep0, usb_out_ep);
            out_tog = 0;
            Delay_Ms (2);
        } else {
            /* NAK: the device is busy. Leave it alone and retry. */
            Delay_Ms (1);
        }
    }
    return res;
}

static uint8_t usb_recv_csw_wait (CSW_t *csw, uint32_t expected_tag,
                                  uint16_t max_tries, uint16_t *csw_tries) {
    uint8_t  res;
    int zlp_retries = 0;
    for (uint16_t tries = 0U; tries < max_tries; tries++) {
        uint16_t plen = sizeof (CSW_t);
        res = USBHSH_GetEndpData (usb_in_ep, &in_tog,
                                  (uint8_t *)csw, &plen);
        /* The only successful CSW is the 13-byte struct. An ERR_SUCCESS
         * with RX_LEN == 0 is the controller reporting the stick's "still
         * busy, no data" ZLP - GetEndpData deliberately leaves the toggle
         * alone for it (see its comment). That is exactly what a WRITE
         * sees while the stick programs the block just received, and the
         * old "any ERR_SUCCESS is a CSW" rule returned the caller an
         * uninitialised struct while the real CSW stayed queued for the
         * next command. Treat a short/empty success as "not ready", back
         * off, and ask again. */
        if ((res == ERR_SUCCESS) && (plen >= (uint16_t)sizeof (CSW_t))) {
            /* A stale CSW must never be accepted as the result of the
             * current command.  This matters after a timeout/recovery where
             * the device may still have the previous response queued. */
            if (csw->dCSWSignature != 0x53425355U
                || csw->dCSWTag != expected_tag) {
                return ERR_USB_TRANSFER;
            }
            if (csw_tries != (uint16_t *)0) {
                /* One poll + 1 ms back-off per try ~= milliseconds. */
                *csw_tries = tries;
            }
            return res;
        }
        if (res == ERR_SUCCESS) {
            if (++zlp_retries >= 20) return ERR_USB_TRANSFER;
            Delay_Us (100);
            continue;
        }
        if (res == MSC_ERR_STALL) {
            /* The endpoint was really halted (phase error / refused
             * command).  Clear it - both sides return to DATA0 - and
             * keep polling for the CSW. */
            uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
            (void)USBHSH_ClearEndpStall (ep0, (uint8_t)(0x80 | usb_in_ep));
            in_tog = 0;
            Delay_Ms (2);
            continue;
        }
        /* Anything else is a NAK: the device is still busy finishing the
         * command (a flash controller committing a WRITE can NAK for
         * hundreds of milliseconds and cheap ones for seconds).  Do NOT
         * clear-halt a merely-busy endpoint - the control transfer
         * disturbs the commit and can desync the bulk toggles. */
        Delay_Ms (1);
    }
    if (csw_tries != (uint16_t *)0) {
        *csw_tries = max_tries;
    }
    return res;
}

/* Normal commands keep the short wait used by the existing read path.
 * Flash media can take substantially longer to commit a WRITE(10), so the
 * write path opts into the longer wait below without slowing reads. */
static uint8_t usb_recv_csw (CSW_t *csw, uint32_t expected_tag) {
    return usb_recv_csw_wait (csw, expected_tag, 60U, (uint16_t *)0);
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
        if (res == MSC_ERR_STALL) {
            /* Real halt: clear it (both sides return to DATA0), retry. */
            uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
            (void)USBHSH_ClearEndpStall (ep0, (uint8_t)(0x80 | usb_in_ep));
            in_tog = 0;
            Delay_Ms (2);
        } else {
            /* NAK: busy. Leave the endpoint alone. */
            Delay_Ms (1);
        }
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
    /* Yield to the Z80 driver before each enumeration attempt: this loop
     * sits behind hundreds of milliseconds of Delay_Ms backoff, and a
     * mailbox request that arrives mid-probe must not wait for it. The
     * error code means "not enumerated this pass, retry later" to every
     * caller - the probe state machine is fully polled and re-runs. */
    if (Nextor_RequestPending () != 0U) return ERR_USB_DISCON;
    Delay_Ms (100);
    enum_cnt++;
    Delay_Ms (8 << enum_cnt);

    /* USBHS has no "begin/end reset" mode arg; it just resets. */
    USBHSH_ResetRootHubPort ();
    for (i = 0, s = 0; i < DEF_RE_ATTACH_TIMEOUT; i++) {
        if (Nextor_RequestPending () != 0U) return ERR_USB_DISCON;
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

/* Probe staging buffer. The READ CAPACITY(10)/(16) responses are supposed
 * to be 8 and 32 bytes respectively, but a misbehaving stick has been seen
 * returning 12 or 18 bytes to CAP10 - the hardware happily copies the
 * full hardware RX_LEN into the destination the caller passes, and a
 * stack cap_buf[8] would be overrun into the caller's res, plen, and
 * the next bytes of the stack frame (with a small-but-real chance that
 * the clobber makes the function look like a SUCCESS with a bogus
 * capacity). 512 bytes covers any single bulk IN packet and gives the
 * probe a place to land the answer safely; the real cap_buf is then
 * filled with a bounded copy. The buffer is static, not on the stack,
 * precisely so no length a broken stick can send overruns the
 * caller's frame. Do NOT move this to a local without re-checking the
 * length discipline in USBHSH_GetEndpData. */
static uint8_t s_probe_buf[512];

/* Returned by the probe when a misbehaving stick gives a wrong-length
 * CAPACITY(10) answer (12 or 18 bytes instead of 8). Distinct from
 * "transient I/O failure" so the retrying loop in usb_scsi_read_capacity
 * can break out instead of wasting the 5-retry budget on a permanent
 * fault - a stick that gives the wrong length once will give the wrong
 * length every time. */
#define USB_PROBE_LEN_MISMATCH   4U

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
    /* Read into the static staging buffer, not the 8-byte stack cap_buf:
     * a broken stick that returns more than 8 bytes (12 or 18 observed
     * in practice) would otherwise overrun the caller's frame via the
     * uncapped memcpy inside USBHSH_GetEndpData. The copy below is
     * explicitly bounded to the real buffer size. */
    res = bulk_in_read_retry (s_probe_buf, &plen, 40);
    if (res != ERR_SUCCESS) {
        printf ("USB: CAP10 data stage failed r=%02x plen=%u\r\n",
                (unsigned)res, (unsigned)plen);
        return 2;
    }
    if (plen != 8U) {
        printf ("USB: CAP10 short read plen=%u (expected 8)\r\n",
                (unsigned)plen);
        /* A misbehaving stick is giving the wrong length for every
         * CAP10 attempt - do not waste the 5-retry budget, signal a
         * permanent fault to the caller so it can break out and try
         * CAP16 instead. */
        return USB_PROBE_LEN_MISMATCH;
    }
    memcpy (cap_buf, s_probe_buf, 8U);
    Delay_Ms (2);
    res = usb_recv_csw (&csw, cbw.dCBWTag);
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
    /* READ CAPACITY returns the LAST LBA, not a count. Callers (diskio.c's
     * disk_ioctl, raw_disk.c) all want a block COUNT, so the +1 belongs
     * here rather than in four call sites. Forgetting it costs exactly the
     * last sector of the medium: the capacity still looks sane, every read
     * below it works, and a partition whose last cluster happens to end on
     * the final block fails with a read error instead of a clean
     * "out of range". */
    *block_count = ((((uint32_t)cap_buf[0] << 24) | ((uint32_t)cap_buf[1] << 16)
                   | ((uint32_t)cap_buf[2] << 8)  |  (uint32_t)cap_buf[3]) + 1U);
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
    /* Same staging-buffer discipline as the CAP10 path: USBHSH_GetEndpData
     * does not cap the memcpy at the caller's buffer size, so a wrong-length
     * reply (a CAP10 answer, padding, sense data) would overrun the
     * stack 32-byte cap_buf. Read into s_probe_buf, then copy. */
    res = bulk_in_read_retry (s_probe_buf, &plen, 40);
    if (res != ERR_SUCCESS || plen < 12) return 2;
    memcpy (cap_buf, s_probe_buf, (plen > 32U) ? 32U : plen);
    Delay_Ms (1);
    /* A successful OUT only means the block reached the device. Cheap flash
     * sticks may spend hundreds of milliseconds programming it before the
     * CSW is available. Do not reset the BOT pipe while that commit is still
     * in progress. */
    res = usb_recv_csw_wait (&csw, cbw.dCBWTag, 250U, (uint16_t *)0);
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
        /* Same LAST-LBA-to-count conversion as the (10) form above. A
         * device too big for 32 bits cannot be addressed by anything that
         * takes a uint32_t block count, so it saturates rather than
         * wrapping to a small number that would look like a tiny disk. */
        *block_count = lba_hi ? 0xFFFFFFFFU : (lba_lo + 1U);
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
        if (r == USB_PROBE_LEN_MISMATCH) {
            /* The stick answers CAP10 with the wrong length every time,
             * which means its bulk pipe is in a confused state (a CAP10
             * reply should be exactly 8 bytes; 12 or 18 is usually a
             * stuck sense-data or padding reply). The proven fix is a
             * BOT reset + settle delay: the terminal's USB manager,
             * which runs the same probe code but later in boot when the
             * stick has had time to wake up, succeeds for exactly this
             * reason, and what unblocks it is the pipe being reset.
             * Without the reset the CAP16 attempt below usually fails
             * the same way. */
            (void)msc_mass_storage_reset ();
            Delay_Ms (200);
            break;
        }
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
/* DIAGNOSTIC counter for the SCSI read path, re-armed at the end of each
 * USB_FileToPSRAM run. Keep it out of normal builds with the trace it caps. */
#if RISKY_DEBUG
static uint32_t s_scsi_dbg = 0U;
#endif

/* Single 512-byte read from the stick... */
static uint8_t scsi_read_sector_once (uint32_t lba, uint8_t *buf,
                                      uint32_t block_size) {
    CBW_t cbw;
    CSW_t csw;
    uint8_t  res;
    uint16_t plen;
    uint32_t bytes_received = 0;
    uint32_t transfer_len   = block_size;

    /* DIAGNOSTIC: per-sector chatter is useful for tracing a stuck read,
     * but floods the normal log during directory scans and file loads. */
#if RISKY_DEBUG
    if (s_scsi_dbg < 64U) {
        printf ("USB: R lba=%lu\r\n", (unsigned long)lba);
        s_scsi_dbg++;
    }
#endif

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
    if (res != ERR_SUCCESS) {
        printf ("USB: R lba=%lu CBW FAIL r=%02x\r\n",
                (unsigned long)lba, (unsigned)res);
        return 1;
    }

    while (bytes_received < transfer_len) {
        plen = transfer_len - bytes_received;
        if (plen > USBHS_BULK_MAX_PACKET) plen = USBHS_BULK_MAX_PACKET;
        int nak_retries = 0;
        do {
            res = USBHSH_GetEndpData (usb_in_ep, &in_tog,
                                      buf + bytes_received, &plen);
            if (res != ERR_SUCCESS) {
                /* Detailed per-packet retry traces are debug-only. The
                 * terminal NAK LIMIT message below remains in normal logs. */
#if RISKY_DEBUG
                if (s_scsi_dbg < 64U) {
                    printf ("USB: R lba=%lu off=%lu IN r=%02x try=%u\r\n",
                            (unsigned long)lba,
                            (unsigned long)bytes_received,
                            (unsigned)res, (unsigned)nak_retries);
                    s_scsi_dbg++;
                }
#endif
                Delay_Ms (1);
                nak_retries++;
                if (nak_retries >= 20) {
                    printf ("USB: R lba=%lu off=%lu NAK LIMIT\r\n",
                            (unsigned long)lba,
                            (unsigned long)bytes_received);
                    return 2;
                }
            }
        } while (res != ERR_SUCCESS);
        if (res != ERR_SUCCESS || plen == 0) return 2;
        bytes_received += plen;
    }

    /* Reads interleave with WRITE commits on the same stick during a
     * stick-to-stick copy: the read's data flows from the controller's
     * cache, but its CSW NAKs until the commit finishes - measured
     * beyond 2.4 s on the test stick (write commits of 0.2-1.1 s with
     * rarer longer stalls). Same window policy as the write path: the
     * total CSW patience stays inside the Z80 driver's MB_POLL bound
     * (~2.9 s by cycle count), because an answer that arrives after the
     * driver has timed out is a DOS drive error anyway. */
    int csw_retries = 0;
    do {
        res = usb_recv_csw_wait (&csw, cbw.dCBWTag, 100U, (uint16_t *)0);
        if (res == ERR_SUCCESS && csw.bCSWStatus == 0) break;
        Delay_Ms (1);
        csw_retries++;
    } while (csw_retries < 20);
    if (res != ERR_SUCCESS || csw.bCSWStatus != 0) {
        printf ("USB: R lba=%lu CSW FAIL r=%02x st=%u tries=%u\r\n",
                (unsigned long)lba, (unsigned)res,
                (unsigned)csw.bCSWStatus, (unsigned)csw_retries);
        return 3;
    }
    return 0;
}

uint8_t usb_scsi_read_sector (uint32_t lba, uint8_t *buf, uint32_t block_size) {
    return usb_scsi_read_sectors (lba, buf, 1U, block_size);
}

uint8_t usb_scsi_read_sectors (uint32_t lba, uint8_t *buf,
                               uint16_t count, uint32_t block_size) {
    if (count == 0U) return 0U;
    for (int attempt = 0; attempt < 2; attempt++) {
        uint8_t r = scsi_read_sector_once (lba, buf, block_size);
        if (r == 0) return 0;
        printf ("USB: R lba=%lu attempt %u failed r=%u -> sense+reset\r\n",
                (unsigned long)lba, (unsigned)(attempt + 1), (unsigned)r);
        uint8_t sense[18] = {0};
        (void)usb_scsi_request_sense (sense, sizeof(sense));
        {
            uint8_t rr = msc_mass_storage_reset ();
            printf ("USB: W lba=%lu recovery reset rc=%02x\r\n",
                    (unsigned long)lba, (unsigned)rr);
            if (rr != 0U) {
                printf ("USB: W lba=%lu recovery reset failed\r\n",
                        (unsigned long)lba);
            }
        }
        Delay_Ms (5);
    }
    return 3;
}

/* Cheap flash sticks commit a WRITE(10) in batches; while an erase/commit
 * runs, the stick NAKs the CSW. Measured on the current test stick:
 * ~210 ms and ~1075 ms alternating under sustained load, with rare
 * stalls past 4.5 s.
 *
 * The budget is NOT "as long as the stick might take". The Z80 driver
 * polls for the answer with MB_POLL, which gives up after ~2.9 s
 * (3 passes x 65535 x ~53 T-states at 3.58 MHz), and an answer that
 * arrives later than that is a DOS drive error anyway - so waiting
 * longer only wedges the mailbox behind a command nobody is listening
 * to anymore. Policy: ~2 s per attempt; the MSC reset plus the one
 * retry then lands the answer just inside the poll window (the commit
 * that stalled attempt 1 is finished by then, so attempt 2 is fast),
 * and a genuinely stuck stick costs DOS a Retry. */
#define WR10_CSW_MAX_TRIES   1800U

/* Write a single block with SCSI WRITE(10).  Mirrors scsi_read_sector_once:
 * CBW (OUT, data-out) -> DATA-OUT packets on the bulk OUT endpoint ->
 * CSW (IN).  The data stage sends `block_size` bytes in 512-byte
 * (USBHS_BULK_MAX_PACKET) chunks.
 *
 * Returns:
 *   0 - written and acknowledged (CSW status 0)
 *   1 - CBW never reached the device
 *   2 - data stage failed (endpoint stayed halted / device refused)
 *   3 - no CSW within the budget - the write MAY STILL BE COMMITTING
 *       inside the FTL; no more SCSI commands may be overlapped
 *   4 - CSW arrived with CHECK CONDITION (the command really completed,
 *       and REQUEST SENSE is now legal and meaningful) */
static uint8_t scsi_write_sector_once (uint32_t lba, const uint8_t *buf,
                                       uint32_t block_size,
                                       uint16_t csw_max_tries,
                                       uint16_t *csw_tries) {
    CBW_t cbw;
    CSW_t csw = {0};
    uint8_t  res;
    uint32_t bytes_sent = 0;

    /* Keep the write path as observable as the read path.  In particular,
     * WRITE(10) has an OUT data stage, so a failure can happen before the
     * command ever reaches the device (CBW or OUT), or after the device has
     * received the block (CSW). */
#if RISKY_DEBUG
    printf ("USB: W lba=%lu bytes=%lu head=%02x %02x %02x %02x "
            "tail=%02x %02x\r\n",
            (unsigned long)lba, (unsigned long)block_size,
            (unsigned)buf[0], (unsigned)buf[1],
            (unsigned)buf[2], (unsigned)buf[3],
            (unsigned)buf[block_size - 2U],
            (unsigned)buf[block_size - 1U]);
#endif

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
            if (res == MSC_ERR_STALL) {
                /* The device halted the OUT endpoint (phase error or
                 * refused command).  A halted endpoint stays halted
                 * until CLEAR_FEATURE, so retrying into it is futile -
                 * clear the halt, resync both sides to DATA0, retry. */
                uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
                (void)USBHSH_ClearEndpStall (ep0, usb_out_ep);
                out_tog = 0;
                Delay_Ms (2);
            } else {
                Delay_Ms (1);
            }
            if (++out_retries >= 20) break;
        } while (res != ERR_SUCCESS);
        if (res != ERR_SUCCESS) {
            printf ("USB: W lba=%lu off=%lu OUT FAIL r=%02x "
                    "tries=%u\r\n",
                    (unsigned long)lba, (unsigned long)bytes_sent,
                    (unsigned)res, (unsigned)out_retries);
            return 2;
        }
        bytes_sent += chunk;
    }

    /* The data OUT stage only tells us that the block reached the device.
     * Flash media may need hundreds of milliseconds - cheap ones seconds -
     * for programming and wear-levelling before it returns the CSW.  Do
     * not mistake that normal busy interval for a transport failure and
     * reset the BOT pipe. */
    res = usb_recv_csw_wait (&csw, cbw.dCBWTag, csw_max_tries, csw_tries);
    if (res != ERR_SUCCESS) {
        printf ("USB: WR10 CSW rc=%02x after ~%u polls\r\n",
                (unsigned)res, (unsigned)csw_max_tries);
        return 3;
    }
    /* A late CSW is worth reporting: it is the direct measurement of how
     * long this stick's FTL really takes to commit a write, and the number
     * that says whether the wait budget above is generous enough. */
    if (csw_tries != (uint16_t *)0 && *csw_tries > 100U) {
        printf ("USB: W lba=%lu CSW after ~%u ms (FTL commit delay)\r\n",
                (unsigned long)lba, (unsigned)*csw_tries);
    }
    if (csw.bCSWStatus != 0) {
        printf ("USB: WR10 CSW status=%u (CHECK CONDITION)\r\n",
                (unsigned)csw.bCSWStatus);
        return 4;
    }
    return 0;
}

uint8_t usb_scsi_write_sector (uint32_t lba, const uint8_t *buf,
                               uint32_t block_size) {
    for (int attempt = 0; attempt < 2; attempt++) {
        uint16_t csw_tries = 0U;
        uint8_t r = scsi_write_sector_once (lba, buf, block_size,
                                            WR10_CSW_MAX_TRIES,
                                            &csw_tries);
        if (r == 0) return 0;
        printf ("USB: W lba=%lu attempt=%u failed stage=%u\r\n",
                (unsigned long)lba, (unsigned)(attempt + 1),
                (unsigned)r);
        if (r != 3) {
            /* The command completed (CHECK CONDITION) or never started
             * (CBW/OUT stage), so the BOT pipe is idle and REQUEST SENSE
             * is legal.  It is NOT legal after stage 3: the WRITE may
             * still be committing inside the FTL, and overlapping another
             * command is exactly what used to wedge the pipe on this
             * stick (the "REQUEST SENSE failed r=02" lines). */
            uint8_t sense[18] = {0};
            uint8_t sr = usb_scsi_request_sense (sense, sizeof(sense));
            if (sr == 0U) {
                printf ("USB: W lba=%lu sense key=%02x asc=%02x "
                        "ascq=%02x\r\n",
                        (unsigned long)lba,
                        (unsigned)(sense[2] & 0x0FU),
                        (unsigned)sense[12], (unsigned)sense[13]);
            } else {
                printf ("USB: W lba=%lu REQUEST SENSE failed r=%02x\r\n",
                        (unsigned long)lba, (unsigned)sr);
            }
        }
        if (r != 4) {
            /* Stage 4 (CHECK CONDITION) left the pipe clean - the CSW
             * arrived - so the BOT reset would be pointless disruption.
             * Every other stage needs the reset before the retry. */
            (void)msc_mass_storage_reset ();
        }
        Delay_Ms (5);
    }
    printf ("USB: W lba=%lu FAILED after 2 attempts "
            "(see the stage lines above)\r\n",
            (unsigned long)lba);
    return 3;
}

/* Single-attempt WRITE(10) with the measured CSW commit latency, for the
 * format-time usable-capacity scan (see terminal.c).  One attempt, no
 * retry/reset storm: the measurement is the point.  `*commit_ms` is the
 * CSW poll count (one poll ~= 1 ms), i.e. approximately how long the
 * stick's FTL took to commit the block.  Returns 0 on success. */
uint8_t usb_scsi_write_sector_timed (uint32_t lba, const uint8_t *buf,
                                     uint32_t block_size,
                                     uint32_t *commit_ms) {
    uint16_t tries = 0U;
    uint8_t r = scsi_write_sector_once (lba, buf, block_size,
                                        WR10_CSW_MAX_TRIES, &tries);
    if (commit_ms != (uint32_t *)0) {
        *commit_ms = (uint32_t)tries;
    }
    return r;
}

/* One multi-block WRITE(10): a single CBW covering `count` blocks, the
 * whole data stage in USBHS_BULK_MAX_PACKET chunks, one CSW wait. This
 * is the firmware half of the bulk mailbox command: a flash stick
 * charges its commit - measured 210 ms to 1075 ms on the test stick -
 * once per SCSI COMMAND, not per sector, so `count` blocks in one
 * command pay ONE commit instead of `count`.
 *
 * Deliberately ONE attempt with the write budget: the caller is the
 * bulk mailbox command, whose DONE has to land inside the driver's
 * ~2.9 s MB_POLL window; its retry is the driver's per-sector fallback.
 * On a failure that leaves the BOT pipe mid-command (CBW sent but never
 * answered, data stage refused) exactly one MSC reset runs before
 * returning, because the per-sector fallback needs a clean pipe. After
 * a clean CHECK CONDITION the pipe is already idle, so the reset is
 * skipped and the sense data is read instead. Returns 0 on success. */
uint8_t usb_scsi_write_blocks (uint32_t lba, const uint8_t *buf,
                               uint32_t block_size, uint16_t count) {
    CBW_t cbw;
    CSW_t csw = {0};
    uint8_t  res;
    uint32_t total      = (uint32_t)count * block_size;
    uint32_t bytes_sent = 0;
    uint16_t tries      = 0U;

    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature          = 0x43425355;
    cbw.dCBWTag                = 0xCAFEBA5E;
    cbw.dCBWDataTransferLength = total;
    cbw.bmCBWFlags             = 0x00;  /* OUT - host to device */
    cbw.bCBWLUN                = 0;
    cbw.bCBWCBLength           = 10;
    cbw.CBWCB[0] = 0x2A;  /* WRITE(10) */
    cbw.CBWCB[2] = (lba >> 24) & 0xFF;
    cbw.CBWCB[3] = (lba >> 16) & 0xFF;
    cbw.CBWCB[4] = (lba >> 8)  & 0xFF;
    cbw.CBWCB[5] = (lba)       & 0xFF;
    cbw.CBWCB[7] = (uint8_t)(((uint32_t)count >> 8) & 0xFF);
    cbw.CBWCB[8] = (uint8_t)((uint32_t)count & 0xFF);

    res = usb_send_cbw (&cbw);
    if (res != ERR_SUCCESS) {
        printf ("USB: WRM lba=%lu+%u CBW FAIL r=%02x\r\n",
                (unsigned long)lba, (unsigned)count, (unsigned)res);
        return 1;
    }

    /* Data stage: the same chunk/stall-clear loop as the single-block
     * path, just walked to `total` instead of one block. */
    while (bytes_sent < total) {
        uint16_t chunk = (uint16_t)((total - bytes_sent)
                                    > USBHS_BULK_MAX_PACKET)
                       ? (uint16_t)USBHS_BULK_MAX_PACKET
                       : (uint16_t)(total - bytes_sent);
        int out_retries = 0;
        do {
            res = USBHSH_SendEndpData (usb_out_ep, &out_tog,
                                       buf + bytes_sent, chunk);
            if (res == ERR_SUCCESS) break;
            if (res == MSC_ERR_STALL) {
                uint8_t ep0 = RootHubDev[DEF_USB_PORT].bEp0MaxPks;
                (void)USBHSH_ClearEndpStall (ep0, usb_out_ep);
                out_tog = 0;
                Delay_Ms (2);
            } else {
                Delay_Ms (1);
            }
            if (++out_retries >= 20) break;
        } while (res != ERR_SUCCESS);
        if (res != ERR_SUCCESS) {
            printf ("USB: WRM lba=%lu+%u off=%lu OUT FAIL r=%02x "
                    "tries=%u\r\n",
                    (unsigned long)lba, (unsigned)count,
                    (unsigned long)bytes_sent, (unsigned)res,
                    (unsigned)out_retries);
            (void)msc_mass_storage_reset ();
            return 2;
        }
        bytes_sent += chunk;
    }

    res = usb_recv_csw_wait (&csw, cbw.dCBWTag, WR10_CSW_MAX_TRIES,
                             &tries);
    if (res != ERR_SUCCESS) {
        printf ("USB: WRM lba=%lu+%u CSW rc=%02x after ~%u polls\r\n",
                (unsigned long)lba, (unsigned)count, (unsigned)res,
                (unsigned)WR10_CSW_MAX_TRIES);
        (void)msc_mass_storage_reset ();
        return 3;
    }
    if (tries > 100U) {
        printf ("USB: WRM lba=%lu+%u CSW after ~%u ms (FTL commit "
                "delay)\r\n",
                (unsigned long)lba, (unsigned)count, (unsigned)tries);
    }
    if (csw.bCSWStatus != 0) {
        /* Clean CHECK CONDITION: the pipe is idle, so REQUEST SENSE is
         * legal and is the diagnosis; no BOT reset needed. */
        uint8_t sense[18] = {0};
        (void)usb_scsi_request_sense (sense, sizeof(sense));
        printf ("USB: WRM lba=%lu+%u CSW status=%u sense key=%02x "
                "asc=%02x ascq=%02x\r\n",
                (unsigned long)lba, (unsigned)count,
                (unsigned)csw.bCSWStatus,
                (unsigned)(sense[2] & 0x0FU),
                (unsigned)sense[12], (unsigned)sense[13]);
        return 4;
    }
    return 0;
}

/* One multi-block READ(10) into `buf`: single CBW, the data stage in
 * USBHS_BULK_MAX_PACKET chunks, then the CSW wait with the read path's
 * patience (a read can arrive while an orphaned write commit from a
 * timed-out command is still grinding, so the same ~2 s budget as the
 * single-sector read applies). One attempt; on a failure that leaves
 * the pipe mid-command one MSC reset runs so the caller's fallback
 * (per-sector commands) starts clean. Returns 0 on success. */
uint8_t usb_scsi_read_blocks (uint32_t lba, uint8_t *buf,
                              uint32_t block_size, uint16_t count) {
    CBW_t cbw;
    CSW_t csw;
    uint8_t  res;
    uint32_t transfer_len   = (uint32_t)count * block_size;
    uint32_t bytes_received = 0;
    int      cbw_send_retries = 0;
    int      csw_retries = 0;

    memset (&cbw, 0, sizeof (cbw));
    cbw.dCBWSignature          = 0x43425355;
    cbw.dCBWTag               = 0xCAFEBABE;
    cbw.dCBWDataTransferLength = transfer_len;
    cbw.bmCBWFlags            = 0x80;  /* IN */
    cbw.bCBWLUN               = 0;
    cbw.bCBWCBLength          = 10;
    cbw.CBWCB[0] = 0x28;  /* READ(10) */
    cbw.CBWCB[2] = (lba >> 24) & 0xFF;
    cbw.CBWCB[3] = (lba >> 16) & 0xFF;
    cbw.CBWCB[4] = (lba >> 8)  & 0xFF;
    cbw.CBWCB[5] = (lba)       & 0xFF;
    cbw.CBWCB[7] = (uint8_t)(((uint32_t)count >> 8) & 0xFF);
    cbw.CBWCB[8] = (uint8_t)((uint32_t)count & 0xFF);

    do {
        res = usb_send_cbw (&cbw);
        if (res == ERR_SUCCESS) break;
        Delay_Ms (1);
        cbw_send_retries++;
    } while (cbw_send_retries < 20);
    if (res != ERR_SUCCESS) {
        printf ("USB: RDM lba=%lu+%u CBW FAIL r=%02x\r\n",
                (unsigned long)lba, (unsigned)count, (unsigned)res);
        return 1;
    }

    while (bytes_received < transfer_len) {
        uint16_t plen = (uint16_t)((transfer_len - bytes_received)
                                   > USBHS_BULK_MAX_PACKET)
                      ? (uint16_t)USBHS_BULK_MAX_PACKET
                      : (uint16_t)(transfer_len - bytes_received);
        int nak_retries = 0;
        do {
            res = USBHSH_GetEndpData (usb_in_ep, &in_tog,
                                      buf + bytes_received, &plen);
            if (res == ERR_SUCCESS) break;
            Delay_Ms (1);
            if (++nak_retries >= 40) break;
        } while (res != ERR_SUCCESS);
        if (res != ERR_SUCCESS || plen == 0) {
            printf ("USB: RDM lba=%lu+%u off=%lu NAK LIMIT r=%02x\r\n",
                    (unsigned long)lba, (unsigned)count,
                    (unsigned long)bytes_received, (unsigned)res);
            (void)msc_mass_storage_reset ();
            return 2;
        }
        bytes_received += plen;
    }

    do {
        res = usb_recv_csw_wait (&csw, cbw.dCBWTag, 100U,
                                 (uint16_t *)0);
        if (res == ERR_SUCCESS && csw.bCSWStatus == 0) break;
        Delay_Ms (1);
        csw_retries++;
    } while (csw_retries < 20);
    if (res != ERR_SUCCESS || csw.bCSWStatus != 0) {
        printf ("USB: RDM lba=%lu+%u CSW FAIL r=%02x st=%u tries=%u\r\n",
                (unsigned long)lba, (unsigned)count, (unsigned)res,
                (unsigned)csw.bCSWStatus, (unsigned)csw_retries);
        (void)msc_mass_storage_reset ();
        return 3;
    }
    return 0;
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
    res = usb_recv_csw (&csw, cbw.dCBWTag);
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
    res = usb_recv_csw (&csw, cbw.dCBWTag);
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
    res = usb_recv_csw (&csw, cbw.dCBWTag);
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
/* Chunk-print throttle for psram_dma_copy: only the first
 * USB_DMA_DBG_CHUNKS chunks of each load print debug lines, so the
 * console isn't flooded (an 8 MiB file = 16384 chunks). */
#define USB_DMA_DBG_CHUNKS  4U
static uint32_t s_dbg_count = 0U;
/* Chunk-loop top-of-iteration diagnostic counter (re-armed at the end
 * of each USB_FileToPSRAM run; the per-sector counters live above the
 * SCSI layer). */
static uint32_t s_chunk_dbg = 0U;

#if !USB_PSRAM_SYNC_COPY
/* Verify the first and last word of a chunk against the source. Catches
 * silent PSRAM corruption that lands a "DMATF set" state but writes
 * the wrong bytes - the symptom that is impossible to detect from the
 * DMA status alone. Both words are read with volatile loads so the
 * readback is not coalesced or cached. */
static uint8_t psram_dma_verify (const uint8_t *sram_src,
                                 uint32_t psram_addr, uint32_t byte_count) {
    const volatile uint32_t *dst = (const volatile uint32_t *)psram_addr;
    const uint32_t *src = (const uint32_t *)sram_src;
    /* First word. */
    if (dst[0] != src[0]) return 1U;
    /* Last word (byte_count is a multiple of 4 - guaranteed by the
     * caller, which falls through to the sync-copy error return if
     * it isn't). */
    if (dst[byte_count / 4U - 1U] != src[byte_count / 4U - 1U]) return 2U;
    return 0U;
}

/* Run one DMA burst (DMATF wait + verify). Returns 0 on verified
 * success, non-zero on failure. The caller decides whether to reset
 * the controller and retry. */
static uint8_t psram_dma_burst (const uint8_t *sram_src, uint32_t psram_addr,
                                uint32_t byte_count) {
    /* Set up the DMA descriptor for the whole chunk. 32-bit transfers
     * with NUM8 beats per burst (8 x 4 B = 32 B per burst) keeps the
     * controller's internal burst buffer reasonable and matches the
     * "single shot per chunk" model we run here. */
    PSRAMDMATypeDef dmas = {0};
    dmas.PSRAM_DMA_DAT_DIR      = DMA_DIR_PSRAM;
    dmas.PSRAM_DMA_MEMORY_SIZE  = PSRAM_DMA_SIZE_32;
    dmas.PSRAM_DMA_DMA_ONEBL    = DMA_BRST_NUM8;
    dmas.PSRAM_DMA_TOWBL_TOUT   = DMA_PAUSE_TIM4;
    dmas.PSRAM_DMA_DATA_NUM     = byte_count / 4U;

    PSRAMDMASet ((uint32_t *)sram_src, psram_addr, &dmas);
    PSRAM_DMA_Cmd (ENABLE);

    /* Bounded wait: a normal burst completes in microseconds. If the
     * PSRAM controller wedges (silicon / timing / temperature), this
     * spin would otherwise hold the main loop forever with interrupts
     * live. Bail out with an error so the caller can reset+retry. */
    uint32_t guard = 2000000U;                    /* ~10 ms of spins */
    while ((PSRAM->ISR & PSRAM_DMATF) == 0U) {
        if (--guard == 0U) {
            PSRAM_DMA_Cmd (DISABLE);
            PSRAM_ClearITPendingBit (PSRAM_DMATF);
            printf ("USB: PSRAM DMA stuck at 0x%08lx (%lu bytes)\r\n",
                    (unsigned long)psram_addr, (unsigned long)byte_count);
            return 2U;
        }
    }
    PSRAM_ClearITPendingBit (PSRAM_DMATF);
    PSRAM_DMA_Cmd (DISABLE);

    /* Verify. A wedge that completed DMATF but wrote the wrong bytes
     * (controller state desync after the burst) is otherwise invisible
     * to the firmware - the bytes land in PSRAM, the cart reads them
     * back, the game runs with corrupt instructions. Read the first
     * and last word and compare; on mismatch return non-zero so the
     * caller can reset+retry. */
    const uint8_t vrc = psram_dma_verify (sram_src, psram_addr, byte_count);
    if (vrc != 0U) {
        printf ("USB: PSRAM DMA verify fail (rc=%u) at 0x%08lx (%lu bytes)\r\n",
                (unsigned)vrc, (unsigned long)psram_addr,
                (unsigned long)byte_count);
    }
    return vrc;
}

/* SRAM -> PSRAM via the PSRAM DMA burst engine.
 *
 * Hold cart ~WAIT around the burst so the cart EXTI0 handler cannot
 * fire in the middle of a burst and corrupt the PSRAM controller's
 * internal state. The handler early-returns while s_msx_wait_held is
 * set (see cart.c), so the DMA burst has the bus to itself without
 * disabling EXTI0 in the PFIC. The hold is per-chunk so the MSX-side
 * progress bar still updates between chunks.
 *
 * Reliability strategy: up to PSRAM_DMA_RETRIES attempts per chunk,
 * each preceded by a PSRAMDeInit() to clear any sticky state left
 * behind by a wedged controller. Each successful transfer is
 * verified by reading back the first and last word from PSRAM and
 * comparing to the source. On wedge or verify-mismatch we return an
 * error so the caller (USB_FileToPSRAM) aborts the load cleanly
 * with a diagnostic instead of writing a corrupt image. */
static uint8_t psram_dma_copy (const uint8_t *sram_src, uint32_t psram_addr,
                               uint32_t byte_count) {
    if ((byte_count & 3U) != 0U) return 1;        /* must be 4-byte aligned */
    if (((uint32_t)sram_src & 3U) != 0U) return 1;
    if (byte_count == 0U) return 0;

    if (s_dbg_count < USB_DMA_DBG_CHUNKS) {
        printf ("USB:   DMA[%lu] sram=0x%08lx psram=0x%08lx bytes=%lu\r\n",
                (unsigned long)s_dbg_count, (unsigned long)sram_src,
                (unsigned long)psram_addr, (unsigned long)byte_count);
        s_dbg_count++;
    }

    Cart_HoldMSXWait_Begin ();

    uint8_t  rc = 2U;                              /* default: stuck */
    for (uint32_t attempt = 0U; attempt < PSRAM_DMA_RETRIES; attempt++) {
        if (attempt > 0U) {
            /* Reset the PSRAM peripheral between attempts so a sticky
             * controller state from a previous wedge does not poison
             * the next burst. PSRAMDeInit() is a fast RCC reset, no
             * reinitialization needed (the boot path did that). */
            PSRAMDeInit ();
            printf ("USB: PSRAM DMA retry %u at 0x%08lx\r\n",
                    (unsigned)attempt, (unsigned long)psram_addr);
        }
        rc = psram_dma_burst (sram_src, psram_addr, byte_count);
        if (rc == 0U) break;                       /* verified success */
    }

    /* Release ~WAIT. The held Z80 cycle (typically a 0x7FFF FIFO read)
     * now completes; the EXTI0 edge latched at its start fires the
     * handler, which serves the cycle as a fresh one. */
    Cart_HoldMSXWait_End ();
    return rc;
}
#endif /* !USB_PSRAM_SYNC_COPY */

#if USB_PSRAM_SYNC_COPY
/* Synchronous (CPU) SRAM -> PSRAM copy. See USB_PSRAM_SYNC_COPY above for
 * why this replaces the PSRAM DMA burst engine at HCLK 200 MHz.
 *
 * Word (32-bit) stores keep the PSRAM controller's preferred access size.
 * No head/tail handling is needed: the caller always hands us s_sector_buf
 * (4-byte aligned) and a per-chunk length that is a multiple of 4, so both
 * source and destination stay aligned for the whole loop. */
static uint8_t psram_sync_copy (const uint8_t *sram_src, uint32_t psram_addr,
                                uint32_t byte_count) {
    if ((byte_count & 3U) != 0U) return 1;        /* must be 4-byte aligned */
    if (((uint32_t)sram_src & 3U) != 0U) return 1;
    if (byte_count == 0U) return 0;

    if (s_dbg_count < USB_DMA_DBG_CHUNKS) {
        printf ("USB:   SYNC[%lu] sram=0x%08lx psram=0x%08lx bytes=%lu\r\n",
                (unsigned long)s_dbg_count, (unsigned long)sram_src,
                (unsigned long)psram_addr, (unsigned long)byte_count);
        s_dbg_count++;
    }

    /* Same ~WAIT contract as the DMA path: hold the Z80 for the duration of
     * the copy so its FIFO poll loop cannot re-enter the cart handler and
     * interleave PSRAM accesses with us. The handler early-returns while
     * held (see cart.c::s_msx_wait_held). */
    Cart_HoldMSXWait_Begin ();

    volatile uint32_t *dst = (volatile uint32_t *)psram_addr;
    const    uint32_t *src = (const uint32_t *)sram_src;
    uint32_t words = byte_count / 4U;
    for (uint32_t i = 0U; i < words; i++) {
        dst[i] = src[i];
    }
    /* Ensure the final PSRAM store has retired before ~WAIT is released and
     * the Z80 reads the byte back. */
    __asm__ volatile ("fence" ::: "memory");

    Cart_HoldMSXWait_End ();
    return 0U;
}
#endif /* USB_PSRAM_SYNC_COPY */

/* Dispatch: pick the configured SRAM -> PSRAM copy engine. */
static inline uint8_t psram_copy_chunk (const uint8_t *sram_src,
                                        uint32_t psram_addr,
                                        uint32_t byte_count) {
#if USB_PSRAM_SYNC_COPY
    return psram_sync_copy (sram_src, psram_addr, byte_count);
#else
    return psram_dma_copy (sram_src, psram_addr, byte_count);
#endif
}

/* ------------------------------------------------------------------------ */
/* High-level: file -> PSRAM                                                */
/* ------------------------------------------------------------------------ */

/* Sector-sized SRAM scratch buffer used as the bulk-IN target before
 * PSRAM DMA.  One sector (typically 512 B); aligned to 4.  Internal
 * SRAM is only ~135 KB but a single 512-byte buffer is fine. */
__attribute__ ((aligned (4))) static uint8_t s_sector_buf[512];

/* Optional progress callback for USB_FileToPSRAM (see usb_disk.h).
 * NULL by default - only the terminal loader installs one. */
void (*USB_ProgressCB) (uint32_t done, uint32_t total) = 0;

/* Set by USB_TryEnsureMounted() once f_mount() has succeeded.  Cleared
 * on every disconnect (so the next file op re-mounts).  Keeps the
 * "am I mounted?" check out of every f_opendir / f_open. */
static uint8_t s_fatfs_mounted = 0U;

void USB_InvalidateMount (void) {
    if (s_fatfs_mounted) {
        (void)f_mount (NULL, "0:", 0);
    }
    s_fatfs_mounted = 0U;
}

/* Attempt to (re-)enumerate the USB device and mount the FAT volume.
 * Returns 0 on success, non-zero (DEF_ERR_*) on failure.
 *
 * Use this from any CLI/file op BEFORE calling f_open / f_opendir so a
 * missed `USB` step (or a stick unplug/replug) doesn't silently fail
 * with FR_NO_FILESYSTEM (12).  Idempotent: cheap to call repeatedly. */
uint8_t USB_TryEnsureEnumerated (void) {
    /* 1. Poll the root hub.  Up to 5 attempts: freshly plugged sticks
     *    often need 200-500 ms to finish their post-enumeration
     *    housekeeping (especially flash translation layer init).
     *    USBH_PreDeal() itself does multiple enumeration retries
     *    internally, so this loop is mostly for the f_mount side. */
    uint8_t r = DEF_ERR_ENUM;
    for (int tries = 0; tries < 5; tries++) {
        /* Yield the service loop the moment the Z80 asks for anything:
         * a probe must never own the bus past a request arrival. */
        if (Nextor_RequestPending () != 0U) return DEF_ERR_ENUM;
        r = USBH_PreDeal ();
        if (r == DEF_ERR_DETECT) {
            /* firmware2 note: no FatFs volume is ever mounted here (the
             * file layer that mounted it - loader/terminal - is not
             * ported), so there is no f_mount(NULL) unmount on disconnect;
             * the FatFs file functions in this module stay unreferenced
             * and are garbage-collected at link time. */
            printf ("USB: stick removed\r\n");
            /* Whatever is caching block data against the old stick -
             * the Nextor engine's capacity/INQUIRY state - belongs to a
             * medium that no longer exists. Drop it so the next idle pass
             * re-probes the (re-)plugged stick.  Guarded by whether the
             * Nextor engine has ever been primed, so callers before
             * main()'s Nextor_Init are harmless no-ops. */
            Nextor_CacheInvalidate ();
            return DEF_ERR_DETECT;
        }
        if (r == DEF_SUCCESS) break;

        /* DEFINITIVE-FIX: DEF_DEFAULT is NOT success.
         *
         * USBHSH_CheckRootHubPortStatus() only reports state through the
         * PORT_STATUS_CHG *edge* flag, which the FIRST PreDeal call
         * consumed. Every later call with no new edge returns
         * ROOT_DEV_FAILED -> USBH_PreDeal falls through to DEF_DEFAULT
         * even while a stick sits enumerated on the port (or is attached
         * but was never enumerated because our first call missed the
         * edge). Treating DEF_DEFAULT as "already fine, break out"
         * credited a fake success above and f_mount then ran against a
         * HostCtl[] struct that had never been filled - the terminal
         * showed "USB not mounted" on every boot while the identical
         * code path under Nextor (RawDisk_Probe -> same function) only
         * ever ran as the FIRST enumerating call after a connect edge
         * and therefore worked.
         *
         * If the driver actually finished enumeration, credit success.
         * Otherwise force a fresh enumeration now; USBH_EnumRootDevice
         * is idempotent and does not need the edge flag. */
        if (RootHubDev[DEF_USB_PORT].bStatus == ROOT_DEV_SUCCESS) break;

        printf ("USB: no valid enumeration (PreDeal=%u), forcing\r\n",
                (unsigned)r);
        if (USBH_EnumRootDevice (DEF_USB_PORT) == ERR_SUCCESS) {
            RootHubDev[DEF_USB_PORT].bStatus = ROOT_DEV_SUCCESS;
            r = DEF_SUCCESS;
            break;
        }
        /* Still failing this pass: rate-limit and retry through the
         * normal top-of-loop path (the forced enumeration already
         * happened, so the next pass either sees SUCCESS or forces
         * again). */
        r = DEF_ERR_ENUM;
        Delay_Ms (100);
        continue;
    }
    if (r == DEF_ERR_ENUM) {
        return DEF_ERR_ENUM;
    }
    if (r != DEF_SUCCESS && r != DEF_DEFAULT) {
        return r;
    }
    return DEF_SUCCESS;
}

uint8_t USB_TryEnsureMounted (void) {
    uint8_t r = USB_TryEnsureEnumerated ();
    if (r != DEF_SUCCESS) {
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
                /* DEFINITIVE-FIX: recover via a real re-enumeration, not
                 * ClearUSB(). The old sequence wiped HostCtl[] (SCSI needs
                 * those endpoints) and then called USBH_PreDeal(), which
                 * - with the PORT_STATUS_CHG edge already consumed -
                 * happily returned DEF_DEFAULT without ever rebuilding the
                 * state it just destroyed. Force-enumerate instead: the
                 * routine is the same one PreDeal calls on a connect edge
                 * and it refills HostCtl[] from the stick's descriptors. */
                if (USBH_EnumRootDevice (DEF_USB_PORT) == ERR_SUCCESS) {
                    RootHubDev[DEF_USB_PORT].bStatus = ROOT_DEV_SUCCESS;
                } else {
                    /* Leave bStatus non-success -> disk_initialize (called
                     * by the next f_mount) will force-enumerate again. */
                    RootHubDev[DEF_USB_PORT].bStatus = ROOT_DEV_FAILED;
                }
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
    printf ("USB: FileToPSRAM path='%s' psram=0x%08lx len=%lu\r\n",
            path, (unsigned long)psram_addr, (unsigned long)len);
    if (USB_TryEnsureMounted () != DEF_SUCCESS) {
        printf ("USB: ensure-mounted FAILED - aborting copy\r\n");
        return 0U;
    }
    printf ("USB: volume mounted OK\r\n");

    FIL     fp;
    FRESULT fr;
    UINT    br;
    uint32_t total = 0U;

    fr = f_open (&fp, path, FA_READ);
    if (fr != FR_OK) {
        printf ("USB: f_open('%s') failed (%u)\r\n", path, (unsigned)fr);
        return 0U;
    }
    printf ("USB: f_open OK (size=%lu)\r\n", (unsigned long)f_size (&fp));

    /* Bounds check: psram_addr is the absolute PSRAM bus address
     * (always >= PSRAM_CART_BASE 0x80000000). Compare the offset
     * within the cart window against the game-image size
     * (CART_GAME_MAX_BYTES = PSRAM_CART_BASE + CART_GAME_BASE ..
     * PSRAM_CART_BASE + CART_SLOTTED_BANK_BASE_OFFSET). The previous
     * version compared against the full 8 MiB PSRAM_CART_SIZE, which
     * let oversize ROMs write into the sub-slot 1 mapper-RAM region
     * and silently corrupt the MSX-DOS mapper banks. */
    if (psram_addr < PSRAM_CART_BASE
        || psram_addr - PSRAM_CART_BASE >= CART_GAME_MAX_BYTES) {
        f_close (&fp);
        printf ("USB: PSRAM address 0x%08lx out of range (game window "
                "= 0x%08lx bytes at PSRAM+0x%08lx)\r\n",
                (unsigned long)psram_addr,
                (unsigned long)CART_GAME_MAX_BYTES,
                (unsigned long)(PSRAM_CART_BASE + CART_GAME_BASE));
        return 0U;
    }
    printf ("USB: PSRAM addr in range (offset %lu, game window %lu bytes)\r\n",
            (unsigned long)(psram_addr - PSRAM_CART_BASE),
            (unsigned long)CART_GAME_MAX_BYTES);

    if (len == 0U) {
        len = (uint32_t)f_size (&fp);
        printf ("USB: len=0 -> using file size %lu\r\n",
                (unsigned long)len);
    }

    uint32_t remaining = CART_GAME_MAX_BYTES - (psram_addr - PSRAM_CART_BASE);
    if (len > remaining) {
        /* Hard error, not a silent cap. A ROM that does not fit the
         * 4 MiB game window used to be silently truncated (and the
         * truncated tail usually left the cart unbootable). Surface
         * the size mismatch so the user knows to re-rip at smaller
         * size or pick a smaller ROM. */
        printf ("USB: REJECT: file %lu bytes > game window %lu bytes "
                "(sub-slot 1 mapper RAM is at PSRAM + 0x%08lx, "
                "%lu MiB reserved)\r\n",
                (unsigned long)len, (unsigned long)remaining,
                (unsigned long)CART_SLOTTED_BANK_BASE_OFFSET,
                (unsigned long)(CART_SLOTTED_BANK_BASE_OFFSET / (1024UL * 1024UL)));
        f_close (&fp);
        return 0U;
    }
    printf ("USB: copy plan: %lu bytes in %lu x %u-byte chunks\r\n",
            (unsigned long)len,
            (unsigned long)((len + sizeof (s_sector_buf) - 1U) /
                            sizeof (s_sector_buf)),
            (unsigned)sizeof (s_sector_buf));

    while (total < len) {
        uint32_t chunk = len - total;
        if (chunk > sizeof (s_sector_buf)) chunk = sizeof (s_sector_buf);

        /* DIAGNOSTIC (first 8 chunks): print at the TOP of each chunk
         * iteration. The DMA[0..3] prints inside psram_dma_copy bracket
         * the copy itself; this print brackets the f_read that runs
         * BEFORE each copy. With both prints present the sequence per
         * chunk is: "chunk N" -> "DMA[N]". A log that stops after
         * "DMA[0]" pins the freeze to either the chunk-1 f_read (stick
         * path) or the progress-bar callback between the two. */
        /* DIAGNOSTIC: EVERY chunk prints its index + offset while the
         * counter is under the cap (raised to 4096 = a 2 MiB copy's
         * worth of lines - generous for diagnosis, harmless because
         * re-armed per load). Sequence per chunk is now unambiguous:
         *   "chunk N" -> "R lba=..." -> "DMA[N]" (first 4) -> "bar"
         * The last line before a freeze pins the exact chunk. */
        if (s_chunk_dbg < 4096U) {
            printf ("USB: chunk %lu at offset %lu\r\n",
                    (unsigned long)(total / (uint32_t)sizeof (s_sector_buf)),
                    (unsigned long)total);
            s_chunk_dbg++;
        }

        fr = f_read (&fp, s_sector_buf, chunk, &br);
        if (fr != FR_OK) {
            printf ("USB: f_read FAILED at offset %lu: FRESULT=%u "
                    "(disk error / stick unplug?)\r\n",
                    (unsigned long)total, (unsigned)fr);
            break;
        }
        if (br == 0U) {
            printf ("USB: f_read returned 0 bytes at offset %lu "
                    "(unexpected EOF)\r\n", (unsigned long)total);
            break;
        }
        if (br != chunk) {
            printf ("USB: short read at offset %lu: got %u, wanted %lu "
                    "(treated as EOF)\r\n",
                    (unsigned long)total, (unsigned)br,
                    (unsigned long)chunk);
        }

        if (psram_copy_chunk (s_sector_buf, psram_addr + total,
                              (uint32_t)br) != 0U) {
            printf ("USB: PSRAM copy FAILED at offset %lu, len %u "
                    "(alignment or copy error)\r\n",
                    (unsigned long)total, (unsigned)br);
            break;
        }
        total += (uint32_t)br;
        /* Progress hook: call AFTER the chunk lands in PSRAM so the
         * percentage reflects data actually stored, not just read. */
        if (USB_ProgressCB != 0) {
            USB_ProgressCB (total, len);
        }
        if (br < chunk) break;     /* EOF */
    }

    printf ("USB: copy done: %lu / %lu bytes (%lu%%), last f_read "
            "FRESULT=%u\r\n",
            (unsigned long)total, (unsigned long)len,
            (unsigned long)(len ? (total * 100UL) / len : 0UL),
            (unsigned)fr);
    printf ("USB: closing file, final result %s\r\n",
            (total == len) ? "COMPLETE" : "INCOMPLETE");
    f_close (&fp);
    s_dbg_count = 0U;   /* re-arm the first-chunk debug for next load */
    s_chunk_dbg = 0U;
#if RISKY_DEBUG
    s_scsi_dbg  = 0U;
#endif
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
