/*-----------------------------------------------------------------------/
/  Low level disk I/O module for FatFs.
/
/  Single-drive glue: physical drive 0 = USB stick on USBHS root hub.
/
/  This is a copy of v303Firmware/User/FATFS/diskio.c with the
/  `extern CircularBuffer scb` and the `utils.h` include removed (that
/  utility buffer is from a different project; the disk layer here only
/  needs the USBHS enumeration glue from usb_disk.{c,h}).
/
/  Writes are intentionally NOT supported: ffconf.h has FF_FS_READONLY=0
/  but the cart-loader use case is read-only (load ROMs from the stick
/  into PSRAM).  If write support is ever wanted, add a
/  usb_scsi_write_sector() implementation in usb_disk.c mirroring
/  usb_scsi_read_sector().
/-----------------------------------------------------------------------*/

#include "ff.h"     /* Obtains integer types */
#include "diskio.h" /* Declarations of disk functions */
#include "usb_disk.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM 0 /* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC 1 /* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB 2 /* Example: Map USB MSD to physical drive 2 */

static uint32_t block_count = 0, block_size = 0;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (BYTE pdrv) {
    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (BYTE pdrv) {
    ClearUSB();
    uint8_t res = USBH_PreDeal();
    if (res == 0 || res == 0xFF) {
        if (usb_scsi_read_capacity (&block_count, &block_size) != 0
            || block_size != 512) {
            return STA_NOINIT;
        }
        return 0;  /* Disk OK */
    }
    return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read (BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    for (UINT i = 0; i < count; i++) {
        if (usb_scsi_read_sector (sector + i, buff + i * 512,
                                  block_size) != 0) {
            return RES_ERROR;
        }
    }
    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s) - not supported                                       */
/*-----------------------------------------------------------------------*/
DRESULT disk_write (BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    (void)pdrv; (void)buff; (void)sector; (void)count;
    return RES_WRPRT;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
    uint32_t bc = 0, bs = 0;
    switch (cmd) {
    case GET_SECTOR_COUNT:
        if (usb_scsi_read_capacity (&bc, &bs) == 0) {
            *(DWORD *)buff = bc + 1;
            return RES_OK;
        }
        return RES_ERROR;
    case GET_SECTOR_SIZE:
        if (usb_scsi_read_capacity (&bc, &bs) == 0) {
            *(WORD *)buff = (WORD)bs;
            return RES_OK;
        }
        return RES_ERROR;
    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;
        return RES_OK;
    case CTRL_SYNC:
        return RES_OK;
    default:
        return RES_PARERR;
    }
}