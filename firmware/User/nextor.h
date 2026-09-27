/*
 * nextor.h - Nextor kernel mapper for the RISKYMSX2 cartridge.
 *
 * The mapper is ASCII16K + a mailbox window punched out of the bank:
 *
 *   0x4000..0x7FFF : 16 KiB bank n of the embedded kernel ROM
 *                    (nextor_rom[], flash-resident)
 *   0x6000 (W)     : select bank n for page 1
 *   0x8000..0xBFFF : 16 KiB bank m of the same ROM
 *   0x7000 (W)     : select bank m for page 2
 *   0x77FF (W)     : alias of 0x7000 (some Nextor drivers use it)
 *   0x7FF0..0x7FF5 : mailbox, decoded by address in every paged-in bank
 *
 * nextor_rom[] is the 128 KiB image built by
 * `make -C MSXSoftware/NextorDriver` (Nextor-3.0.RISKYMSX2.ROM) and
 * embedded by the firmware Makefile's regenerate_nextor_rom target.
 * 128 KiB = 8 banks x 16 KiB, so the bank number in a 0x6000 / 0x7000
 * write is 0..7.
 *
 * The kernel's own driver (MSXSoftware/NextorDriver/driver.asm) runs
 * from page 1 and reaches the disk ONLY through the mailbox. It has no
 * other hardware interface: no I/O ports, no ATA task file, no Z80 RAM
 * work area. That is why the mailbox is the single extension point.
 *
 * Bus-cycle routing and the ASCII16K bank decode live in
 * cart.c::Cart_EXTI0_Nextor_Handler; everything behind the mailbox
 * (command dispatch, result FIFO, disk access) lives in nextor.c.
 */

#ifndef __NEXTOR_H
#define __NEXTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================
 * Cart window geometry
 *
 * Fixed contract - MSXSoftware/NextorDriver/driver.asm is assembled
 * against these addresses, so they must not drift.
 * ====================================================================== */

/* Bank-select registers. A write here latches the 16 KiB bank for that
 * page; the value written IS the bank number (no bit-reversing, no
 * masking). Page 3 mirrors page 2 by the MSX slot logic. */
#define NEXTOR_BANK_REG_PAGE1     0x6000U
#define NEXTOR_BANK_REG_PAGE2     0x7000U
#define NEXTOR_BANK_REG_PAGE2_ALT 0x77FFU

/* Bank size and origin, used to turn a bank number into a byte offset
 * into nextor_rom[]:  off = (bank << 14) + (address & 0x3FFF). */
#define NEXTOR_BANK_SHIFT         14
#define NEXTOR_BANK_SIZE          (1U << NEXTOR_BANK_SHIFT)   /* 16 KiB */

/* Mailbox window. The whole 0x7FF0..0x7FFF page slice is decoded by
 * the handler (one AND + compare) and handed to nextor.c, which
 * interprets the individual registers. The six registers below are the
 * ones driver.asm defines; 0x7FF6..0x7FFF are decoded by the handler
 * but unused by the protocol.
 *
 * The register index is therefore a straight 1:1 with the low address
 * byte, and the window test has to keep A0..A3. That is safe because
 * A0 and A1 really are on the bus: the bank registers in cart.c are
 * compared exactly (0x6000, 0x7000, and 0x77FF which pins A0..A7
 * individually), and bank switching works, so the low address bits
 * arrive intact. A 4-byte-stride layout would tolerate their absence
 * but is not needed and would break the existing window test. */
#define NEXTOR_MBOX_BASE          0x7FF0U
#define NEXTOR_MBOX_END           0x7FF5U

/* Mailbox register indices (address - NEXTOR_MBOX_BASE). */
#define NEXTOR_MBOX_CMD           0x00U   /* W: start a command          */
#define NEXTOR_MBOX_STAT          0x00U   /* R: status bits              */
#define NEXTOR_MBOX_DATA          0x01U   /* R: pop result / W: push arg */
#define NEXTOR_MBOX_RXCNT_LO      0x02U   /* R: result bytes available  */
#define NEXTOR_MBOX_RXCNT_HI      0x03U
#define NEXTOR_MBOX_ERR           0x04U   /* R: last error code         */
#define NEXTOR_MBOX_VER           0x05U   /* R: firmware version byte   */

/* Decode a cart address to a register index. Reached only for addresses
 * inside the window, so no further range check is needed. */
#define NEXTOR_MBOX_REG(address) \
    ((uint8_t)((uint16_t)(address) - NEXTOR_MBOX_BASE))

/* STATUS bits (bit7 MSB first - the driver tests them as a byte). */
#define NEXTOR_STAT_READY         0x80U   /* firmware alive            */
#define NEXTOR_STAT_DONE          0x40U   /* command finished           */
#define NEXTOR_STAT_ERR           0x20U   /* command failed             */
#define NEXTOR_STAT_RX_AVAIL      0x10U   /* result FIFO not empty      */

/* ERR port values. */
#define NEXTOR_ERR_NONE           0x00U
#define NEXTOR_ERR_NO_MEDIA       0x01U
#define NEXTOR_ERR_IO             0x02U
#define NEXTOR_ERR_TIMEOUT        0x03U

/* Command bytes written to NEXTOR_MBOX_CMD. */
#define NEXTOR_CMD_HANDSHAKE      0x00U
#define NEXTOR_CMD_CAPACITY       0x01U
#define NEXTOR_CMD_STATUS         0x02U
#define NEXTOR_CMD_READ           0x03U
#define NEXTOR_CMD_WRITE          0x04U
#define NEXTOR_CMD_ABORT          0x05U
#define NEXTOR_CMD_STAPEEK        0x06U
#define NEXTOR_CMD_MAX            0x07U

/* Handshake reply: "RNX2" + version byte. */
#define NEXTOR_MAGIC              "RNX2"
#define NEXTOR_VERSION            0x01U

/* ========================================================================
 * Public API
 * ====================================================================== */

/* Initialise the mapper + mailbox to power-on state. Called from main()
 * and from Cart_SetMapper() on every swap to CART_MAP_NEXTOR. */
void Nextor_Init (void);

/* Main-loop service hook: drains the command in flight into the result
 * FIFO. Must be called from main(), never from IRQ context - it is the
 * only place blocking disk work is allowed. */
void Nextor_Service (void);

/* --- Code entries for Cart_EXTI0_Nextor_Handler (IRQ context) ------- */

/* Read entry. The handler calls this ONLY for addresses inside the
 * mailbox window (0x7FF0..0x7FF5); every other cart read is served
 * from nextor_rom[] by the handler itself. Returns the mailbox register
 * selected by (address - NEXTOR_MBOX_BASE).
 *
 * MUST be IRQ-safe - no blocking, no printf, no long loops. */
uint8_t Nextor_ReadByte (uint16_t address);

/* Write entry. The handler calls this ONLY for addresses inside the
 * mailbox window. Same constraints as the read entry. */
void Nextor_WriteByte (uint16_t address, uint8_t value);

/* -------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __NEXTOR_H */
