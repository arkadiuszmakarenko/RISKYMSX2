/*
 * nextor.c - Nextor kernel mapper support: the mailbox window.
 *
 * The ASCII16K bank decode and the ROM read path live in
 * cart.c::Cart_EXTI0_Nextor_Handler. This module owns everything
 * BEHIND the mailbox - the six registers at 0x7FF0..0x7FF5 that
 * MSXSoftware/NextorDriver/driver.asm uses to reach the disk.
 *
 *   1. Lifecycle
 *        Nextor_Init      - power-on mailbox state
 *        Nextor_Service   - main-loop hook, drains the command in flight
 *   2. Code entries for Cart_EXTI0_Nextor_Handler
 *        Nextor_ReadByte  - IRQ context, mailbox reads only
 *        Nextor_WriteByte - IRQ context, mailbox writes only
 *
 * The handler calls the two code entries ONLY for addresses inside
 * NEXTOR_MBOX_BASE..NEXTOR_MBOX_END; every other cart read is served
 * from nextor_rom[] by the handler. Register semantics (addresses,
 * status bits, command bytes) are in nextor.h.
 *
 * STATUS: the mailbox is a skeleton - the entries are no-ops, so the
 * driver sees a dead mailbox and the kernel will not mount the disk.
 * The bank-switching half of the mapper is complete and correct, so
 * the kernel ROM itself boots and its driver code runs; only the disk
 * is unreachable.
 */

#include "nextor.h"


extern const uint8_t  nextor_rom[];
extern const uint32_t nextor_rom_len;

/* ========================================================================
 * 1. Lifecycle
 * ====================================================================== */

/* Power-on mailbox state. Called from main() at boot and again from
 * Cart_SetMapper() on every swap to CART_MAP_NEXTOR.
 *
 * READY is set unconditionally: it means "the firmware is alive and
 * answering", which is the driver's very first question. DONE stays
 * clear until a command actually completes - the driver's
 * MB_WAIT_FOR_DONE loop has to see it change, so pre-setting it would
 * make every command look like it returned stale results. */
void Nextor_Init (void) {
    /* TODO: clear the result FIFO, the argument collector, the error
     * code, and the command-in-flight state; set STATUS = READY. */
}

/* Main-loop service hook. Called from main()'s idle loop; never from
 * IRQ context. This is the only place blocking disk work may happen:
 * a command written to the mailbox sets CMD_PENDING here, and the work
 * runs between Z80 bus cycles so the driver's polling loop can make
 * progress.
 *
 * TODO: if a command is pending, run it, fill the result FIFO, set or
 * clear DONE/ERR according to the outcome, and clear the pending flag. */
void Nextor_Service (void) {
}

/* ========================================================================
 * 2. Code entries - called from Cart_EXTI0_Nextor_Handler.
 *
 * Both are reached only for addresses in NEXTOR_MBOX_BASE..
 * NEXTOR_MBOX_END, so the register index is just (address -
 * NEXTOR_MBOX_BASE) and needs no further range check. Keeping the
 * window test in the handler means every decode miss there is
 * address-local and silent.
 *
 * IRQ context. No blocking calls, no printf, no long loops.
 * ====================================================================== */

uint8_t Nextor_ReadByte (uint16_t address) {
    /* Not implemented. 0xFF is the floating-bus pattern the MSX reads
     * when nothing drives the data bus, which is also what the handler
     * produces for an out-of-ROM address - so the driver's polling
     * loops see a stable, if wrong, value and time out instead of
     * hanging forever. */
    (void)address;
    return 0xFFU;
}

void Nextor_WriteByte (uint16_t address, uint8_t value) {
    /* Not implemented - writes are dropped, so no command ever starts. */
    (void)address;
    (void)value;
}
