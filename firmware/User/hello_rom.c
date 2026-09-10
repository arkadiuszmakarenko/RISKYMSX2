/*
 * Cart image placeholder.
 *
 * The .cartrom section is declared empty - no flash data, no bytes.
 * The startup copy loop in startup_ch32v4x7.S copies 0 bytes from the
 * (empty) flash region into the SRAM mirror at SRAM_ROM_BASE, leaving
 * the mirror uninitialised. main() then zeroes the 32 KiB mirror so the
 * cart handler returns a deterministic value (0xFF, the MSX "open bus"
 * pattern for an unpopulated slot) before any XLOAD upload arrives.
 *
 * The actual cart ROM image is supplied at runtime by the CLI's XLOAD
 * command (or LOAD). On boot, main() prints:
 *
 *     Cart image: SRAM @ 0x20014400 (32 KiB, EMPTY - upload via XLOAD)
 *
 * and waits for an upload.
 */
#include <stdint.h>

/* Empty array - just provides the symbols and keeps the section present
 * so the linker emits _cartrom_vma / _cartrom_lma / _cartrom_end for the
 * startup copy loop. Without KEEP() and at least one input byte, the
 * linker would discard the section as unreferenced. */
__attribute__((section(".cartrom")))
const uint8_t hello_rom[1] = { 0xFF };
const uint32_t hello_rom_len = 1U;