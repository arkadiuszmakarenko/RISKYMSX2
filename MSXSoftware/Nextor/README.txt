Drop your Sunrise IDE Master Only Nextor kernel image here as
nextor_sunrise.bin and rebuild the firmware.

The file must be a 16 KiB-aligned ROM image (typically 128 KiB / 1 MiB
ROM capacity) compatible with the Carnivore2-style Sunrise IDE mapper
decode:

  - Page 1 (0x4000..0x7FFF) is a 16 KiB banked window. The bank index
    is bit-reversed and combined with the IDE-enable flag when the
    Z80 writes to 0x4104. Bit-reverse is per Carnivore2 VHDL:
    bank = (value & 0x01) << 2 | (value & 0x02) << 1 | (value & 0x04);
    value bit 7 is the IDE enable.

  - The 0x7C00..0x7DFF window is the 16-bit ATA data register (PIO).

  - The 0x7E00..0x7EFF window is the ATA task-file register file
    (mirrored every 16 bytes).

  - The first 16 bytes of bank 0 contain the standard 'AB' cart header
    (0x41 0x42 0x.. 0x..).

Working sources for the .bin file:

  - The blueMSX "SunRise_IDE_MO_Nextor-3.0.bin" drop.
  - The Sunrise IDE / Carnivore2 firmware repository's release bins.
  - The Nextor-3.0.x Carnivore2 build from the official SDK with the
    sunrise_ide_master_only.rom target.

A 16-byte ROM header at offset 0x10 of bank 0 should look like
"AB" (the standard MSX cartridge ID). Some builds place the 'AB' at
0x0000 of bank 0 (nextor_rom[0..1]); others place it at 0x4000 (the
cart-window read of 0x4000 needs to look at nextor_rom[0x10] in
sector mode). Check the .bin against the Sunrise IDE mapper source in
firmware/User/sunrise_ide.c::Sunrise_IDE_ReadByte before assuming a
particular layout.