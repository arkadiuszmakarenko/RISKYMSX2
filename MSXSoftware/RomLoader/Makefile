# Makefile - build the RISKYMSX2 ROM LOADER MSX cartridge with SDCC.
#
# Requirements: sdcc, sdasz80, makebin (package "sdcc" on Debian/Ubuntu).
#
# Targets:
#   selector.bin  - 32 KiB loader ROM ("AB" header at 0x4000)
#   embed         - regenerate firmware/User/selector_rom.c from
#                   selector.bin (via tools/rom2c.py) so the firmware
#                   boots straight into the selector
#   all           - selector.bin
#
# IMAGE ASSEMBLY (two-stage, because the payload must be linked at its
# RUN address but stored at a different ROM address):
#
#   Stage 1: link payload.ihx with --code-loc 0xC000 --data-loc 0xC800
#            (MSX RAM page 3).  sdcc emits position-DEPENDENT absolute
#            calls/jumps and merges zero-initialised globals into
#            _DATA right after _CODE, so the payload is one contiguous
#            copyable image.  The gap 0xC6F1..0xC800 between code end
#            and data start is zero-filled by makebin - harmless.
#
#            IMPORTANT: data/bss must NOT land in 0x8000..0xBFFF (page
#            2) - the cart claims that page, so Z80 variable accesses
#            would hit the cartridge bus, not RAM.  Page 3 (0xC000+)
#            is main RAM on every MSX.
#
#   Stage 2: makebin the payload into a 64 KiB image, extract the
#            0xC000..0xC000+span bytes as payload.bin (trailing zero
#            bytes trimmed).
#
#   Stage 3: assemble the ROM: header + stub at 0x4000 (crt0.rel linked
#            at its ROM address), patch payload_len at 0x4010, place
#            payload.bin at 0x4040, pad to 32 KiB with 0xFF.
#
# The loader talks to the firmware via the mailbox at cart window
# 0x7FF0..0x7FFF (see romloader.c header comment) while the firmware's
# LOADER mapper is installed.

CC      = sdcc
AS      = sdasz80
CFLAGS  = -mz80 --no-std-crt0 --opt-code-size
ASFLAGS = -plosgff

PAYLOAD_CODE_LOC = 0xC000   # MSX RAM run address (page 3)
PAYLOAD_DATA_LOC = 0xC800   # data right after code (page 3, NOT page 2!)

# Repo-relative paths for the embed step (we sit in MSXSoftware/RomLoader).
ROM2C   = ../../tools/rom2c.py
FWM_ROM = ../../firmware/User/selector_rom.c
ROM_BIN = selector.bin

all: $(ROM_BIN)

# ---- Stage 1+2: the RAM-resident payload -------------------------------
payload.ihx: romloader.c
	$(CC) $(CFLAGS) --code-loc $(PAYLOAD_CODE_LOC) \
	       --data-loc $(PAYLOAD_DATA_LOC) -o $@ romloader.c

payload.bin: payload.ihx payload.map
	makebin -s 65536 < $< > payload.flat.bin
	python3 extract_payload.py
	rm -f payload.flat.bin

# ---- Stage 3: header + stub + payload ---------------------------------
stub.ihx: crt0.rel
	-$(CC) $(CFLAGS) --code-loc 0x4000 -o $@ crt0.rel

stub.flat.bin: stub.ihx
	makebin -s 65536 < $< > $@

# The final ROM image is now called selector.bin, but keep the legacy
# romloader.bin name as an alias for older scripts.
$(ROM_BIN): stub.flat.bin payload.bin assemble_rom.py
	python3 assemble_rom.py "$@"

romloader.bin: $(ROM_BIN)
	@:

# Regenerate the firmware's embedded ROM source from selector.bin.
embed: $(ROM_BIN)
	python3 $(ROM2C) $(ROM_BIN) $(FWM_ROM)
	@echo "embedded $(ROM_BIN) -> $(FWM_ROM)"

%.rel: %.s
	$(AS) $(ASFLAGS) $@ $<

clean:
	rm -f *.rel *.ihx *.bin *.flat.bin *.lk *.map *.mem *.lst *.rst \
	      *.sym *.asm *.noi

.PHONY: all clean embed