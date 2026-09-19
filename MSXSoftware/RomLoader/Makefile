# Makefile - build the RISKYMSX2 ROM LOADER MSX cartridge with SDCC.
#
# Requirements: sdcc, sdasz80, makebin (package "sdcc" on Debian/Ubuntu).
#
# Targets:
#   selector.bin  - 32 KiB loader ROM ("AB" header at 0x4000)
#   embed         - regenerate firmware/User/maptest_rom.c (the file the
#                   CART_MAP_FLASH mapper actually serves) and
#                   firmware/User/selector_rom.c (legacy mirror) from
#                   selector.bin via tools/rom2c.py
#   all           - selector.bin
#
# The cartridge now boots like a normal MSX ROM cartridge (same shape
# as the temporary hello.bin test image): the code is linked at 0x4020,
# runs directly from ROM, and uses the mailbox window at 0x7FF0..0x7FFF
# as the shared envelope for USB directory, file, and mapper data.

CC      = sdcc
AS      = sdasz80
CFLAGS  = -mz80 --no-std-crt0 --opt-code-size
ASFLAGS = -plosgff

CODE_LOC = 0x4020   # standard ROM code entry after the 16-byte header
DATA_LOC = 0xC000   # MSX RAM page 3

# Repo-relative paths for the embed step (we sit in MSXSoftware/RomLoader).
ROM2C   = ../../tools/rom2c.py
# The cart's FLASH mapper serves `maptest_rom[]` from this C file
# (see firmware/Makefile ROM_EMBED_SRC := User/maptest_rom.c, and
# Cart_EXTI0_Flash_Handler in cart.c which reads maptest_rom[]). The
# selector_rom.c copy is kept in sync for the legacy CLI / debug paths
# that look it up by name. This target drives the file the boot mapper
# actually uses.
FWM_ROM = ../../firmware/User/maptest_rom.c
FWM_ROM_SFX = ../../firmware/User/selector_rom.c  # mirror copy
ROM_BIN = selector.bin

all: $(ROM_BIN)

# ---- Direct ROM build --------------------------------------------------
romloader.ihx: crt0.rel romloader.rel
	$(CC) $(CFLAGS) --code-loc $(CODE_LOC) --data-loc $(DATA_LOC) \
	       -o $@ crt0.rel romloader.rel

$(ROM_BIN): romloader.ihx
	makebin -s 65536 < $< > $(ROM_BIN).full.bin
	dd if=$(ROM_BIN).full.bin of=$@ bs=16384 skip=1 count=2 status=none
	rm -f $(ROM_BIN).full.bin
	@echo "$@: $$(wc -c < $@) bytes (32 KiB ROM image)"

romloader.bin: $(ROM_BIN)
	@:

# Regenerate the firmware's embedded ROM source from selector.bin.
embed: $(ROM_BIN)
	python3 $(ROM2C) $(ROM_BIN) $(FWM_ROM)
	# Mirror copy under the legacy name so the CLI / debug paths that
	# reference selector_rom[] stay in sync with what the cart serves.
	cp -f $(FWM_ROM) $(FWM_ROM_SFX)
	@echo "embedded $(ROM_BIN) -> $(FWM_ROM) (and $(FWM_ROM_SFX))"

%.rel: %.s
	$(AS) $(ASFLAGS) $@ $<

%.rel: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.rel *.ihx *.bin *.flat.bin *.lk *.map *.mem *.lst *.rst \
	      *.sym *.asm *.noi

.PHONY: all clean embed