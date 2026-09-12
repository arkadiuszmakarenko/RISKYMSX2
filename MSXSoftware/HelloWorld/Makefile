# Makefile - build MSX ROM cartridge binaries with SDCC.
#
# Requirements: sdcc, sdasz80, makebin (package "sdcc" on Debian/Ubuntu).
#
# Targets:
#   hello.bin      - 32 KiB "Hello World" ROM (header "AB" at 0x4000)
#   psramdiag.bin  - 32 KiB PSRAM/bus measurement ROM for the HP 1660C
#                    (phases PH1..PH5, walking pattern @ 0x5000)
#   embed          - regenerate firmware/User/hello_rom.c from
#                    psramdiag.bin (via tools/rom2c.py) so the firmware
#                    serves the diagnostic ROM from BOTH the SRAM window
#                    and the PSRAM mirror at boot.
#   all           - hello.bin + psramdiag.bin
#
# ROM layout (both images):
#   0x4000  MSX ROM header ("AB", INIT=..., STATEMENT/DEVICE/TEXT = 0)
#   0x4020  .text (--code-loc)
#   0x5000  256-byte walking pattern (diagstub.s, psramdiag only)
#   0xC000  .data (--data-loc, MSX RAM)

CC      = sdcc
AS      = sdasz80
CFLAGS  = -mz80 --no-std-crt0 --code-loc 0x4020 --data-loc 0xC000 \
          --opt-code-size
ASFLAGS = -plosgff

# Repo-relative paths for the embed step (we sit in MSXSoftware/HelloWorld).
ROM2C   = ../../tools/rom2c.py
FWM_ROM = ../../firmware/User/hello_rom.c

all: hello.bin psramdiag.bin

# Extract pages 1-2 (0x4000-0xBFFF, 32 KiB) as the raw ROM image so
# that byte 0 of the .bin is the "AB" header at 0x4000.
hello.bin: hello.ihx
	makebin -s 65536 < $< > hello.full.bin
	dd if=hello.full.bin of=$@ bs=16384 skip=1 count=2 status=none
	rm -f hello.full.bin
	@echo "$@: $$(wc -c < $@) bytes (32 KiB ROM image, header 'AB' at byte 0)"

psramdiag.bin: psramdiag.ihx
	makebin -s 65536 < $< > psramdiag.full.bin
	dd if=psramdiag.full.bin of=$@ bs=16384 skip=1 count=2 status=none
	rm -f psramdiag.full.bin
	@echo "$@: $$(wc -c < $@) bytes (32 KiB diagnostic ROM)"

hello.ihx: crt0.rel hello.rel
	$(CC) $(CFLAGS) -o $@ crt0.rel hello.rel

psramdiag.ihx: crt0.rel psramdiag.rel diagstub.rel
	$(CC) $(CFLAGS) -o $@ crt0.rel psramdiag.rel diagstub.rel

# Regenerate the firmware's embedded ROM source from psramdiag.bin.
# The firmware then mirrors it into BOTH cart windows at boot:
#   - SRAM window: main() copies hello_rom[] -> s_cart_image_sram[]
#   - PSRAM window: PSRAM_Init() copies + byte-verifies it @ 0x80000000
embed: psramdiag.bin
	python3 $(ROM2C) psramdiag.bin $(FWM_ROM)
	@echo "embedded psramdiag.bin -> $(FWM_ROM)"

%.rel: %.c
	$(CC) $(CFLAGS) -c $<

%.rel: %.s
	$(AS) $(ASFLAGS) $@ $<

clean:
	rm -f *.rel *.ihx *.bin *.full.bin *.lk *.map *.mem *.lst *.rst \
	      *.sym *.asm *.noi

.PHONY: all clean embed