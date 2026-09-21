# Makefile - build the MSX-side terminal ROM (terminal.bin).
#
# The terminal program is a port of v303firmwaer/MSXTerminal/msxterminal.asm
# to sdasz80 syntax (see terminal.asm for the porting notes). It builds
# into a 32 KiB ROM image that the RISKYMSX2 firmware serves as the
# CART_MAP_TERMINAL mapper (see firmware/User/cart.c).
#
# Targets:
#   terminal.bin  - 32 KiB terminal ROM ("AB" header at 0x4000)
#
# Output goes to ../terminal.bin so the parent Makefile can pass it
# to tools/rom2c.py for embedding into firmware/User/terminal_rom.c.

CC      = sdcc
AS      = sdasz80
CFLAGS  = -mz80 --no-std-crt0 --opt-code-size
ASFLAGS = -plosgff

OUTPUT_BIN = ../terminal.bin

.PHONY: all clean
all: $(OUTPUT_BIN)

terminal.rel: terminal.asm
	$(AS) $(ASFLAGS) $@ $<

terminal.ihx: terminal.rel
	$(CC) $(CFLAGS) -o $@ terminal.rel

$(OUTPUT_BIN): terminal.ihx
	@makebin -s 65536 < $< > terminal.full.bin
	@# Trim the 64 KiB makebin output to just the bytes we actually
	@# use: 0x4000..0x40FF (256 bytes) covers init (0x4010) + body
	@# (BODY_SIZE = 0x8E bytes ending at 0x40AC) + arrow sprite at
	@# 0x40F0..0x40F8. Reading cart addresses outside 0x4000..0x40FF
	@# on the MSX side never happens (the terminal program never
	@# fetches from beyond its own body), so padding is wasted flash.
	@dd if=terminal.full.bin of=$@ bs=1 skip=16384 count=256 status=none
	@rm -f terminal.full.bin
	@echo "$@: $$(wc -c < $@) bytes (trimmed ROM image)"

clean:
	rm -f *.rel *.ihx *.bin *.full.bin *.lst *.rst *.sym