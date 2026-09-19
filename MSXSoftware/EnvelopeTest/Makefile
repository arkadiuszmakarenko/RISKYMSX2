# Makefile - build the envelope-test MSX cartridge with SDCC.
#
# Requirements: sdcc, sdasz80, makebin (package "sdcc" on Debian/Ubuntu).
#
# Target:
#   envtest.bin  - 32 KiB envelope test ROM ("AB" header at 0x4000)
#
# Boots like the working hello.bin / fixed selector ROM: entry stub
# at 0x4010 (between header and _CODE at 0x4020) so makebin does not
# overwrite it with the first C function.

CC      = sdcc
AS      = sdasz80
CFLAGS  = -mz80 --no-std-crt0 --opt-code-size
ASFLAGS = -plosgff

CODE_LOC = 0x4020
DATA_LOC = 0xC000

all: envtest.bin

envtest.ihx: crt0.rel envtest.rel
	$(CC) $(CFLAGS) --code-loc $(CODE_LOC) --data-loc $(DATA_LOC) \
	       -o $@ crt0.rel envtest.rel

envtest.bin: envtest.ihx
	makebin -s 65536 < $< > envtest.full.bin
	dd if=envtest.full.bin of=$@ bs=16384 skip=1 count=2 status=none
	rm -f envtest.full.bin
	@echo "$@: $$(wc -c < $@) bytes (32 KiB ROM image)"

%.rel: %.s
	$(AS) $(ASFLAGS) $@ $<

%.rel: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.rel *.ihx *.bin *.full.bin *.lk *.map *.mem *.lst \
	      *.rst *.sym *.asm *.noi

.PHONY: all clean
