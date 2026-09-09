# Makefile - build an MSX ROM cartridge binary (hello.bin) with SDCC.
#
# Requirements: sdcc, sdasz80, makebin (package "sdcc" on Debian/Ubuntu).
# Output: hello.bin - a plain 32 KiB ROM image (header "AB" at 0x4000),
#         suitable for flashing to the cartridge / loading in an emulator.

TARGET  = hello
CC      = sdcc
AS      = sdasz80
CFLAGS  = -mz80 --no-std-crt0 --code-loc 0x4020 --data-loc 0xC000 \
          --opt-code-size
ASFLAGS = -plosgff

OBJS    = crt0.rel $(TARGET).rel

all: $(TARGET).bin

# Extract pages 1-2 (0x4000-0xBFFF, 32 KiB) as the raw ROM image so
# that byte 0 of the .bin is the "AB" header at 0x4000.
$(TARGET).bin: $(TARGET).ihx
	makebin -s 65536 < $< > $(TARGET).full.bin
	dd if=$(TARGET).full.bin of=$@ bs=16384 skip=1 count=2 status=none
	rm -f $(TARGET).full.bin
	@echo "$@: $$(wc -c < $@) bytes (32 KiB ROM image, header 'AB' at byte 0)"

$(TARGET).ihx: $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

%.rel: %.c
	$(CC) $(CFLAGS) -c $<

%.rel: %.s
	$(AS) $(ASFLAGS) $@ $<

clean:
	rm -f *.rel *.ihx *.bin *.lk *.map *.mem *.lst *.rst *.sym *.asm *.noi

.PHONY: all clean
