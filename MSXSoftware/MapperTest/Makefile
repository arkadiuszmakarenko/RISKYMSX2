CC      = sdcc
AS      = sdasz80
CFLAGS  = -mz80 --no-std-crt0 --opt-code-size
ASFLAGS = -plosgff

CODE_LOC = 0x4020
DATA_LOC = 0xC000

all: maptest.bin

maptest.ihx: crt0.rel maptest.rel
	$(CC) $(CFLAGS) --code-loc $(CODE_LOC) --data-loc $(DATA_LOC) \
	       -o $@ crt0.rel maptest.rel

maptest.bin: maptest.ihx
	makebin -s 65536 < $< > maptest.full.bin
	dd if=maptest.full.bin of=$@ bs=16384 skip=1 count=2 status=none
	rm -f maptest.full.bin

%.rel: %.s
	$(AS) $(ASFLAGS) $@ $<

%.rel: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *.rel *.ihx *.bin *.full.bin *.lk *.map *.mem *.lst \
	      *.rst *.sym *.asm *.noi

.PHONY: all clean
