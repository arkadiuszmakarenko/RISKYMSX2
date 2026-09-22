# Makefile for the RISKYMSX2 Nextor driver + kernel ROM pipeline.
#
#   make            build the flashable Nextor ROM (Nextor-3.0.RISKYMSX2.ROM)
#   make embed      rebuild the ROM AND regenerate firmware/User/nextor_rom.c
#   make test       build the ROM and boot-test it in openMSX (C-BIOS; the
#                   driver's mailbox handshake fails there -> kernel boots
#                   without the driver = the plan's graceful-degradation test)
#   make clean      remove build artifacts
#
# Tools: N80 (vendored, needs a .NET runtime; DOTNET_ROLL_FORWARD covers a
# newer runtime than N80 was built against) and mknexrom (vendored).
#
# Pipeline (docs/NEXTOR_PLAN.md D3):
#   N80 driver.asm -> driver.bin (exactly 16080 bytes, 0x4100..0x7FCF)
#   N80 chgbnk.asm -> chgbnk.bin (ASCII16 bank-switch tail, 0x7FD0..0x7FFF)
#   mknexrom <base.dat> <ROM> /d:driver.bin /m:chgbnk.bin
#   -> 8 banks x 16 KB = 128 KB ROM (7 kernel banks + driver bank 7)
#   tools/rom2c.py --nextor -> firmware/User/nextor_rom.c (.cartrom embed)

NAME      := RISKYMSX2
ROM       := Nextor-3.0.RISKYMSX2.ROM

VENDOR    := vendor
NEXTOR_SDK  := $(VENDOR)/nextor-sdk/sdk
NEXTOR_BASE := $(VENDOR)/Nextor-3.0.0-beta1.base.dat
N80         := $(VENDOR)/n80/N80
MKNEXROM    := $(VENDOR)/mknexrom

N80_FLAGS := --no-string-escapes --no-show-banner --verbosity 0 \
             --include-directory $(NEXTOR_SDK)

# Firmware-side embed target (mirrors the RomLoader embed flow: the
# firmware Makefile depends on this file and re-runs `make -C embed`).
FW_ROM_C := ../../firmware/User/nextor_rom.c
ROM2C    := ../../tools/rom2c.py

export DOTNET_ROLL_FORWARD := Major

.PHONY: all clean embed test

all: $(ROM)

embed: $(ROM) $(FW_ROM_C)

$(FW_ROM_C): $(ROM) $(ROM2C)
	python3 $(ROM2C) $(ROM) $(FW_ROM_C) \
	    "Nextor 3.0 beta 1 kernel + RISKYMSX2 USB driver (ASCII16 mapper)" \
	    --nextor

$(ROM): driver.bin chgbnk.bin
	@test -f "$(NEXTOR_BASE)" || { \
	    echo "*** Kernel base file not found: $(NEXTOR_BASE)"; exit 1; }
	$(MKNEXROM) $(NEXTOR_BASE) $@ /d:driver.bin /m:chgbnk.bin
	@echo "Built $@"

driver.bin: driver.asm
	$(N80) driver.asm driver.bin $(N80_FLAGS)
	@test $$(stat -c%s driver.bin) -eq 16336 || { \
	    echo "*** driver.bin is not 16336 bytes (expected 0x4000..0x7FCF: 256 dummy + 16080 code)"; exit 1; }

chgbnk.bin: chgbnk.asm
	$(N80) chgbnk.asm chgbnk.bin $(N80_FLAGS)

test: $(ROM)
	/tmp/opencode/msx/xvfb-openmsx.sh -machine C-BIOS_MSX2+ \
	    -carta $(CURDIR)/$(ROM) -romtype ASCII16 \
	    -command "after time 15 { screenshot /tmp/opencode/msx/nextor-test.png } ; after time 16 exit"

clean:
	rm -f driver.bin chgbnk.bin $(ROM) $(FW_ROM_C)
