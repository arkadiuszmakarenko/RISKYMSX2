# =============================================================================
# RISKYMSX2 firmware2 build (minimal Nextor boot firmware)
# =============================================================================
# Adapted from ../firmware/Makefile. Scope of this firmware: boot the
# cartridge into the Nextor kernel mapper (ASCII16K over the flash-resident
# nextor_rom[]), expose the expanded-slot sub-slots and the PSRAM-backed
# MSX memory mapper, and serve the kernel driver's mailbox disk requests
# through the USB host stack. Nothing else - no game mappers, no SCC, no
# terminal, no loader.
#
# Usage (from this directory, i.e. firmware2/RISKYMSX2/):
#
#   make            # incremental build -> obj/RISKYMSX2.{elf,hex,lst,map}
#   make -B         # force rebuild (all sources)
#   make clean      # rm -rf obj/
#   make rebuild    # clean + all
#   make size       # print RAM/Flash usage vs capacity
#   make list       # show which CC/openocd/chip are in use
#   make flash      # program the chip with OpenOCD (WCH-Link E)
#   make prog       # alias for flash
#
# Toolchain:
#   Override RISKY_TC=<path-to-bin-dir> or TRI=<gcc-prefix>. Default points
#   at /opt/riscv-toolchain/RISC-V Embedded GCC15/bin (the WCH MRS toolchain).
#
# Chip:
#   CH32V4x7 (CH32V467VET: 512K flash + 200K SRAM). Ld/Link.ld selects the
#   FLASH-512K + RAM-200K memory split.
#
# Nextor ROM:
#   User/nextor_rom.c is GENERATED from ../../MSXSoftware/NextorDriver
#   (N80 -> driver.bin, mknexrom -> 8 x 16 KiB banks, tools/rom2c.py embed).
#   Editing driver.asm and running `make` here is enough - the ROM is
#   rebuilt and re-embedded first. Override with NEXTOR_ROM=<file> to serve
#   a prebuilt image instead.
# =============================================================================

PROJECT       := RISKYMSX2
LINKER        := Ld/Link.ld
.DEFAULT_GOAL  := all

# --- Memory capacity (mirrors Ld/Link.ld) -----------------------------------
# FLASH 512K; RAM region is 200K-1K = 199K (the linker script places RAM at
# 0x20000000+1024 and keeps the first 1 KiB out of the region).
FLASH_SIZE    := 524288      # 512 * 1024
RAM_SIZE      := 203776      # 199 * 1024

# --- Parallel builds --------------------------------------------------------
ifeq (,$(filter -j -j% --jobs,$(MAKEOVERRIDES)))
  RISKY_JOBS ?= 8
  MAKEFLAGS  += -j$(RISKY_JOBS)
endif

# --- Toolchain --------------------------------------------------------------
TOOLCHAIN_DIR ?= /opt/riscv-toolchain/RISC-V Embedded GCC15/bin
TRI           ?= riscv32-wch-elf
CC            := $(TOOLCHAIN_DIR)/$(TRI)-gcc
OBJCOPY       := $(TOOLCHAIN_DIR)/$(TRI)-objcopy
OBJDUMP       := $(TOOLCHAIN_DIR)/$(TRI)-objdump
SIZE          := $(TOOLCHAIN_DIR)/$(TRI)-size

ARCH_FLAGS    ?= $(RISKY_ARCH)

CFLAGS_COMMON  = $(ARCH_FLAGS) \
                 -fmax-errors=20 -O3 -fmessage-length=0 \
                 -fsigned-char -ffunction-sections -fdata-sections -fno-common \
                 -Wunused -Wuninitialized \
                 -g -gdwarf-4 -std=gnu99

# This firmware is SILENT by default: printf is wrapped at link time so
# neither formatting nor UART transmission can steal main-loop time while
# the Z80 is polling the mailbox. The UART is never initialized at all.
# Build with QUIET=0 to get the real polled-UART printf back (you must
# then also add USART_Printf_Init to main()).
QUIET           ?= 1
DEBUG           ?= 0
ifeq ($(DEBUG),1)
  CFLAGS_COMMON += -DRISKY_DEBUG=1
  DEBUG_MODE     := debug
else
  CFLAGS_COMMON += -DRISKY_DEBUG=0
  DEBUG_MODE     := release
endif
ifeq ($(QUIET),1)
  CFLAGS_COMMON += -DRISKY_QUIET=1
  QUIET_LDFLAGS  := -Wl,--wrap=printf
  BUILD_MODE     := $(DEBUG_MODE)-quiet
else
  BUILD_MODE     := $(DEBUG_MODE)
endif
# Fold the cart diagnostics into the mode stamp so flipping them
# rebuilds every object (the .o files do not track these flags).
BUILD_MODE := $(BUILD_MODE)-s$(CART_TIMING_STATS)-la$(CART_LA_MARKER)

# PROBES=1 forwards -ds RW_PROBES to the driver build, re-enabling the
# per-sector value markers and the per-READ_WRITE frame trace that the
# read hot path leaves out by default. It only changes the embedded ROM,
# never the firmware image.
PROBES          ?= 0

# --- Game base (PSRAM placement diagnostic) ----------------------------------
# Where inside the 8 MiB PSRAM window the cart image window starts, in
# whole MiB. 0 = default. Only the USB staging path consumes it here.
#     make rebuild GAME_BASE_MB=2
GAME_BASE_MB ?= 0
CFLAGS_COMMON += -DCART_GAME_BASE_MB=$(GAME_BASE_MB)

# --- Link-Time Optimization (bus-handler latency) -----------------------------
# See ../firmware/Makefile for the rationale. OFF by default: the HPE
# ("WCH-Interrupt-fast") handler markers interacting with LTO is not
# boot-tested. Enable, flash, and verify a cold boot before keeping it.
RISKY_LTO         ?= 0
ifeq ($(RISKY_LTO),1)
  CFLAGS_COMMON += -flto
  LTO_LDFLAGS     := -flto
else
  LTO_LDFLAGS     :=
endif

# PSRAM serve-latency diagnostics (see User/cart.c): CART_TIMING_STATS
# times every mapper-RAM serve (entry -> byte on bus) and counts "late"
# serves; CART_LA_MARKER pulses PA0 around each PSRAM access for a logic
# analyser. Both add cycles inside the bus interrupt - diagnostic builds
# only. The mode stamp below makes switching them rebuild everything:
#
#     make rebuild QUIET=0 DEBUG=1 CART_TIMING_STATS=1 CART_LA_MARKER=1
#
CART_TIMING_STATS ?= 0
CFLAGS_COMMON += -DCART_TIMING_STATS=$(CART_TIMING_STATS)
CART_LA_MARKER  ?= 0
CFLAGS_COMMON += -DCART_LA_MARKER=$(CART_LA_MARKER)

# --- Flash tool --------------------------------------------------------------
# CH32V4x7 (chip id 0xa6) is NOT supported by upstream `wlink` or
# `minichlink`; the WCH fork of OpenOCD drives it via the SDI module.
OCD           ?= $(shell command -v openocd 2>/dev/null)
RISKY_OCD     ?= /usr/share/MRS2/MRS-linux-x64/resources/app/resources/linux/components/WCH/OpenOCD/OpenOCD/bin/openocd
RISKY_OCD_CFG ?= $(dir $(RISKY_OCD))wch-riscv.cfg

WLINK         ?= wlink
WLINK_CHIP    ?= CH32V30X         # CH32V4x7 family is selected via CH32V30X id
WLINK_ADDR    ?= 0x00000000       # WCH openocd flash bank base

# --- Directories -------------------------------------------------------------
SRCDIRS        := Startup Debug Core Peripheral/src \
                  User User/FATFS User/USB_Host
INCDIRS        := -IDebug -ICore \
                  -IUser -IUser/FATFS -IUser/USB_Host \
                  -IPeripheral/inc -IStartup
REPO_ROOT       := $(CURDIR)/../..
MSX_SOFTWARE_DIR := $(REPO_ROOT)/MSXSoftware
# The embedded ROM is a GENERATED SOURCE with a real file target and real
# prerequisites, so `make` derives it in dependency order (see the
# nextor_rom.c rule further down).
ROM_EMBED_SRC    := User/nextor_rom.c

# Nextor kernel ROM. Built from the driver sources in
# ../../MSXSoftware/NextorDriver (N80 -> driver.bin, then mknexrom packs
# 8 x 16 KiB banks), so editing driver.asm and running `make` here is
# enough - the ROM is rebuilt first, then re-embedded.
NEXTOR_DIR     := $(MSX_SOFTWARE_DIR)/NextorDriver
NEXTOR_ROM     ?= $(NEXTOR_DIR)/Nextor-3.0.RISKYMSX2.ROM
ROM2C          := $(REPO_ROOT)/tools/rom2c.py
NEXTOR_OWN_ROM := $(if $(filter $(NEXTOR_DIR)/Nextor-3.0.RISKYMSX2.ROM,$(NEXTOR_ROM)),yes,no)
NEXTOR_ASM     := $(wildcard $(NEXTOR_DIR)/*.asm)
OBJDIR         := obj

# --- Discover sources automatically ------------------------------------------
SRC_C          := $(wildcard Startup/*.c  Startup/*.S \
                    Debug/*.c \
                    Core/*.c \
                    Peripheral/src/*.c \
                    User/*.c \
                    User/FATFS/*.c \
                    User/USB_Host/*.c) $(ROM_EMBED_SRC)
SRC_C          := $(sort $(SRC_C))

# Objects mirror source tree under $(OBJDIR)/<subdir>/<name>.o.
OBJ_C          := $(patsubst %.c,$(OBJDIR)/%.o,$(SRC_C))
OBJ_S          := $(patsubst %.S,$(OBJDIR)/%.o,$(OBJ_C))
OBJS           := $(OBJ_S)
DEPS           := $(OBJS:.o=.d)

# Mode changes rebuild every object automatically.
BUILD_MODE_STAMP := $(OBJDIR)/.build-mode
$(OBJS): $(BUILD_MODE_STAMP)
$(BUILD_MODE_STAMP): FORCE
	@mkdir -p "$(OBJDIR)"
	@if test "`cat '$@' 2>/dev/null`" != "$(BUILD_MODE)"; then \
		echo "$(BUILD_MODE)" > "$@"; \
	fi

.PHONY: FORCE
FORCE:

# --- Output files ------------------------------------------------------------
ELF            := $(OBJDIR)/$(PROJECT).elf
HEX            := $(OBJDIR)/$(PROJECT).hex
LST            := $(OBJDIR)/$(PROJECT).lst
MAP            := $(OBJDIR)/$(PROJECT).map

# --- Toolchain sanity check (runs once per build) ----------------------------
ifeq ($(shell test -x '$(CC)' && echo OK),)
  $(error Toolchain not found or executable: $(CC). \
          Set RISKY_TC=<path-to-bin-dir> or TRI=<prefix>, or install the \
          riscv-none-embed gcc.)
endif

# Auto-detect the right -march syntax for the XW extension on this CC.
ARCH_TRY_LIST := \
  rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw \
  rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_xw \
  rv32imacxw \
  rv32imac_xw

CCQ := '$(CC)'
DETECTED_ARCH := $(shell \
  for cand in $(ARCH_TRY_LIST); do \
    if $(CCQ) -march=$$cand -mabi=ilp32 -E -x c /dev/null >/dev/null 2>&1; then \
      echo "$$cand"; break; \
    fi; \
  done)

ifeq ($(RISKY_ARCH),)
  ifeq ($(DETECTED_ARCH),)
    $(error Could not find a working -march for WCH XW on $(CC). \
            Tried: $(ARCH_TRY_LIST). Set RISKY_ARCH=... manually.)
  endif
  ARCH_FLAGS := -march=$(DETECTED_ARCH) -mabi=ilp32 -msmall-data-limit=8 -msave-restore
endif


# =============================================================================
# Rules
# =============================================================================
.PHONY: all clean rebuild flash prog size list read erase showsize regenerate_nextor_rom

all: $(ELF) $(HEX) $(LST) showsize
	@echo "*** $(PROJECT) build complete ***"

# --- Link --------------------------------------------------------------------
$(ELF): $(OBJS) $(LINKER)
	@echo "  LD    $@"
	@"$(CC)" $(ARCH_FLAGS) -Os -g -gdwarf-4 $(QUIET_LDFLAGS) $(LTO_LDFLAGS) \
		-T "$(CURDIR)/$(LINKER)" \
		-nostartfiles \
		-Xlinker --gc-sections \
		-Wl,-Map,"$(MAP)" \
		--specs=nano.specs --specs=nosys.specs \
		-o "$@" $(OBJS) -lm

# --- Nextor ROM -> User/nextor_rom.c --------------------------------------
# Two real file targets, chained by prerequisites: the ROM is rebuilt from
# the driver .asm files first, then embedded. Because User/nextor_rom.c is
# an ordinary file target and obj/User/nextor_rom.o depends on it, the
# build graph guarantees the embed finishes BEFORE the compile starts - in
# serial and under -jN alike. A single `make` after a driver.asm edit is
# therefore enough.
NEXTOR_PROBE_STAMP := $(OBJDIR)/.nextor-probes-$(if $(filter 1,$(PROBES)),on,off)
$(NEXTOR_PROBE_STAMP):
	@mkdir -p "$(OBJDIR)"
	@rm -f "$(OBJDIR)"/.nextor-probes-on "$(OBJDIR)"/.nextor-probes-off
	@touch "$@"

ifeq ($(NEXTOR_OWN_ROM),yes)
$(NEXTOR_ROM): $(NEXTOR_ASM) $(NEXTOR_DIR)/Makefile $(NEXTOR_PROBE_STAMP)
	@echo "  ROM   $(notdir $@)"
	@$(MAKE) -C "$(NEXTOR_DIR)" PROBES=$(PROBES)
else
# NEXTOR_ROM was overridden: this image is not ours to build, so declare
# no rule for it at all. `make` then requires the file to already exist
# and embeds it verbatim.
endif

# --nextor makes rom2c.py validate the image (AB header at bank 0,
# NEXTORv3_DRIVER signature + K_SIZE in the last bank) instead of
# embedding whatever bytes are there.
User/nextor_rom.c: $(NEXTOR_ROM) $(ROM2C)
	@echo "  EMBED $(notdir $@)"
	@python3 "$(ROM2C)" "$(NEXTOR_ROM)" "$@" \
	    "Nextor 3.0 beta 1 kernel + RISKYMSX2 USB driver (ASCII16 mapper)" \
	    --nextor

# Kept for compatibility; the real work happens through the file rules
# above. Forces a re-embed by invalidating the generated source.
regenerate_nextor_rom:
	@rm -f "$(CURDIR)/User/nextor_rom.c"
	@$(MAKE) User/nextor_rom.c

$(HEX): $(ELF)
	@echo "  HEX   $@"
	@"$(OBJCOPY)" -O ihex "$<" "$@"

$(LST): $(ELF)
	@echo "  LST   $@"
	@"$(OBJDUMP)" --all-headers --demangle --disassemble -M xw "$<" > "$@"

# --- Compile -----------------------------------------------------------------
$(OBJDIR)/%.o: %.c
	@echo "  CC    $<"
	@mkdir -p "$(dir $@)"
	@"$(CC)" $(CFLAGS_COMMON) $(INCDIRS) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" \
		-c -o "$@" "$<"

# Hand-written .S startup file (uses preprocessor, so -x assembler-with-cpp).
$(OBJDIR)/%.o: %.S
	@echo "  AS    $<"
	@mkdir -p "$(dir $@)"
	@"$(CC)" $(CFLAGS_COMMON) $(INCDIRS) -x assembler-with-cpp \
		-MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" \
		-c -o "$@" "$<"

-include $(DEPS)

# --- Housekeeping ------------------------------------------------------------
clean:
	@echo "  CLEAN $(OBJDIR)"
	@rm -rf $(OBJDIR)

rebuild: clean
	@$(MAKE) -j1 all

# --- Convenience targets -----------------------------------------------------
showsize: $(ELF)
	@"$(SIZE)" --format=berkeley "$<"
	@"$(SIZE)" --format=berkeley "$<" | tail -n 1 | awk \
		-v FSZ=$(FLASH_SIZE) -v RSZ=$(RAM_SIZE) '{ \
			flash = $$1 + $$2; \
			ram   = $$2 + $$3; \
			printf "  FLASH: %8d / %8d bytes  (%5.1f%% used, %8d left)\n", \
			       flash, FSZ, 100 * flash / FSZ, FSZ - flash; \
			printf "  RAM  : %8d / %8d bytes  (%5.1f%% used, %8d left)\n", \
			       ram, RSZ, 100 * ram / RSZ, RSZ - ram; \
		}'

size: showsize

list:
	@echo "CC       = $(CC)"
	@echo "QUIET    = $(QUIET) (mode=$(BUILD_MODE))"
	@"$(CC)" --version | head -1
	@echo "OCD      = $(RISKY_OCD)"
	@echo "OCD_CFG  = $(RISKY_OCD_CFG)"
	@echo "WLINK    = $(WLINK)"
	@echo "CHIP     = $(WLINK_CHIP)"
	@echo "ADDR     = $(WLINK_ADDR)"

# --- Flash --------------------------------------------------------------------
read:
	@"$(RISKY_OCD)" -f "$(RISKY_OCD_CFG)" \
		-c "init" \
		-c "reset halt" \
		-c "dump_image $(or $(OUT),dump.bin) $(or $(ADDR),0x08000000) $(or $(SIZE),4096)" \
		-c "shutdown"

erase:
	@"$(RISKY_OCD)" -f "$(RISKY_OCD_CFG)" \
		-c "init" \
		-c "reset halt" \
		-c "flash erase_sector wch_riscv.flash 0 last" \
		-c "shutdown"

flash prog: $(HEX)
ifneq ($(RISKY_FLASH),wlink)
	@echo "  OCD    $(HEX) -> wch_riscv.cpu @ $(WLINK_ADDR)"
	@"$(RISKY_OCD)" -f "$(RISKY_OCD_CFG)" \
		-c "init" \
		-c "reset halt" \
		-c "flash write_image erase $(CURDIR)/$(HEX) $(WLINK_ADDR)" \
		-c "reset run" \
		-c "shutdown"
else
	@echo "  WLINK  $(HEX) -> $(WLINK_CHIP) @ $(WLINK_ADDR)"
	@$(WLINK) --chip $(WLINK_CHIP) flash -e $(HEX)
endif
