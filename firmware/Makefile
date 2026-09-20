# =============================================================================
# RISKYMSX2 firmware build (drop-in replacement for MounRiver Studio)
# =============================================================================
# Usage (from this directory, i.e. firmware/):
#
#   make            # incremental build -> obj/RISKYMSX2.{elf,hex,lst}
#                     # parallel build (-j8) is on by default; pass -jN to
#                     # override, or -j1 to force serial.
#   make -B         # force rebuild (all sources)
#   make clean      # rm -rf obj/   (nukes intermediates and outputs)
#   make rebuild    # clean + all
#   make size       # print RAM/Flash usage vs capacity (also auto-printed
#                     after every successful build)
#   make list       # show which CC/wlink/chip are in use
#   make flash      # program the chip with OpenOCD (WCH-Link E)
#   make prog       # alias for flash
#
# Toolchain:
#   Override RISKY_TC=<path-to-bin-dir> or TRI=<gcc-prefix>. Default points
#   at /opt/riscv-toolchain/RISC-V Embedded GCC15/bin (the WCH MRS toolchain).
#
# Flash tool:
#   wlink from the system's wch-link / minichlink package. Override with
#   RISKY_FLASH=<cmd> or change WLINK below.
#
# Chip:
#   CH32V4x7 (per board/.template MCU=CH32V467VET). Override with
#   WLINK_CHIP=<id> (e.g. CH32V30X is the upstream id WCH-Link uses for
#   the V4x family).
#
# Architecture (-march):
#   Auto-detected at parse time by trying each known WCH XW march string
#   against the active $(CC). Override with RISKY_ARCH="-march=... -mabi=
#   ilp32 -msmall-data-limit=8 -msave-restore".
#
# Notes:
#   * Source layout, includes, link script, and output filenames match the
#     auto-generated obj/makefile produced by MRS so the resulting ELF is
#     bit-equivalent to what MRS would have produced.
#   * SysClock / HSE_VALUE selection lives in User/system_ch32v4x7.c
#     (HSE_VALUE=25000000UL is the right value for RISKYMSX2's 25 MHz HSE).
# =============================================================================

PROJECT       := RISKYMSX2
LINKER        := Ld/Link.ld

# --- Memory capacity (mirrors Ld/Link.ld) -----------------------------------
# FLASH 576K; RAM region is 136K-1K = 135K (the linker script places RAM at
# 0x20000000+1024 and keeps the first 1 KiB out of the region). Used to turn
# the binutils size output into "bytes used / bytes free" in `showsize`.
FLASH_SIZE    := 589824      # 576 * 1024
RAM_SIZE      := 138240      # 135 * 1024

# --- Parallel builds --------------------------------------------------------
# Default to -j8 unless the user passes -jN (including -j1) on the command
# line. `MAKEOVERRIDES` holds command-line variable assignments; if the
# caller already specified one of the common parallel knobs, respect it.
# Override the default with RISKY_JOBS=N.
#
# Note: setting -j8 in MAKEFLAGS makes the rule apply recursively to any
# sub-makes this Makefile spawns (currently none). To override for one
# invocation: `make -j1 ...` or `make RISKY_JOBS=1`.
ifeq (,$(filter -j -j% --jobs,$(MAKEOVERRIDES)))
  RISKY_JOBS ?= 8
  MAKEFLAGS  += -j$(RISKY_JOBS)
endif

# --- Toolchain --------------------------------------------------------------
# Default: WCH RISC-V Embedded GCC 15.x installed at
#   /opt/riscv-toolchain/RISC-V Embedded GCC15/bin
# This is the same toolchain that MounRiver Studio (MRS) ships with, so the
# default build is byte-for-byte equivalent to the auto-generated
# obj/makefile. Override by setting RISKY_TC=<dir> and/or TRI=<prefix>.
#
# The default path contains a literal space ("RISC-V Embedded GCC15"),
# which GNU make's $(wildcard ...) cannot handle directly. We resolve the
# tool existence via $(shell test ...) and only pass the resolved paths
# to recipes, never through $(wildcard).
TOOLCHAIN_DIR ?= /opt/riscv-toolchain/RISC-V Embedded GCC15/bin
TRI           ?= riscv32-wch-elf
CC            := $(TOOLCHAIN_DIR)/$(TRI)-gcc
OBJCOPY       := $(TOOLCHAIN_DIR)/$(TRI)-objcopy
OBJDUMP       := $(TOOLCHAIN_DIR)/$(TRI)-objdump
SIZE          := $(TOOLCHAIN_DIR)/$(TRI)-size

# Architecture defaults are detected below from the active toolchain:
#   * The WCH MCS toolchain accepts "...imac_zba_zbb_zbc_zbs_..._xw".
#   * The xPack riscv-none-embed (GCC 8.2.0) accepts the same string with an
#     underscore for the WCH XW extension ONLY in newer subversions; the
#     older forms ("rv32imacxw" without the leading underscore on the suffix)
#     are accepted by every release and produce identical machine code for
#     the code in this project (no B/K/V instructions are emitted).
# Override with RISKY_ARCH=... to force.
ARCH_FLAGS    ?= $(RISKY_ARCH)

CFLAGS_COMMON  = $(ARCH_FLAGS) \
                 -fmax-errors=20 -Os -fmessage-length=0 \
                 -fsigned-char -ffunction-sections -fdata-sections -fno-common \
                 -Wunused -Wuninitialized \
                 -g -gdwarf-4 -std=gnu99

# --- Game base (PSRAM placement diagnostic) ----------------------------------
# Where inside the 8 MiB PSRAM window the served cart image starts, in
# whole MiB. 0 = image at PSRAM start (default). Valid: 0..7.
#
# The constant flows to cart.h as CART_GAME_BASE_MB via this -D; every
# mapper read path (C + asm) and the LOAD_ROM/CAT/XLOAD/DUMP write paths
# fold the same offset in, so the image moves as one block. Use it to
# test whether a game failure is PSRAM-address-dependent:
#
#     make rebuild GAME_BASE_MB=0 && make flash   # baseline
#     make rebuild GAME_BASE_MB=2 && make flash   # image at PSRAM+2MiB
#     make rebuild GAME_BASE_MB=4 && make flash
#
# NOTE: changing this requires a FULL rebuild (the constant is baked
# into asm `lui` immediates) - use `make rebuild`, not `make`.
GAME_BASE_MB ?= 0
CFLAGS_COMMON += -DCART_GAME_BASE_MB=$(GAME_BASE_MB)

# KONAMISCC SCC write-queue toggle (2026-09-19 isolation test).
#
# Compile-time switch for cart.c's RunKonamiSCC. Default 1 (queue
# enabled, normal SCC audio operation). Set to 0 to compile
# RunKonamiSCC as a byte-for-byte clone of RunKonamiNOSCC with the
# SCC_QueueWrite call stubbed out - the only diff between it and
# NOSCC is then the SCC_Init() called by Cart_SetMapper(). Use this
# to test whether a KONAMISCC freeze is in the cart-read handler
# (which would also affect NOSCC) or in the SCC queue / TIM4 path
# (which would NOT affect NOSCC).
#
#     make rebuild                   # default, SCC queue enabled
#     make rebuild KONAMISCC_SCC_QUEUE=0   # queue stubbed (isolation build)
#
# NOTE: changing this only forces a recompile of User/cart.c -
# `make` after the rebuild will pick it up (User/cart.o is the
# only file that uses the macro).
KONAMISCC_SCC_QUEUE ?= 1
CFLAGS_COMMON += -DKONAMISCC_SCC_QUEUE=$(KONAMISCC_SCC_QUEUE)

# --- Flash tool --------------------------------------------------------------
# CH32V4x7 (chip id 0xa6) is NOT supported by upstream `wlink` or `minichlink`
# (they report "Chip Type unknown [a6]" and refuse to program). The WCH-LinkE
# can still drive the SDI debug module — you just have to talk to it with the
# WCH fork of OpenOCD. The `wch-riscv.cfg` script next to the openocd binary
# wires up the `wlinke` adapter + `wch_riscv` target + SDI transport.
#
# `flash` defaults to the openocd path. Override on the command line:
#   make flash                     # openocd (default)
#   make flash RISKY_FLASH=wlink   # use the wlink command (chip must be in
#                                  # wlink's known-chips list; V4x is not)
#
# Override the openocd binary or its WCH config script directly:
#   RISKY_OCD=<path-to-openocd>        make flash
#   RISKY_OCD_CFG=<path-to-wch-riscv.cfg>   make flash

OCD           ?= $(shell command -v openocd 2>/dev/null)
RISKY_OCD     ?= /usr/share/MRS2/MRS-linux-x64/resources/app/resources/linux/components/WCH/OpenOCD/OpenOCD/bin/openocd
RISKY_OCD_CFG ?= $(dir $(RISKY_OCD))wch-riscv.cfg

WLINK         ?= wlink
WLINK_CHIP    ?= CH32V30X         # CH32V4x7 family is selected via CH32V30X id
# CH32V4x7 flash is aliased at 0x00000000 in the WCH openocd wch_riscv flash
# bank, but also visible at 0x08000000 through the boot-remap bit. The
# driver expects the offset that matches the bank base (0x0). Override
# WLINK_ADDR=0x08000000 only if you change the flash bank address.
WLINK_ADDR    ?= 0x00000000       # WCH openocd flash bank base

# --- Directories -------------------------------------------------------------
SRCDIRS        := Startup Debug Core Peripheral/src \
                  User User/FATFS User/USB_Host
INCDIRS        := -IDebug -ICore \
                  -IUser -IUser/FATFS -IUser/USB_Host \
                  -IPeripheral/inc -IStartup
MSX_SOFTWARE_DIR := $(CURDIR)/../MSXSoftware
ROM_LOADER_DIR   := $(MSX_SOFTWARE_DIR)/RomLoader
ROM_EMBED_SRC    := User/selector_rom.c
ROM_EMBED_DEPS   := $(MSX_SOFTWARE_DIR)/RomLoader/Makefile \
					$(MSX_SOFTWARE_DIR)/RomLoader/crt0.s \
					$(MSX_SOFTWARE_DIR)/RomLoader/romloader.c \
					$(CURDIR)/../tools/rom2c.py
OBJDIR         := obj

# --- Discover sources automatically ------------------------------------------
# (wildcard ...) means: when you add a new .c/.S, it just works, no list edit.)
SRC_C          := $(wildcard Startup/*.c  Startup/*.S \
                    Debug/*.c \
                    Core/*.c \
                    Peripheral/src/*.c \
                    User/*.c \
                    User/FATFS/*.c \
					User/USB_Host/*.c) $(ROM_EMBED_SRC)
SRC_C          := $(filter-out User/envtest_rom.c,$(SRC_C))
SRC_C          := $(sort $(SRC_C))

# Objects mirror source tree under $(OBJDIR)/<subdir>/<name>.o.
OBJ_C          := $(patsubst %.c,$(OBJDIR)/%.o,$(SRC_C))
OBJ_S          := $(patsubst %.S,$(OBJDIR)/%.o,$(OBJ_C))
OBJS           := $(OBJ_S)
DEPS           := $(OBJS:.o=.d)

# If User/PSRAM is pulled in later, also pick it up — kept as a no-op target
# so calling `make obj/User/psram.o` works without breaking the wildcard.
EXTRA_OBJS     :=

# --- Output files (match MRS' obj/makefile exactly) -----------------------
ELF            := $(OBJDIR)/$(PROJECT).elf
HEX            := $(OBJDIR)/$(PROJECT).hex
LST            := $(OBJDIR)/$(PROJECT).lst
MAP            := $(OBJDIR)/$(PROJECT).map

# --- Toolchain sanity check (runs once per build) ----------------------------
# GNU make's $(wildcard ...) can't match paths that contain literal spaces,
# so probe with $(shell test -x ...) instead.
ifeq ($(shell test -x '$(CC)' && echo OK),)
  $(error Toolchain not found or not executable: $(CC). \
          Set RISKY_TC=<path-to-bin-dir> or TRI=<gcc-prefix>, or install the \
          riscv-none-embed gcc.)
endif

# Auto-detect the right -march syntax for the XW extension on this CC.
# We try, in order:
#   1. The MRS-equivalent full march string (B/K/Zve64x/Zvl64b/Zvbb/XW).
#      Accepted by the WCH MCS toolchain (GCC 12+) that ships with MounRiver.
#   2. The same minus `zvbb` (which was split in upstream RISC-V and is
#      unknown to upstream binutils after GCC 12).
#   3. The bare `rv32imacxw` (no underscore, accepted by xPack 8.2.0).
# Each probe runs cc1 -E against /dev/null; success means the toolchain
# actually understands the march AND can preprocess it.
ARCH_TRY_LIST := \
  rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw \
  rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_xw \
  rv32imacxw \
  rv32imac_xw

# Pre-quote the toolchain path so $(shell) calls don't break on spaces.
# We use an absolute path to the gcc binary as a single shell word.
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
.PHONY: all clean rebuild flash prog size list read erase showsize

all: $(ELF) $(HEX) $(LST) showsize
	@echo "*** $(PROJECT) build complete ***"

# --- Link --------------------------------------------------------------------
# $(ELF) recipe reproduces obj/makefile's RISKYMSX2.elf target.
#   -march/mabi pulled from ARCH_FLAGS.
#   -T path is absolute so the makefile is location-independent.
#   --gc-sections matches MRS' --Xlinker --gc-sections invocation.
#   --specs=nano.specs / nosys.specs are WCH's default newlib configs.
$(ELF): $(OBJS) $(EXTRA_OBJS) $(LINKER)
	@echo "  LD    $@"
	@"$(CC)" $(ARCH_FLAGS) -Os -g -gdwarf-4 \
		-T "$(CURDIR)/$(LINKER)" \
		-nostartfiles \
		-Xlinker --gc-sections \
		-Wl,-Map,"$(MAP)" \
		--specs=nano.specs --specs=nosys.specs \
		-o "$@" $(OBJS) $(EXTRA_OBJS) -lm

$(ROM_EMBED_SRC): $(ROM_EMBED_DEPS)
	@echo "  EMBED $(notdir $@)"
	@"$(MAKE)" -C "$(ROM_LOADER_DIR)" embed

$(HEX): $(ELF)
	@echo "  HEX   $@"
	@"$(OBJCOPY)" -O ihex "$<" "$@"

$(LST): $(ELF)
	@echo "  LST   $@"
	@"$(OBJDUMP)" --all-headers --demangle --disassemble -M xw "$<" > "$@"

# --- Compile -----------------------------------------------------------------
# C sources. We mkdir the output subdir on first hit because some sources
# live several levels deep (User/FATFS/, User/USB_Host/).
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

# Pull in dependency files (.d) generated by -MMD. The conditional guards
# against the first build where no .d files exist yet.
-include $(DEPS)

# --- Housekeeping ------------------------------------------------------------
clean:
	@echo "  CLEAN $(OBJDIR)"
	@rm -rf $(OBJDIR)

rebuild: clean
	@$(MAKE) -j1 all

# --- Convenience targets -----------------------------------------------------
# Memory report: binutils `size` gives text/data/bss; for this single-region
# map, FLASH usage = text + data (initialized bytes living in flash) and RAM
# usage = data + bss. Note: the 2 KiB stack (Ld/Link.ld __stack_size) sits at
# the top of RAM and is NOT counted in bss.
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
	@"$(CC)" --version | head -1
	@echo "OCD      = $(RISKY_OCD)"
	@echo "OCD_CFG  = $(RISKY_OCD_CFG)"
	@echo "WLINK    = $(WLINK)"
	@echo "CHIP     = $(WLINK_CHIP)"
	@echo "ADDR     = $(WLINK_ADDR)"

# --- Flash --------------------------------------------------------------------
# The WCH-LinkE exposes the SDI debug module on CH32V4x7 (chip id 0xa6). The
# `wlink` and `minichlink` CLIs in this repo don't know that id, so they
# refuse to even initialize the link. The WCH-bundled OpenOCD does support
# it (via the `wlinke` adapter driver and the `wch_riscv` flash bank), so
# default `make flash` to OpenOCD and only fall back to wlink if explicitly
# asked (RISKY_FLASH=wlink).

# `make read OUT=dump.bin [SIZE=4096] [ADDR=0x08000000]` pulls raw bytes off
# the chip (useful for verifying what was just flashed or rescuing a
# bricked board). Default size = 4 KiB.
read:
	@"$(RISKY_OCD)" -f "$(RISKY_OCD_CFG)" \
		-c "init" \
		-c "reset halt" \
		-c "dump_image $(or $(OUT),dump.bin) $(or $(ADDR),0x08000000) $(or $(SIZE),4096)" \
		-c "shutdown"

# `make erase` mass-erases the code flash. The chip boots with empty
# memory and the BOOT-loader won't start; a follow-up `make flash` is
# needed to make the board do anything.
erase:
	@"$(RISKY_OCD)" -f "$(RISKY_OCD_CFG)" \
		-c "init" \
		-c "reset halt" \
		-c "flash erase_sector wch_riscv.flash 0 last" \
		-c "shutdown"

# The flash sequence:
#   1. openocd init
#   2. halt (so flash programming doesn't race the running bootloader)
#   3. flash write_image erase    (use openocd's erase-then-write; the
#                                  wch_riscv flash driver runs the right
#                                  chip-specific sequence under the hood)
#   4. reset run                   (re-attach and resume)
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
