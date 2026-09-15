################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################
-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/USB_Host/subdir.mk
-include User/Host_UDisk/subdir.mk
-include User/subdir.mk
-include Udisk_Lib/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
HOST_Udisk.hex \

SECONDARY_LIST += \
HOST_Udisk.lst \

SECONDARY_SIZE += \
HOST_Udisk.siz \


# All Target
all: 
	$(MAKE) --no-print-directory main-build 

main-build: HOST_Udisk.elf secondary-outputs

# Tool invocations
HOST_Udisk.elf: $(OBJS) $(USER_OBJS_ESCAPE)
	@	riscv32-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -gdwarf-4 -T "/home/makaron/Repo/CH32V467/EVT/EXAM/SRC/Ld/Link.ld" -nostartfiles -Xlinker --gc-sections -L"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/Udisk_Lib" -Wl,-Map,"HOST_Udisk.map" --specs=nano.specs --specs=nosys.specs -o "HOST_Udisk.elf" $(OBJS) $(LIBS)
HOST_Udisk.hex: HOST_Udisk.elf
	@	riscv32-wch-elf-objcopy -O ihex "HOST_Udisk.elf" "HOST_Udisk.hex"
HOST_Udisk.lst: HOST_Udisk.elf
	@	riscv32-wch-elf-objdump --all-headers --demangle --disassemble -M xw "HOST_Udisk.elf" > "HOST_Udisk.lst"
HOST_Udisk.siz: HOST_Udisk.elf
	riscv32-wch-elf-size --format=berkeley "HOST_Udisk.elf"

# Other Targets
clean:
	-$(RM) $(DIR_OBJS) $(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE) HOST_Udisk.elf
	-$(RM) $(DIR_EXPANDS) $(CALLGRAPH_DOT)
	-$(RM) $(DIR_DEPS)
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets