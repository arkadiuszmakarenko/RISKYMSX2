################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/core_riscv.c 

C_DEPS += \
./Core/core_riscv.d 

OBJS += \
./Core/core_riscv.o 

DIR_OBJS += \
./Core/*.o \

DIR_DEPS += \
./Core/*.d \

DIR_EXPANDS += \
./Core/*.271r.expand \


# Each subdirectory must supply rules for building sources it contributes
Core/%.o: ../Core/%.c
	@	riscv32-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -gdwarf-4 -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Debug" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Core" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Peripheral/inc" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User/FATFS" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User/USB_Host" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

