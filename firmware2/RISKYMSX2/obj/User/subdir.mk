################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/cart.c \
../User/ch32v4x7_it.c \
../User/cli.c \
../User/emu2212.c \
../User/main.c \
../User/psram.c \
../User/scc.c \
../User/system_ch32v4x7.c 

C_DEPS += \
./User/cart.d \
./User/ch32v4x7_it.d \
./User/cli.d \
./User/emu2212.d \
./User/main.d \
./User/psram.d \
./User/scc.d \
./User/system_ch32v4x7.d 

OBJS += \
./User/cart.o \
./User/ch32v4x7_it.o \
./User/cli.o \
./User/emu2212.o \
./User/main.o \
./User/psram.o \
./User/scc.o \
./User/system_ch32v4x7.o 

DIR_OBJS += \
./User/*.o \

DIR_DEPS += \
./User/*.d \

DIR_EXPANDS += \
./User/*.271r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv32-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -gdwarf-4 -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Debug" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Core" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Peripheral/inc" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User/FATFS" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User/USB_Host" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

