################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/FATFS/diskio.c \
../User/FATFS/ff.c \
../User/FATFS/ffsystem.c \
../User/FATFS/ffunicode.c \
../User/FATFS/usb_disk.c 

C_DEPS += \
./User/FATFS/diskio.d \
./User/FATFS/ff.d \
./User/FATFS/ffsystem.d \
./User/FATFS/ffunicode.d \
./User/FATFS/usb_disk.d 

OBJS += \
./User/FATFS/diskio.o \
./User/FATFS/ff.o \
./User/FATFS/ffsystem.o \
./User/FATFS/ffunicode.o \
./User/FATFS/usb_disk.o 

DIR_OBJS += \
./User/FATFS/*.o \

DIR_DEPS += \
./User/FATFS/*.d \

DIR_EXPANDS += \
./User/FATFS/*.271r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/FATFS/%.o: ../User/FATFS/%.c
	@	riscv32-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -gdwarf-4 -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Debug" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Core" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Peripheral/inc" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User/FATFS" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User/USB_Host" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

