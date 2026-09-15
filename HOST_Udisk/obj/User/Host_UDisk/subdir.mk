################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/Host_UDisk/UDisk_Func_CreatDir.c \
../User/Host_UDisk/UDisk_Func_LongName.c \
../User/Host_UDisk/UDisk_HW.c \
../User/Host_UDisk/Udisk_Func_BasicOp.c 

C_DEPS += \
./User/Host_UDisk/UDisk_Func_CreatDir.d \
./User/Host_UDisk/UDisk_Func_LongName.d \
./User/Host_UDisk/UDisk_HW.d \
./User/Host_UDisk/Udisk_Func_BasicOp.d 

OBJS += \
./User/Host_UDisk/UDisk_Func_CreatDir.o \
./User/Host_UDisk/UDisk_Func_LongName.o \
./User/Host_UDisk/UDisk_HW.o \
./User/Host_UDisk/Udisk_Func_BasicOp.o 

DIR_OBJS += \
./User/Host_UDisk/*.o \

DIR_DEPS += \
./User/Host_UDisk/*.d \

DIR_EXPANDS += \
./User/Host_UDisk/*.271r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/Host_UDisk/%.o: ../User/Host_UDisk/%.c
	@	riscv32-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -gdwarf-4 -I"/home/makaron/Repo/CH32V467/EVT/EXAM/SRC/Debug" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/SRC/Core" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/HOST_Udisk/User" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/SRC/Peripheral/inc" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/HOST_Udisk/User/USB_Host" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/HOST_Udisk/User/Host_UDisk" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/Udisk_Lib" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

