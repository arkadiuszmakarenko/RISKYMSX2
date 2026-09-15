################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/Udisk_Lib/CH32V407UFI.c 

C_DEPS += \
./Udisk_Lib/CH32V407UFI.d 

OBJS += \
./Udisk_Lib/CH32V407UFI.o 

DIR_OBJS += \
./Udisk_Lib/*.o \

DIR_DEPS += \
./Udisk_Lib/*.d \

DIR_EXPANDS += \
./Udisk_Lib/*.271r.expand \


# Each subdirectory must supply rules for building sources it contributes
Udisk_Lib/CH32V407UFI.o: /home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/Udisk_Lib/CH32V407UFI.c
	@	riscv32-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -gdwarf-4 -I"/home/makaron/Repo/CH32V467/EVT/EXAM/SRC/Debug" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/SRC/Core" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/HOST_Udisk/User" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/SRC/Peripheral/inc" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/HOST_Udisk/User/USB_Host" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/HOST_Udisk/User/Host_UDisk" -I"/home/makaron/Repo/CH32V467/EVT/EXAM/USBHS/Host/Udisk_Lib" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

