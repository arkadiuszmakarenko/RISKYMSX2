################################################################################
# MRS Version: 2.5.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripheral/src/ch32v4x7_adc.c \
../Peripheral/src/ch32v4x7_argb.c \
../Peripheral/src/ch32v4x7_bkp.c \
../Peripheral/src/ch32v4x7_can.c \
../Peripheral/src/ch32v4x7_crc.c \
../Peripheral/src/ch32v4x7_dac.c \
../Peripheral/src/ch32v4x7_dbgmcu.c \
../Peripheral/src/ch32v4x7_dma.c \
../Peripheral/src/ch32v4x7_dvp.c \
../Peripheral/src/ch32v4x7_eth.c \
../Peripheral/src/ch32v4x7_exti.c \
../Peripheral/src/ch32v4x7_flash.c \
../Peripheral/src/ch32v4x7_fsmc.c \
../Peripheral/src/ch32v4x7_gpio.c \
../Peripheral/src/ch32v4x7_i2c.c \
../Peripheral/src/ch32v4x7_i3c.c \
../Peripheral/src/ch32v4x7_iwdg.c \
../Peripheral/src/ch32v4x7_ltdc.c \
../Peripheral/src/ch32v4x7_opa.c \
../Peripheral/src/ch32v4x7_psram.c \
../Peripheral/src/ch32v4x7_pwr.c \
../Peripheral/src/ch32v4x7_rcc.c \
../Peripheral/src/ch32v4x7_rng.c \
../Peripheral/src/ch32v4x7_rtc.c \
../Peripheral/src/ch32v4x7_sdio.c \
../Peripheral/src/ch32v4x7_spi.c \
../Peripheral/src/ch32v4x7_tim.c \
../Peripheral/src/ch32v4x7_usart.c \
../Peripheral/src/ch32v4x7_wwdg.c 

C_DEPS += \
./Peripheral/src/ch32v4x7_adc.d \
./Peripheral/src/ch32v4x7_argb.d \
./Peripheral/src/ch32v4x7_bkp.d \
./Peripheral/src/ch32v4x7_can.d \
./Peripheral/src/ch32v4x7_crc.d \
./Peripheral/src/ch32v4x7_dac.d \
./Peripheral/src/ch32v4x7_dbgmcu.d \
./Peripheral/src/ch32v4x7_dma.d \
./Peripheral/src/ch32v4x7_dvp.d \
./Peripheral/src/ch32v4x7_eth.d \
./Peripheral/src/ch32v4x7_exti.d \
./Peripheral/src/ch32v4x7_flash.d \
./Peripheral/src/ch32v4x7_fsmc.d \
./Peripheral/src/ch32v4x7_gpio.d \
./Peripheral/src/ch32v4x7_i2c.d \
./Peripheral/src/ch32v4x7_i3c.d \
./Peripheral/src/ch32v4x7_iwdg.d \
./Peripheral/src/ch32v4x7_ltdc.d \
./Peripheral/src/ch32v4x7_opa.d \
./Peripheral/src/ch32v4x7_psram.d \
./Peripheral/src/ch32v4x7_pwr.d \
./Peripheral/src/ch32v4x7_rcc.d \
./Peripheral/src/ch32v4x7_rng.d \
./Peripheral/src/ch32v4x7_rtc.d \
./Peripheral/src/ch32v4x7_sdio.d \
./Peripheral/src/ch32v4x7_spi.d \
./Peripheral/src/ch32v4x7_tim.d \
./Peripheral/src/ch32v4x7_usart.d \
./Peripheral/src/ch32v4x7_wwdg.d 

OBJS += \
./Peripheral/src/ch32v4x7_adc.o \
./Peripheral/src/ch32v4x7_argb.o \
./Peripheral/src/ch32v4x7_bkp.o \
./Peripheral/src/ch32v4x7_can.o \
./Peripheral/src/ch32v4x7_crc.o \
./Peripheral/src/ch32v4x7_dac.o \
./Peripheral/src/ch32v4x7_dbgmcu.o \
./Peripheral/src/ch32v4x7_dma.o \
./Peripheral/src/ch32v4x7_dvp.o \
./Peripheral/src/ch32v4x7_eth.o \
./Peripheral/src/ch32v4x7_exti.o \
./Peripheral/src/ch32v4x7_flash.o \
./Peripheral/src/ch32v4x7_fsmc.o \
./Peripheral/src/ch32v4x7_gpio.o \
./Peripheral/src/ch32v4x7_i2c.o \
./Peripheral/src/ch32v4x7_i3c.o \
./Peripheral/src/ch32v4x7_iwdg.o \
./Peripheral/src/ch32v4x7_ltdc.o \
./Peripheral/src/ch32v4x7_opa.o \
./Peripheral/src/ch32v4x7_psram.o \
./Peripheral/src/ch32v4x7_pwr.o \
./Peripheral/src/ch32v4x7_rcc.o \
./Peripheral/src/ch32v4x7_rng.o \
./Peripheral/src/ch32v4x7_rtc.o \
./Peripheral/src/ch32v4x7_sdio.o \
./Peripheral/src/ch32v4x7_spi.o \
./Peripheral/src/ch32v4x7_tim.o \
./Peripheral/src/ch32v4x7_usart.o \
./Peripheral/src/ch32v4x7_wwdg.o 

DIR_OBJS += \
./Peripheral/src/*.o \

DIR_DEPS += \
./Peripheral/src/*.d \

DIR_EXPANDS += \
./Peripheral/src/*.271r.expand \


# Each subdirectory must supply rules for building sources it contributes
Peripheral/src/%.o: ../Peripheral/src/%.c
	@	riscv32-wch-elf-gcc -march=rv32imac_zba_zbb_zbc_zbs_zve64x_zvl64b_zvbb_xw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -gdwarf-4 -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Debug" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Core" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/Peripheral/inc" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User/FATFS" -I"/home/makaron/Repo/RISKYMSX2/firmware2/RISKYMSX2/User/USB_Host" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

