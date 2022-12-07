################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/microSD/file_system.c \
../Core/Src/microSD/sd_spi.c 

OBJS += \
./Core/Src/microSD/file_system.o \
./Core/Src/microSD/sd_spi.o 

C_DEPS += \
./Core/Src/microSD/file_system.d \
./Core/Src/microSD/sd_spi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/microSD/%.o Core/Src/microSD/%.su: ../Core/Src/microSD/%.c Core/Src/microSD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-microSD

clean-Core-2f-Src-2f-microSD:
	-$(RM) ./Core/Src/microSD/file_system.d ./Core/Src/microSD/file_system.o ./Core/Src/microSD/file_system.su ./Core/Src/microSD/sd_spi.d ./Core/Src/microSD/sd_spi.o ./Core/Src/microSD/sd_spi.su

.PHONY: clean-Core-2f-Src-2f-microSD

