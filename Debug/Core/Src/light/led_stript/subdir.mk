################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/light/led_stript/ARGB.c \
../Core/Src/light/led_stript/control.c 

OBJS += \
./Core/Src/light/led_stript/ARGB.o \
./Core/Src/light/led_stript/control.o 

C_DEPS += \
./Core/Src/light/led_stript/ARGB.d \
./Core/Src/light/led_stript/control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/light/led_stript/%.o Core/Src/light/led_stript/%.su: ../Core/Src/light/led_stript/%.c Core/Src/light/led_stript/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-light-2f-led_stript

clean-Core-2f-Src-2f-light-2f-led_stript:
	-$(RM) ./Core/Src/light/led_stript/ARGB.d ./Core/Src/light/led_stript/ARGB.o ./Core/Src/light/led_stript/ARGB.su ./Core/Src/light/led_stript/control.d ./Core/Src/light/led_stript/control.o ./Core/Src/light/led_stript/control.su

.PHONY: clean-Core-2f-Src-2f-light-2f-led_stript

