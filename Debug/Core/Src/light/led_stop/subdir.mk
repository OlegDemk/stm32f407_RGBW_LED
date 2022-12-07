################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/light/led_stop/stop_light.c 

OBJS += \
./Core/Src/light/led_stop/stop_light.o 

C_DEPS += \
./Core/Src/light/led_stop/stop_light.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/light/led_stop/%.o Core/Src/light/led_stop/%.su: ../Core/Src/light/led_stop/%.c Core/Src/light/led_stop/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-light-2f-led_stop

clean-Core-2f-Src-2f-light-2f-led_stop:
	-$(RM) ./Core/Src/light/led_stop/stop_light.d ./Core/Src/light/led_stop/stop_light.o ./Core/Src/light/led_stop/stop_light.su

.PHONY: clean-Core-2f-Src-2f-light-2f-led_stop

