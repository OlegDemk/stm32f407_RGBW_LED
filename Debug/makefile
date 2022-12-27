################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_HOST/Target/subdir.mk
-include USB_HOST/App/subdir.mk
-include Middlewares/Third_Party/FatFs/src/option/subdir.mk
-include Middlewares/Third_Party/FatFs/src/subdir.mk
-include Middlewares/ST/STM32_USB_Host_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Src/subdir.mk
-include FATFS/Target/subdir.mk
-include FATFS/App/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/queue/subdir.mk
-include Core/Src/microSD/subdir.mk
-include Core/Src/light/led_stript/subdir.mk
-include Core/Src/light/led_stop/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := lights_RGBW_PWM_DMA_two_chanels2_WORKS7
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
lights_RGBW_PWM_DMA_two_chanels2_WORKS7.elf \

MAP_FILES += \
lights_RGBW_PWM_DMA_two_chanels2_WORKS7.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
lights_RGBW_PWM_DMA_two_chanels2_WORKS7.list \


# All Target
all: main-build

# Main-build Target
main-build: lights_RGBW_PWM_DMA_two_chanels2_WORKS7.elf secondary-outputs

# Tool invocations
lights_RGBW_PWM_DMA_two_chanels2_WORKS7.elf lights_RGBW_PWM_DMA_two_chanels2_WORKS7.map: $(OBJS) $(USER_OBJS) /home/odemki/my_files/work/work_projects/1.Current/9.Valeo_CES/12.stm32f407vg_LEDs/STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "lights_RGBW_PWM_DMA_two_chanels2_WORKS7.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/odemki/my_files/work/work_projects/1.Current/9.Valeo_CES/12.stm32f407vg_LEDs/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="lights_RGBW_PWM_DMA_two_chanels2_WORKS7.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

lights_RGBW_PWM_DMA_two_chanels2_WORKS7.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "lights_RGBW_PWM_DMA_two_chanels2_WORKS7.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout lights_RGBW_PWM_DMA_two_chanels2_WORKS7.elf lights_RGBW_PWM_DMA_two_chanels2_WORKS7.list lights_RGBW_PWM_DMA_two_chanels2_WORKS7.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
