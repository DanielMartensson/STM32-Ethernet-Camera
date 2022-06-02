################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/LwIP/src/netif/ppp/subdir.mk
-include Middlewares/Third_Party/LwIP/src/netif/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/ipv6/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/ipv4/subdir.mk
-include Middlewares/Third_Party/LwIP/src/core/subdir.mk
-include Middlewares/Third_Party/LwIP/src/apps/mqtt/subdir.mk
-include Middlewares/Third_Party/LwIP/src/api/subdir.mk
-include LWIP/Target/subdir.mk
-include LWIP/App/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/Components/dp83848/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/STM32_ILI9341_FSMC_16_Bit/subdir.mk
-include Core/Src/MCP4728/subdir.mk
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


BUILD_ARTIFACT_NAME := ETH OPAMP DCMI FSMC CAN ENCODER CS1000
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.elf \

MAP_FILES += \
ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.list \


# All Target
all: main-build

# Main-build Target
main-build: ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.elf secondary-outputs

# Tool invocations
ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.elf ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.map: $(OBJS) $(USER_OBJS) C:\Users\danie\Documents\STM32CubeIDE-Workspace\ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000\STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "ETH OPAMP DCMI FSMC CAN ENCODER CS1000.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\danie\Documents\STM32CubeIDE-Workspace\ETH OPAMP DCMI FSMC CAN ENCODER CS1000\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ETH OPAMP DCMI FSMC CAN ENCODER CS1000.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ETH OPAMP DCMI FSMC CAN ENCODER CS1000.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.elf ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.list ETH\ OPAMP\ DCMI\ FSMC\ CAN\ ENCODER\ CS1000.map ETH\\ OPAMP\\ DCMI\\ FSMC\\ CAN\\ ENCODER\\ CS1000.elf default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
