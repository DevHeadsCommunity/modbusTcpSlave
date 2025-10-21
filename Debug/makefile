################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/modbus/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/httpServer/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/TFTP/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/SNTP/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/SNMP/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/MQTT/MQTTPacket/src/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/MQTT/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/DNS/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/DHCP6/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/DHCP/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Internet/AAC/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Ethernet/W6300/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Ethernet/W6100/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Ethernet/W5500/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Ethernet/W5300/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Ethernet/W5200/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Ethernet/W5100S/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Ethernet/W5100/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Ethernet/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Application/multicast/subdir.mk
-include Core/Src/ioLibrary_Driver-master/Application/loopback/subdir.mk
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


BUILD_ARTIFACT_NAME := modbusTcpSlave
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
modbusTcpSlave.elf \

MAP_FILES += \
modbusTcpSlave.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
modbusTcpSlave.list \


# All Target
all: main-build

# Main-build Target
main-build: modbusTcpSlave.elf secondary-outputs

# Tool invocations
modbusTcpSlave.elf modbusTcpSlave.map: $(OBJS) $(USER_OBJS) C:\Users\UMESH\Desktop\STM32\modbusTcpSlave\STM32F303K8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "modbusTcpSlave.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\UMESH\Desktop\STM32\modbusTcpSlave\STM32F303K8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="modbusTcpSlave.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

modbusTcpSlave.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "modbusTcpSlave.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) default.size.stdout modbusTcpSlave.elf modbusTcpSlave.list modbusTcpSlave.map
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
