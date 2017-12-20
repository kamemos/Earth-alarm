################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/STM32F4-Discovery/subdir.mk
-include Drivers/Components/wm8994/subdir.mk
-include Drivers/Components/ts3510/subdir.mk
-include Drivers/Components/stmpe811/subdir.mk
-include Drivers/Components/stmpe1600/subdir.mk
-include Drivers/Components/st7789h2/subdir.mk
-include Drivers/Components/st7735/subdir.mk
-include Drivers/Components/s5k5cag/subdir.mk
-include Drivers/Components/ov2640/subdir.mk
-include Drivers/Components/otm8009a/subdir.mk
-include Drivers/Components/mfxstm32l152/subdir.mk
-include Drivers/Components/lsm303dlhc/subdir.mk
-include Drivers/Components/ls016b8uy/subdir.mk
-include Drivers/Components/lis3dsh/subdir.mk
-include Drivers/Components/lis302dl/subdir.mk
-include Drivers/Components/l3gd20/subdir.mk
-include Drivers/Components/ili9341/subdir.mk
-include Drivers/Components/ili9325/subdir.mk
-include Drivers/Components/ft6x06/subdir.mk
-include Drivers/Components/exc7200/subdir.mk
-include Drivers/Components/cs43l22/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Acceralo2.elf

# Tool invocations
Acceralo2.elf: $(OBJS) $(USER_OBJS) ../STM32F407VGTx_FLASH.ld
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -specs=nosys.specs -specs=nano.specs -T"../STM32F407VGTx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -o "Acceralo2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "Acceralo2.elf" "Acceralo2.bin"
	arm-none-eabi-size "Acceralo2.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
