################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/st7735/st7735.c 

OBJS += \
./Drivers/Components/st7735/st7735.o 

C_DEPS += \
./Drivers/Components/st7735/st7735.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/st7735/%.o: ../Drivers/Components/st7735/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/Benjamin/workspace/Acceralo2/Inc" -I"C:/Users/Benjamin/workspace/Acceralo2/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Benjamin/workspace/Acceralo2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Benjamin/workspace/Acceralo2/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Benjamin/workspace/Acceralo2/Drivers/CMSIS/Include" -I"C:/Users/Benjamin/workspace/Acceralo2/Inc"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


