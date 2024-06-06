################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DH22/Src/dht22.c 

OBJS += \
./DH22/Src/dht22.o 

C_DEPS += \
./DH22/Src/dht22.d 


# Each subdirectory must supply rules for building sources it contributes
DH22/Src/%.o DH22/Src/%.su DH22/Src/%.cyclo: ../DH22/Src/%.c DH22/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/NRF/Inc" -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/DH22/Inc" -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/RGB/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-DH22-2f-Src

clean-DH22-2f-Src:
	-$(RM) ./DH22/Src/dht22.cyclo ./DH22/Src/dht22.d ./DH22/Src/dht22.o ./DH22/Src/dht22.su

.PHONY: clean-DH22-2f-Src

