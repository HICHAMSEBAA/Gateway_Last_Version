################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NRF/Src/MY_NRF24.c 

OBJS += \
./NRF/Src/MY_NRF24.o 

C_DEPS += \
./NRF/Src/MY_NRF24.d 


# Each subdirectory must supply rules for building sources it contributes
NRF/Src/%.o NRF/Src/%.su NRF/Src/%.cyclo: ../NRF/Src/%.c NRF/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/NRF/Inc" -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/DH22/Inc" -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/RGB/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-NRF-2f-Src

clean-NRF-2f-Src:
	-$(RM) ./NRF/Src/MY_NRF24.cyclo ./NRF/Src/MY_NRF24.d ./NRF/Src/MY_NRF24.o ./NRF/Src/MY_NRF24.su

.PHONY: clean-NRF-2f-Src

