################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RGB/Src/RGB.c 

OBJS += \
./RGB/Src/RGB.o 

C_DEPS += \
./RGB/Src/RGB.d 


# Each subdirectory must supply rules for building sources it contributes
RGB/Src/%.o RGB/Src/%.su RGB/Src/%.cyclo: ../RGB/Src/%.c RGB/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/NRF/Inc" -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/DH22/Inc" -I"/home/hicham/STM32CubeIDE/workspace_1.14.0/Slave1_NRF24L01_V2/RGB/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-RGB-2f-Src

clean-RGB-2f-Src:
	-$(RM) ./RGB/Src/RGB.cyclo ./RGB/Src/RGB.d ./RGB/Src/RGB.o ./RGB/Src/RGB.su

.PHONY: clean-RGB-2f-Src

