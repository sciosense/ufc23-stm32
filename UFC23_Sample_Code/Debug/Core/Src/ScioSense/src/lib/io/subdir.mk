################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ScioSense/src/lib/io/ScioSense_IOInterface_STM32.c 

C_DEPS += \
./Core/Src/ScioSense/src/lib/io/ScioSense_IOInterface_STM32.d 

OBJS += \
./Core/Src/ScioSense/src/lib/io/ScioSense_IOInterface_STM32.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ScioSense/src/lib/io/%.o Core/Src/ScioSense/src/lib/io/%.su Core/Src/ScioSense/src/lib/io/%.cyclo: ../Core/Src/ScioSense/src/lib/io/%.c Core/Src/ScioSense/src/lib/io/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U385xx -c -I../Core/Inc -I../Drivers/STM32U3xx_HAL_Driver/Inc -I../Drivers/STM32U3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ScioSense-2f-src-2f-lib-2f-io

clean-Core-2f-Src-2f-ScioSense-2f-src-2f-lib-2f-io:
	-$(RM) ./Core/Src/ScioSense/src/lib/io/ScioSense_IOInterface_STM32.cyclo ./Core/Src/ScioSense/src/lib/io/ScioSense_IOInterface_STM32.d ./Core/Src/ScioSense/src/lib/io/ScioSense_IOInterface_STM32.o ./Core/Src/ScioSense/src/lib/io/ScioSense_IOInterface_STM32.su

.PHONY: clean-Core-2f-Src-2f-ScioSense-2f-src-2f-lib-2f-io

