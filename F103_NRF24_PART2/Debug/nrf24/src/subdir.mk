################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../nrf24/src/MY_NRF24.c 

OBJS += \
./nrf24/src/MY_NRF24.o 

C_DEPS += \
./nrf24/src/MY_NRF24.d 


# Each subdirectory must supply rules for building sources it contributes
nrf24/src/%.o: ../nrf24/src/%.c nrf24/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Giang/Documents/F103_NRF24_PART2/nrf24/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-nrf24-2f-src

clean-nrf24-2f-src:
	-$(RM) ./nrf24/src/MY_NRF24.d ./nrf24/src/MY_NRF24.o

.PHONY: clean-nrf24-2f-src

