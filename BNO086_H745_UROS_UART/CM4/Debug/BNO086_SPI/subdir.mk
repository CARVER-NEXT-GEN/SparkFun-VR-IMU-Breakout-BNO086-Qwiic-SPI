################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BNO086_SPI/BNO086_SPI.c 

OBJS += \
./BNO086_SPI/BNO086_SPI.o 

C_DEPS += \
./BNO086_SPI/BNO086_SPI.d 


# Each subdirectory must supply rules for building sources it contributes
BNO086_SPI/%.o BNO086_SPI/%.su BNO086_SPI/%.cyclo: ../BNO086_SPI/%.c BNO086_SPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/tim/Downloads/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI-main/UROS/BNO086_H745/Common" -I"/home/tim/Downloads/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI-main/UROS/BNO086_H745/CM4/BNO086_SPI" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BNO086_SPI

clean-BNO086_SPI:
	-$(RM) ./BNO086_SPI/BNO086_SPI.cyclo ./BNO086_SPI/BNO086_SPI.d ./BNO086_SPI/BNO086_SPI.o ./BNO086_SPI/BNO086_SPI.su

.PHONY: clean-BNO086_SPI

