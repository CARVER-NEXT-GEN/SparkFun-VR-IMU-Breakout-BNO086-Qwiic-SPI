################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BNO055/BNO055.c \
../BNO055/BNO055_config.c 

OBJS += \
./BNO055/BNO055.o \
./BNO055/BNO055_config.o 

C_DEPS += \
./BNO055/BNO055.d \
./BNO055/BNO055_config.d 


# Each subdirectory must supply rules for building sources it contributes
BNO055/%.o BNO055/%.su BNO055/%.cyclo: ../BNO055/%.c BNO055/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I"/home/tim/uros_ws/H745ZI-Q/BNO055/Common" -I"/home/tim/uros_ws/H745ZI-Q/BNO055/Common/BNO086_SPI" -I"/home/tim/uros_ws/H745ZI-Q/BNO055/CM4/BNO055" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BNO055

clean-BNO055:
	-$(RM) ./BNO055/BNO055.cyclo ./BNO055/BNO055.d ./BNO055/BNO055.o ./BNO055/BNO055.su ./BNO055/BNO055_config.cyclo ./BNO055/BNO055_config.d ./BNO055/BNO055_config.o ./BNO055/BNO055_config.su

.PHONY: clean-BNO055

