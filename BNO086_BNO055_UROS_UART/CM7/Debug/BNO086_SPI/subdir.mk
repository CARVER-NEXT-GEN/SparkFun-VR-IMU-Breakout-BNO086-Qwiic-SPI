################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BNO086_SPI/BNO086_SPI.c \
../BNO086_SPI/Quaternion.c 

OBJS += \
./BNO086_SPI/BNO086_SPI.o \
./BNO086_SPI/Quaternion.o 

C_DEPS += \
./BNO086_SPI/BNO086_SPI.d \
./BNO086_SPI/Quaternion.d 


# Each subdirectory must supply rules for building sources it contributes
BNO086_SPI/%.o BNO086_SPI/%.su BNO086_SPI/%.cyclo: ../BNO086_SPI/%.c BNO086_SPI/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I"/home/tim/uros_ws/H745ZI-Q/UROS/BNO086_H745/CM7/BNO086_SPI" -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/tim/uros_ws/H745ZI-Q/UROS/BNO086_H745/CM7/BNO086_SPI" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BNO086_SPI

clean-BNO086_SPI:
	-$(RM) ./BNO086_SPI/BNO086_SPI.cyclo ./BNO086_SPI/BNO086_SPI.d ./BNO086_SPI/BNO086_SPI.o ./BNO086_SPI/BNO086_SPI.su ./BNO086_SPI/Quaternion.cyclo ./BNO086_SPI/Quaternion.d ./BNO086_SPI/Quaternion.o ./BNO086_SPI/Quaternion.su

.PHONY: clean-BNO086_SPI

