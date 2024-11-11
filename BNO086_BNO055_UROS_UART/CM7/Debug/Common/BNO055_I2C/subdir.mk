################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/home/tim/Documents/GitHub/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI/BNO086_BNO055_UROS_UART/Common/BNO055_I2C/BNO055.c \
/home/tim/Documents/GitHub/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI/BNO086_BNO055_UROS_UART/Common/BNO055_I2C/BNO055_config.c 

OBJS += \
./Common/BNO055_I2C/BNO055.o \
./Common/BNO055_I2C/BNO055_config.o 

C_DEPS += \
./Common/BNO055_I2C/BNO055.d \
./Common/BNO055_I2C/BNO055_config.d 


# Each subdirectory must supply rules for building sources it contributes
Common/BNO055_I2C/BNO055.o: /home/tim/Documents/GitHub/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI/BNO086_BNO055_UROS_UART/Common/BNO055_I2C/BNO055.c Common/BNO055_I2C/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/tim/Documents/GitHub/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI/BNO086_BNO055_UROS_UART/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Common/BNO055_I2C/BNO055_config.o: /home/tim/Documents/GitHub/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI/BNO086_BNO055_UROS_UART/Common/BNO055_I2C/BNO055_config.c Common/BNO055_I2C/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DCORE_CM7 -DUSE_HAL_DRIVER -DSTM32H745xx -c -I../Core/Inc -I../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Middlewares/Third_Party/FreeRTOS/Source/include -I../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/tim/Documents/GitHub/SparkFun-VR-IMU-Breakout-BNO086-Qwiic-SPI/BNO086_BNO055_UROS_UART/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Common-2f-BNO055_I2C

clean-Common-2f-BNO055_I2C:
	-$(RM) ./Common/BNO055_I2C/BNO055.cyclo ./Common/BNO055_I2C/BNO055.d ./Common/BNO055_I2C/BNO055.o ./Common/BNO055_I2C/BNO055.su ./Common/BNO055_I2C/BNO055_config.cyclo ./Common/BNO055_I2C/BNO055_config.d ./Common/BNO055_I2C/BNO055_config.o ./Common/BNO055_I2C/BNO055_config.su

.PHONY: clean-Common-2f-BNO055_I2C

