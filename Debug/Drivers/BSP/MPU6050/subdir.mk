################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/MPU6050/inv_mpu.c \
../Drivers/BSP/MPU6050/inv_mpu_dmp_motion_driver.c \
../Drivers/BSP/MPU6050/mpu6050.c \
../Drivers/BSP/MPU6050/mpu6050_dmp.c 

OBJS += \
./Drivers/BSP/MPU6050/inv_mpu.o \
./Drivers/BSP/MPU6050/inv_mpu_dmp_motion_driver.o \
./Drivers/BSP/MPU6050/mpu6050.o \
./Drivers/BSP/MPU6050/mpu6050_dmp.o 

C_DEPS += \
./Drivers/BSP/MPU6050/inv_mpu.d \
./Drivers/BSP/MPU6050/inv_mpu_dmp_motion_driver.d \
./Drivers/BSP/MPU6050/mpu6050.d \
./Drivers/BSP/MPU6050/mpu6050_dmp.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/MPU6050/%.o Drivers/BSP/MPU6050/%.su Drivers/BSP/MPU6050/%.cyclo: ../Drivers/BSP/MPU6050/%.c Drivers/BSP/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -IC:/Users/MXQ11/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -IC:/Users/MXQ11/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.6/Drivers/STM32F1xx_HAL_Driver/Inc -IC:/Users/MXQ11/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.6/Drivers/CMSIS/Device/ST/STM32F1xx/Include -IC:/Users/MXQ11/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.6/Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-MPU6050

clean-Drivers-2f-BSP-2f-MPU6050:
	-$(RM) ./Drivers/BSP/MPU6050/inv_mpu.cyclo ./Drivers/BSP/MPU6050/inv_mpu.d ./Drivers/BSP/MPU6050/inv_mpu.o ./Drivers/BSP/MPU6050/inv_mpu.su ./Drivers/BSP/MPU6050/inv_mpu_dmp_motion_driver.cyclo ./Drivers/BSP/MPU6050/inv_mpu_dmp_motion_driver.d ./Drivers/BSP/MPU6050/inv_mpu_dmp_motion_driver.o ./Drivers/BSP/MPU6050/inv_mpu_dmp_motion_driver.su ./Drivers/BSP/MPU6050/mpu6050.cyclo ./Drivers/BSP/MPU6050/mpu6050.d ./Drivers/BSP/MPU6050/mpu6050.o ./Drivers/BSP/MPU6050/mpu6050.su ./Drivers/BSP/MPU6050/mpu6050_dmp.cyclo ./Drivers/BSP/MPU6050/mpu6050_dmp.d ./Drivers/BSP/MPU6050/mpu6050_dmp.o ./Drivers/BSP/MPU6050/mpu6050_dmp.su

.PHONY: clean-Drivers-2f-BSP-2f-MPU6050

