################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/IKS01A2/iks01a2_env_sensors.c \
../Drivers/BSP/IKS01A2/iks01a2_env_sensors_ex.c \
../Drivers/BSP/IKS01A2/iks01a2_motion_sensors.c \
../Drivers/BSP/IKS01A2/iks01a2_motion_sensors_ex.c 

OBJS += \
./Drivers/BSP/IKS01A2/iks01a2_env_sensors.o \
./Drivers/BSP/IKS01A2/iks01a2_env_sensors_ex.o \
./Drivers/BSP/IKS01A2/iks01a2_motion_sensors.o \
./Drivers/BSP/IKS01A2/iks01a2_motion_sensors_ex.o 

C_DEPS += \
./Drivers/BSP/IKS01A2/iks01a2_env_sensors.d \
./Drivers/BSP/IKS01A2/iks01a2_env_sensors_ex.d \
./Drivers/BSP/IKS01A2/iks01a2_motion_sensors.d \
./Drivers/BSP/IKS01A2/iks01a2_motion_sensors_ex.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/IKS01A2/%.o Drivers/BSP/IKS01A2/%.su Drivers/BSP/IKS01A2/%.cyclo: ../Drivers/BSP/IKS01A2/%.c Drivers/BSP/IKS01A2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Drivers/BSP/Components/lsm6dsl -I../Drivers/BSP/Components/lps22hb -I../Drivers/BSP/Components/lsm303agr -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/IKS01A2 -I../Drivers/BSP/Components/Common -I../X-CUBE-MEMS1/Target -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-IKS01A2

clean-Drivers-2f-BSP-2f-IKS01A2:
	-$(RM) ./Drivers/BSP/IKS01A2/iks01a2_env_sensors.cyclo ./Drivers/BSP/IKS01A2/iks01a2_env_sensors.d ./Drivers/BSP/IKS01A2/iks01a2_env_sensors.o ./Drivers/BSP/IKS01A2/iks01a2_env_sensors.su ./Drivers/BSP/IKS01A2/iks01a2_env_sensors_ex.cyclo ./Drivers/BSP/IKS01A2/iks01a2_env_sensors_ex.d ./Drivers/BSP/IKS01A2/iks01a2_env_sensors_ex.o ./Drivers/BSP/IKS01A2/iks01a2_env_sensors_ex.su ./Drivers/BSP/IKS01A2/iks01a2_motion_sensors.cyclo ./Drivers/BSP/IKS01A2/iks01a2_motion_sensors.d ./Drivers/BSP/IKS01A2/iks01a2_motion_sensors.o ./Drivers/BSP/IKS01A2/iks01a2_motion_sensors.su ./Drivers/BSP/IKS01A2/iks01a2_motion_sensors_ex.cyclo ./Drivers/BSP/IKS01A2/iks01a2_motion_sensors_ex.d ./Drivers/BSP/IKS01A2/iks01a2_motion_sensors_ex.o ./Drivers/BSP/IKS01A2/iks01a2_motion_sensors_ex.su

.PHONY: clean-Drivers-2f-BSP-2f-IKS01A2

