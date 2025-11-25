################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/lsm303dlhc/lsm303dlhc.c 

OBJS += \
./Drivers/Components/lsm303dlhc/lsm303dlhc.o 

C_DEPS += \
./Drivers/Components/lsm303dlhc/lsm303dlhc.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/lsm303dlhc/%.o Drivers/Components/lsm303dlhc/%.su Drivers/Components/lsm303dlhc/%.cyclo: ../Drivers/Components/lsm303dlhc/%.c Drivers/Components/lsm303dlhc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"C:/Users/User/STM32CubeIDE/workspace_2.0.0/ECSE444_FinalProject/Drivers/Components" -I"C:/Users/User/STM32CubeIDE/workspace_2.0.0/ECSE444_FinalProject/Drivers/Components/mx25r6435f" -I"C:/Users/User/STM32CubeIDE/workspace_2.0.0/ECSE444_FinalProject/Drivers/Components/hts221" -I"C:/Users/User/STM32CubeIDE/workspace_2.0.0/ECSE444_FinalProject/Drivers/Components/Common" -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-lsm303dlhc

clean-Drivers-2f-Components-2f-lsm303dlhc:
	-$(RM) ./Drivers/Components/lsm303dlhc/lsm303dlhc.cyclo ./Drivers/Components/lsm303dlhc/lsm303dlhc.d ./Drivers/Components/lsm303dlhc/lsm303dlhc.o ./Drivers/Components/lsm303dlhc/lsm303dlhc.su

.PHONY: clean-Drivers-2f-Components-2f-lsm303dlhc

