################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/ls016b8uy/ls016b8uy.c 

OBJS += \
./Drivers/Components/ls016b8uy/ls016b8uy.o 

C_DEPS += \
./Drivers/Components/ls016b8uy/ls016b8uy.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/ls016b8uy/%.o Drivers/Components/ls016b8uy/%.su Drivers/Components/ls016b8uy/%.cyclo: ../Drivers/Components/ls016b8uy/%.c Drivers/Components/ls016b8uy/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"C:/Users/ASUS/Desktop/ECSE444_FinalProject/Drivers/Components" -I"C:/Users/ASUS/Desktop/ECSE444_FinalProject/Drivers/Components/mx25r6435f" -I"C:/Users/ASUS/Desktop/ECSE444_FinalProject/Drivers/Components/hts221" -I"C:/Users/ASUS/Desktop/ECSE444_FinalProject/Drivers/Components/Common" -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-ls016b8uy

clean-Drivers-2f-Components-2f-ls016b8uy:
	-$(RM) ./Drivers/Components/ls016b8uy/ls016b8uy.cyclo ./Drivers/Components/ls016b8uy/ls016b8uy.d ./Drivers/Components/ls016b8uy/ls016b8uy.o ./Drivers/Components/ls016b8uy/ls016b8uy.su

.PHONY: clean-Drivers-2f-Components-2f-ls016b8uy

