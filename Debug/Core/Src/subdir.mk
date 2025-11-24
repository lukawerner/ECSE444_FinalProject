################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/helper_functions.c \
../Core/Src/main.c \
../Core/Src/motor.c \
../Core/Src/stm32l4s5i_iot01.c \
../Core/Src/stm32l4s5i_iot01_qspi.c \
../Core/Src/stm32l4s5i_iot01_tsensor.c \
../Core/Src/stm32l4xx_hal_msp.c \
../Core/Src/stm32l4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32l4xx.c \
../Core/Src/wifi.c 

OBJS += \
./Core/Src/helper_functions.o \
./Core/Src/main.o \
./Core/Src/motor.o \
./Core/Src/stm32l4s5i_iot01.o \
./Core/Src/stm32l4s5i_iot01_qspi.o \
./Core/Src/stm32l4s5i_iot01_tsensor.o \
./Core/Src/stm32l4xx_hal_msp.o \
./Core/Src/stm32l4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32l4xx.o \
./Core/Src/wifi.o 

C_DEPS += \
./Core/Src/helper_functions.d \
./Core/Src/main.d \
./Core/Src/motor.d \
./Core/Src/stm32l4s5i_iot01.d \
./Core/Src/stm32l4s5i_iot01_qspi.d \
./Core/Src/stm32l4s5i_iot01_tsensor.d \
./Core/Src/stm32l4xx_hal_msp.d \
./Core/Src/stm32l4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32l4xx.d \
./Core/Src/wifi.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"/Users/tingyichen/STM32CubeIDE/workspace_1.19.0/ECSE_444_FinalProject/Drivers/Components" -I"/Users/tingyichen/STM32CubeIDE/workspace_1.19.0/ECSE_444_FinalProject/Drivers/Components/mx25r6435f" -I"/Users/tingyichen/STM32CubeIDE/workspace_1.19.0/ECSE_444_FinalProject/Drivers/Components/hts221" -I"/Users/tingyichen/STM32CubeIDE/workspace_1.19.0/ECSE_444_FinalProject/Drivers/Components/Common" -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/helper_functions.cyclo ./Core/Src/helper_functions.d ./Core/Src/helper_functions.o ./Core/Src/helper_functions.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/motor.cyclo ./Core/Src/motor.d ./Core/Src/motor.o ./Core/Src/motor.su ./Core/Src/stm32l4s5i_iot01.cyclo ./Core/Src/stm32l4s5i_iot01.d ./Core/Src/stm32l4s5i_iot01.o ./Core/Src/stm32l4s5i_iot01.su ./Core/Src/stm32l4s5i_iot01_qspi.cyclo ./Core/Src/stm32l4s5i_iot01_qspi.d ./Core/Src/stm32l4s5i_iot01_qspi.o ./Core/Src/stm32l4s5i_iot01_qspi.su ./Core/Src/stm32l4s5i_iot01_tsensor.cyclo ./Core/Src/stm32l4s5i_iot01_tsensor.d ./Core/Src/stm32l4s5i_iot01_tsensor.o ./Core/Src/stm32l4s5i_iot01_tsensor.su ./Core/Src/stm32l4xx_hal_msp.cyclo ./Core/Src/stm32l4xx_hal_msp.d ./Core/Src/stm32l4xx_hal_msp.o ./Core/Src/stm32l4xx_hal_msp.su ./Core/Src/stm32l4xx_it.cyclo ./Core/Src/stm32l4xx_it.d ./Core/Src/stm32l4xx_it.o ./Core/Src/stm32l4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32l4xx.cyclo ./Core/Src/system_stm32l4xx.d ./Core/Src/system_stm32l4xx.o ./Core/Src/system_stm32l4xx.su ./Core/Src/wifi.cyclo ./Core/Src/wifi.d ./Core/Src/wifi.o ./Core/Src/wifi.su

.PHONY: clean-Core-2f-Src

