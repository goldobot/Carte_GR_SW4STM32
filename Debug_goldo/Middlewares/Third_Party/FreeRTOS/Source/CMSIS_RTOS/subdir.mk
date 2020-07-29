################################################################################
# Used only by Goldo. Do not use!
################################################################################
PREFIX_GOLDO = ${TOPDIR}

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"${PREFIX_GOLDO}/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Include" -I"${PREFIX_GOLDO}/goldobot/include" -I"${PREFIX_GOLDO}/goldobot/platforms/os/freertos/include"  -O2 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


