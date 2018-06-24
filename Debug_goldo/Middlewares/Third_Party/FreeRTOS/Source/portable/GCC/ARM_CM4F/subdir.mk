################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c 

OBJS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.o 

C_DEPS += \
./Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.o: ../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Inc" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/CMSIS/Include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/goldobot/include"  -O2 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


