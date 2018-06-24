################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../goldobot/platforms/nucleo_f303re_freertos_9_0/src/hal.cpp 

OBJS += \
./goldobot/platforms/nucleo_f303re_freertos_9_0/src/hal.o 

CPP_DEPS += \
./goldobot/platforms/nucleo_f303re_freertos_9_0/src/hal.d 


# Each subdirectory must supply rules for building sources it contributes
goldobot/platforms/nucleo_f303re_freertos_9_0/src/%.o: ../goldobot/platforms/nucleo_f303re_freertos_9_0/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Inc" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/CMSIS/Include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/goldobot/include"  -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


