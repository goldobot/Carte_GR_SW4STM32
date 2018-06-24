################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../goldobot/src/core/low_pass_filter.cpp \
../goldobot/src/core/message_queue.cpp \
../goldobot/src/core/pid_controller.cpp \
../goldobot/src/core/trapezoidal_speed_profile.cpp 

OBJS += \
./goldobot/src/core/low_pass_filter.o \
./goldobot/src/core/message_queue.o \
./goldobot/src/core/pid_controller.o \
./goldobot/src/core/trapezoidal_speed_profile.o 

CPP_DEPS += \
./goldobot/src/core/low_pass_filter.d \
./goldobot/src/core/message_queue.d \
./goldobot/src/core/pid_controller.d \
./goldobot/src/core/trapezoidal_speed_profile.d 


# Each subdirectory must supply rules for building sources it contributes
goldobot/src/core/%.o: ../goldobot/src/core/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Inc" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/CMSIS/Include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/goldobot/include"  -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


