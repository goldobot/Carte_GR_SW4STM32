################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../goldobot/src/tasks/arms.cpp \
../goldobot/src/tasks/fpga.cpp \
../goldobot/src/tasks/gyro.cpp \
../goldobot/src/tasks/heartbeat.cpp \
../goldobot/src/tasks/main.cpp \
../goldobot/src/tasks/propulsion.cpp \
../goldobot/src/tasks/task.cpp \
../goldobot/src/tasks/uart_comm.cpp 

OBJS += \
./goldobot/src/tasks/arms.o \
./goldobot/src/tasks/fpga.o \
./goldobot/src/tasks/gyro.o \
./goldobot/src/tasks/heartbeat.o \
./goldobot/src/tasks/main.o \
./goldobot/src/tasks/propulsion.o \
./goldobot/src/tasks/task.o \
./goldobot/src/tasks/uart_comm.o 

CPP_DEPS += \
./goldobot/src/tasks/arms.d \
./goldobot/src/tasks/fpga.d \
./goldobot/src/tasks/gyro.d \
./goldobot/src/tasks/heartbeat.d \
./goldobot/src/tasks/main.d \
./goldobot/src/tasks/propulsion.d \
./goldobot/src/tasks/task.d \
./goldobot/src/tasks/uart_comm.d 


# Each subdirectory must supply rules for building sources it contributes
goldobot/src/tasks/%.o: ../goldobot/src/tasks/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Inc" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/STM32F3xx_HAL_Driver/Inc" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/Drivers/CMSIS/Include" -I"/home/jlouis/src/robotik/coupe2018/goldorak/soft/gros_robot_postcoupe/Carte_GR_SW4STM32/goldobot/include"  -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


