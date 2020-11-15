################################################################################
# Used only by Goldo. Do not use!
################################################################################
PREFIX_GOLDO = ${TOPDIR}

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../goldobot/src/goldo/tasks/arms.cpp \
../goldobot/src/goldo/tasks/fpga.cpp \
../goldobot/src/goldo/tasks/heartbeat.cpp \
../goldobot/src/goldo/tasks/rttelemetry.cpp \
../goldobot/src/goldo/tasks/main.cpp \
../goldobot/src/goldo/tasks/propulsion.cpp \
../goldobot/src/goldo/tasks/task.cpp \
../goldobot/src/goldo/tasks/uart_comm.cpp

OBJS += \
./goldobot/src/goldo/tasks/arms.o \
./goldobot/src/goldo/tasks/fpga.o \
./goldobot/src/goldo/tasks/heartbeat.o \
./goldobot/src/goldo/tasks/rttelemetry.o \
./goldobot/src/goldo/tasks/main.o \
./goldobot/src/goldo/tasks/propulsion.o \
./goldobot/src/goldo/tasks/task.o \
./goldobot/src/goldo/tasks/uart_comm.o

CPP_DEPS += \
./goldobot/src/goldo/tasks/arms.d \
./goldobot/src/goldo/tasks/fpga.d \
./goldobot/src/goldo/tasks/heartbeat.d \
./goldobot/src/goldo/tasks/rttelemetry.d \
./goldobot/src/goldo/tasks/main.d \
./goldobot/src/goldo/tasks/propulsion.d \
./goldobot/src/goldo/tasks/task.d \
./goldobot/src/goldo/tasks/uart_comm.d


# Each subdirectory must supply rules for building sources it contributes
goldobot/src/goldo/tasks/%.o: ../goldobot/src/goldo/tasks/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"${PREFIX_GOLDO}/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Include" -I"${PREFIX_GOLDO}/goldobot/include" ${GOLDOFLAGS} -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


