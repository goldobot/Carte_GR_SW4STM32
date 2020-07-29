################################################################################
# Used only by Goldo. Do not use!
################################################################################
PREFIX_GOLDO = ${TOPDIR}

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../goldobot/src/tasks/arms.cpp \
../goldobot/src/tasks/fpga.cpp \
../goldobot/src/tasks/heartbeat.cpp \
../goldobot/src/tasks/rttelemetry.cpp \
../goldobot/src/tasks/main.cpp \
../goldobot/src/tasks/propulsion.cpp \
../goldobot/src/tasks/uart_comm.cpp

OBJS += \
./goldobot/src/tasks/arms.o \
./goldobot/src/tasks/fpga.o \
./goldobot/src/tasks/heartbeat.o \
./goldobot/src/tasks/rttelemetry.o \
./goldobot/src/tasks/main.o \
./goldobot/src/tasks/propulsion.o \
./goldobot/src/tasks/uart_comm.o

CPP_DEPS += \
./goldobot/src/tasks/arms.d \
./goldobot/src/tasks/fpga.d \
./goldobot/src/tasks/heartbeat.d \
./goldobot/src/tasks/rttelemetry.d \
./goldobot/src/tasks/main.d \
./goldobot/src/tasks/propulsion.d \
./goldobot/src/tasks/uart_comm.d


# Each subdirectory must supply rules for building sources it contributes
goldobot/src/tasks/%.o: ../goldobot/src/tasks/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"${PREFIX_GOLDO}/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Include" -I"${PREFIX_GOLDO}/goldobot/include" -I"${PREFIX_GOLDO}/goldobot/platforms/os/freertos/include" ${GOLDOFLAGS} -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


