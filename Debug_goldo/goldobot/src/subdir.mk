################################################################################
# Used only by Goldo. Do not use!
################################################################################
PREFIX_GOLDO = ${TOPDIR}

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../goldobot/src/comm_deserializer.cpp \
../goldobot/src/comm_serializer.cpp \
../goldobot/src/goldobot_main.cpp \
../goldobot/src/robot.cpp \
../goldobot/src/robot_simulator.cpp \
../goldobot/src/sequence_engine.cpp \
../goldobot/src/strategy_engine.cpp \
../goldobot/src/trajectory_planner.cpp 

OBJS += \
./goldobot/src/comm_deserializer.o \
./goldobot/src/comm_serializer.o \
./goldobot/src/goldobot_main.o \
./goldobot/src/robot.o \
./goldobot/src/robot_simulator.o \
./goldobot/src/sequence_engine.o \
./goldobot/src/strategy_engine.o \
./goldobot/src/trajectory_planner.o 

CPP_DEPS += \
./goldobot/src/comm_deserializer.d \
./goldobot/src/comm_serializer.d \
./goldobot/src/goldobot_main.d \
./goldobot/src/robot.d \
./goldobot/src/robot_simulator.d \
./goldobot/src/sequence_engine.d \
./goldobot/src/strategy_engine.d \
./goldobot/src/trajectory_planner.d 


# Each subdirectory must supply rules for building sources it contributes
goldobot/src/%.o: ../goldobot/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"${PREFIX_GOLDO}/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Include" -I"${PREFIX_GOLDO}/goldobot/include"  -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


