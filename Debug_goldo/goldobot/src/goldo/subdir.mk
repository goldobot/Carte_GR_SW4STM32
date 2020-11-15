################################################################################
# Used only by Goldo. Do not use!
################################################################################
PREFIX_GOLDO = ${TOPDIR}

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../goldobot/src/goldo/goldobot_main.cpp \
../goldobot/src/goldo/robot.cpp \
../goldobot/src/goldo/robot_simulator.cpp \
../goldobot/src/goldo/sequence_engine.cpp

OBJS += \
./goldobot/src/goldo/goldobot_main.o \
./goldobot/src/goldo/robot.o \
./goldobot/src/goldo/robot_simulator.o \
./goldobot/src/goldo/sequence_engine.o

CPP_DEPS += \
./goldobot/src/goldo/goldobot_main.d \
./goldobot/src/goldo/robot.d \
./goldobot/src/goldo/robot_simulator.d \
./goldobot/src/goldo/sequence_engine.d


# Each subdirectory must supply rules for building sources it contributes
goldobot/src/goldo/%.o: ../goldobot/src/goldo/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"${PREFIX_GOLDO}/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc" -I"${PREFIX_GOLDO}/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/include" -I"${PREFIX_GOLDO}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"${PREFIX_GOLDO}/Drivers/CMSIS/Include" -I"${PREFIX_GOLDO}/goldobot/include"  -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fno-exceptions -fno-rtti -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


