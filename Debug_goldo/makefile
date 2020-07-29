################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf
PATH := /opt/stm32/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.arm-none.linux64_1.15.0.201708311556/tools/compiler/bin:$(PATH)

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include goldobot/src/tasks/subdir.mk
-include goldobot/src/propulsion/subdir.mk
-include goldobot/src/core/subdir.mk
-include goldobot/src/subdir.mk
-include goldobot/platforms/nucleo_f303re_freertos_9_0/src/subdir.mk
-include goldobot/platforms/os/freertos/src/subdir.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

TOPDIR ?= ${shell pwd}/..
GIT_REF := $(shell git describe --abbrev=8 --dirty --always)
GIT_BRANCH := $(shell git describe --all)
GOLDOFLAGS = -DGOLDO_GIT_VERSION="$(GIT_BRANCH) $(GIT_REF)"

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: Carte_GR_SW4STM32.elf

test-Goldo:
	-@echo 'GOLDOFLAGS = ${GOLDOFLAGS}'

# Tool invocations
Carte_GR_SW4STM32.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU G++ Linker'
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -specs=nosys.specs -specs=nano.specs -u _printf_float -T"../STM32F303RETx_FLASH.ld" -Wl,-Map=output.map -Wl,--gc-sections -fno-exceptions -fno-rtti -o "Carte_GR_SW4STM32.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
#	-$(RM) *
	find . -name "*.o" | xargs rm -f
	find . -name "*.d" | xargs rm -f
	rm -f Carte_GR_SW4STM32.bin
	rm -f Carte_GR_SW4STM32.elf
	rm -f output.map
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "Carte_GR_SW4STM32.elf" "Carte_GR_SW4STM32.bin"
	arm-none-eabi-size "Carte_GR_SW4STM32.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
