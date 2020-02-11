

set(CMAKE_SYSTEM_NAME Generic) 


set(TOOLCHAIN_PATH C:/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.arm-none.win32_1.17.0.201812190825/tools/compiler)

# Set C and C++ compiler and linker
set(CMAKE_C_COMPILER ${TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc.exe)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PATH}/bin/arm-none-eabi-gcc.exe)
SET(CMAKE_C_COMPILER_FORCED)
SET(CMAKE_CXX_COMPILER_FORCED)
set(CMAKE_LINKER ${TOOLCHAIN_PATH}/bin/arm-none-eabi-ld.exe)

set(COMMON_FLAGS "-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=c99")
SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++14 -fno-exceptions -fno-rtti")
set(CMAKE_EXE_LINKER_FLAGS "${COMMON_FLAG} -specs=nosys.specs -specs=nano.specs -fno-exceptions -fno-rtti")