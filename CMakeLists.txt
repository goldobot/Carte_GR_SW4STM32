cmake_minimum_required(VERSION 3.10)

project(Carte_GR VERSION 0.0 LANGUAGES C CXX ASM)

set(GOLDOBOT_PLATFORM_OS FreeRTOS)

include_directories(Inc)

execute_process(
  COMMAND git describe --all
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_BRANCH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
  COMMAND git describe --abbrev=8 --dirty --always
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_REF
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions("-DGOLDO_GIT_VERSION=\"${GIT_BRANCH} ${GIT_REF}\"")


add_subdirectory(Middlewares)
add_subdirectory(Drivers)
add_subdirectory(goldobot)

add_executable(firmware
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_hal_timebase_TIM.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32f3xx_it.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/syscalls.c
  ${CMAKE_CURRENT_SOURCE_DIR}/Src/system_stm32f3xx.c
  ${CMAKE_CURRENT_SOURCE_DIR}/goldobot/src/goldobot_main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/goldobot/platforms/nucleo_f303re_freertos_9_0/src/hal.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/startup/startup_stm32f303xe.s
  )
  
target_link_libraries(firmware
  drivers_CMSIS
  drivers_STM32F3xx_HAL_Driver
  middlewares_FreeRTOS
  goldobot
  )
  
target_compile_options(firmware PUBLIC
  -T ${CMAKE_CURRENT_SOURCE_DIR}/STM32F303RETx_FLASH.ld
  )
  
set_target_properties(firmware PROPERTIES
  SUFFIX ".elf"
  )
  
add_custom_command(TARGET firmware
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex "${CMAKE_CURRENT_BINARY_DIR}/firmware.elf" "${CMAKE_CURRENT_BINARY_DIR}/firmware.hex"
  COMMAND ${CMAKE_OBJCOPY} -O binary "${CMAKE_CURRENT_BINARY_DIR}/firmware.elf" "${CMAKE_CURRENT_BINARY_DIR}/firmware.bin"
  COMMAND ${CMAKE_SIZE_UTIL} "${CMAKE_CURRENT_BINARY_DIR}/firmware.elf"
  )
  
