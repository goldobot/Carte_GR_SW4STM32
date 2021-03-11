add_library(goldobot_platform_os STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/platform/config.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/platform/task.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/task.cpp
  )

target_include_directories(goldobot_platform_os PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)  
 
target_link_libraries(goldobot_platform_os
  drivers_CMSIS
  drivers_STM32F3xx_HAL_Driver
  middlewares_FreeRTOS
  )