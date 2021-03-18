add_library(goldobot_platform_board STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_callback_handler_task.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_dma.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_gpio.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_i2c.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_io_device_queue.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_io_device.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_spi.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_timer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_trace.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/hal_uart.cpp
  )

target_include_directories(goldobot_platform_board PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
  $<INSTALL_INTERFACE:include>)  
 
target_link_libraries(goldobot_platform_board
  goldobot_platform_os
  drivers_STM32F3xx_HAL_Driver
  )