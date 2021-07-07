add_library(drivers_CMSIS INTERFACE)

target_include_directories(drivers_CMSIS INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Device/ST/STM32F3xx/Include>
  )

target_compile_definitions(drivers_CMSIS INTERFACE
  STM32F303xE
  )
