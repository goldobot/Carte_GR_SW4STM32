add_library(freertos)

target_sources(freertos PRIVATE
  tasks.c
  )

target_include_directories(freertos PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F>
  )
