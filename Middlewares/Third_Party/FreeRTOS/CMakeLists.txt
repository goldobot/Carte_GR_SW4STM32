add_library(middlewares_FreeRTOS STATIC)

target_sources(middlewares_FreeRTOS PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/croutine.c
  ${CMAKE_CURRENT_SOURCE_DIR}/event_groups.c
  ${CMAKE_CURRENT_SOURCE_DIR}/list.c
  ${CMAKE_CURRENT_SOURCE_DIR}/queue.c
  ${CMAKE_CURRENT_SOURCE_DIR}/stream_buffer.c
  ${CMAKE_CURRENT_SOURCE_DIR}/tasks.c
  ${CMAKE_CURRENT_SOURCE_DIR}/timers.c
  ${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F/port.c
  ${CMAKE_CURRENT_SOURCE_DIR}/portable/MemMang/heap_4.c
  ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_RTOS/cmsis_os.c
  )

target_include_directories(middlewares_FreeRTOS PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/CMSIS_RTOS>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F>
  )
  
target_link_libraries(middlewares_FreeRTOS
  drivers_CMSIS
  )

