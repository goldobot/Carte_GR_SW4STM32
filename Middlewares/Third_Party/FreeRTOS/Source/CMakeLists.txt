add_library(middlewares_FreeRTOS OBJECT)

target_sources(middlewares_FreeRTOS PRIVATE
  croutine.c
  event_groups.c
  list.c
  queue.c
  stream_buffer.c
  tasks.c
  timers.c
  portable/GCC/ARM_CM4F/port.c
  portable/MemMang/heap_4.c
  )

target_include_directories(middlewares_FreeRTOS PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/portable/GCC/ARM_CM4F>
  )
