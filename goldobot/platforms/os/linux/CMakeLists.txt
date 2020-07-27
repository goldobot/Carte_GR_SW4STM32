add_library(goldobot_platform_os OBJECT)

target_include_directories(goldobot_platform_os PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_sources(goldobot_platform_os PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/message_queue.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/message_exchange.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/task.cpp
  )