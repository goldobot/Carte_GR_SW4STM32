add_library(goldobot STATIC)

target_sources(goldobot PRIVATE
  src/core/dynamixels_interface.cpp
  )

target_include_directories(goldobot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

target_link_libraries(goldobot PRIVATE freertos)