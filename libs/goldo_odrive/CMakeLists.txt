cmake_minimum_required(VERSION 3.8)

project(goldo_odrive LANGUAGES CXX)

add_library(goldo_odrive STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/odrive/odrive_client.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/odrive/odrive_stream_parser.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/odrive/odrive_stream_writer.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/odrive/odrive_utils.hpp

  ${CMAKE_CURRENT_SOURCE_DIR}/src/odrive_client.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/odrive_stream_parser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/odrive_stream_writer.cpp
  )

target_include_directories(goldo_odrive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_link_libraries(goldo_odrive PUBLIC
  goldo_core
  )

if (GOLDO_BUILD_TESTS)
  add_subdirectory(tests)
endif ()