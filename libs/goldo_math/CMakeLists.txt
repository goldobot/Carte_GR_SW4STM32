cmake_minimum_required(VERSION 3.8)

project(goldo_math LANGUAGES CXX)

add_library(goldo_math STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/core/derivative_filter.hpp

  ${CMAKE_CURRENT_SOURCE_DIR}/src/derivative_filter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/math_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pid_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/trajectory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/trapezoidal_speed_profile.cpp

  )

target_include_directories(goldo_math PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

#add_subdirectory(tests)