cmake_minimum_required(VERSION 3.8)

project(goldobot LANGUAGES CXX)

add_subdirectory(platforms)

add_library(goldobot OBJECT)


target_sources(goldobot PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/sequence_engine.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/hal.hpp

  )


target_sources(goldobot PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/low_pass_filter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/pid_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/trapezoidal_speed_profile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/propulsion/controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/propulsion/low_level_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/propulsion/simple_odometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/propulsion/trajectory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/propulsion.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_simulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sequence_engine.cpp
  )

target_include_directories(goldobot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_link_libraries(goldobot PUBLIC
  goldobot_platform_os)