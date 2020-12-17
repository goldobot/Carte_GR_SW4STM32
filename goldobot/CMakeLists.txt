cmake_minimum_required(VERSION 3.8)

project(goldobot LANGUAGES CXX)

add_subdirectory(platforms)

add_library(goldobot STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/sequence_engine.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/hal.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/tasks/main.hpp

  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/derivative_filter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/math_utils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/pid_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core/trapezoidal_speed_profile.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/src/odrive/odrive_stream_parser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/odrive/odrive_stream_writer.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/src/propulsion/controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/propulsion/low_level_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/propulsion/simple_odometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/propulsion/trajectory.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/debug.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/dynamixels_comm.cpp  
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/fpga.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/heartbeat.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/odrive_comm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/servos.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/propulsion.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/tasks/uart_comm.cpp

  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_simulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sequence_engine.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/comm_serializer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/comm_deserializer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/crc.cpp
  )

target_include_directories(goldobot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_link_libraries(goldobot PUBLIC
  goldobot_platform_board
)