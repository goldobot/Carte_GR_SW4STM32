cmake_minimum_required(VERSION 3.8)

project(goldo_propulsion LANGUAGES CXX)

add_library(goldo_propulsion STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/propulsion/controller.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/propulsion/controller_config.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/propulsion/low_level_controller.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/propulsion/odometry_config.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/propulsion/simple_odometry.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/propulsion/speed_controller.hpp

  ${CMAKE_CURRENT_SOURCE_DIR}/src/controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/low_level_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_simulator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/simple_odometry.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/speed_controller.cpp
  )

target_include_directories(goldo_propulsion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_link_libraries(goldo_propulsion
  goldo_math
  )

if (GOLDO_BUILD_TESTS)
  add_subdirectory(tests)
endif ()
