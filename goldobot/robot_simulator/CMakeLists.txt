set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "disable yaml tests")
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "disable yaml tools")
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "disable yaml contrib")
add_subdirectory(yaml-cpp)

add_executable(goldobot_test 
  main.cpp
  hal.cpp
  robot_emulator.hpp
  robot_emulator.cpp
  )

target_link_libraries(goldobot_test goldobot 
"${CMAKE_CURRENT_SOURCE_DIR}/libzmq-v141-mt-4_3_2.lib"
  yaml-cpp
)