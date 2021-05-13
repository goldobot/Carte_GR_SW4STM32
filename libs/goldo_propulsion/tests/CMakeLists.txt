add_executable(sample_propulsion_controller 
  samples/sample_propulsion_controller.cpp
samples/sample_propulsion_controller.cpp
  )

target_link_libraries(sample_propulsion_controller goldo_propulsion thirdparty_yaml-cpp)

add_executable(sample_speed_controller 
  samples/sample_speed_controller.cpp
  )

target_link_libraries(sample_speed_controller goldo_propulsion thirdparty_yaml-cpp)

