add_executable(sample_propulsion_controller 
  samples/sample_propulsion_controller.cpp
  )

target_link_libraries(sample_propulsion_controller goldo_propulsion thirdparty_yaml-cpp)

