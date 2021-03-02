add_executable(sample_pid_controller 
  samples/sample_pid_controller.cpp
  samples/yaml/Yaml.cpp
  )

target_link_libraries(sample_pid_controller goldo_math)

include(GoogleTest)

add_executable(gtest_goldo_math 
  utests/test_derivative_filter.cpp
  )
  
target_link_libraries(gtest_goldo_math
  goldo_math
  gtest gtest_main)

gtest_discover_tests(gtest_goldo_math)
