include(GoogleTest)

add_executable(gtest_goldo_core
  utests/test_circular_buffer.cpp
  utests/test_message_exchange.cpp
  utests/test_message_queue.cpp
  )
  
target_link_libraries(gtest_goldo_core
  goldo_core
  gtest gtest_main)

gtest_discover_tests(gtest_goldo_core)
