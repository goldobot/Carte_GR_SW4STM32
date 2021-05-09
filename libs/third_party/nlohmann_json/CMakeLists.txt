add_library(thirdparty_nlohman_json INTERFACE)

target_include_directories(thirdparty_nlohman_json INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
