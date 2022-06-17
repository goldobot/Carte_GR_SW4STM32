add_library(middlewares_Eigen INTERFACE)

target_include_directories(middlewares_Eigen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
