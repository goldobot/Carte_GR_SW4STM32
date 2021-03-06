cmake_minimum_required(VERSION 3.8)

project(goldo_core LANGUAGES CXX)

add_library(goldo_core STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src/message_exchange.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/message_queue.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/version.cpp
  )

target_sources(goldo_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/core/circular_buffer.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/core/message_exchange.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/goldobot/core/message_queue.hpp
  )


target_include_directories(goldo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_compile_definitions(goldo_core PRIVATE
 GOLDO_GIT_COMMIT_ID="${GOLDO_GIT_COMMIT_ID}"
 GOLDO_GIT_COMMIT_SHORT_ID="${GOLDO_GIT_COMMIT_SHORT_ID}"
 GOLDO_GIT_COMMIT_DATE="${GOLDO_GIT_COMMIT_DATE}"
 GOLDO_GIT_BRANCH="${GOLDO_GIT_BRANCH}"
)

if (GOLDO_BUILD_NUCLEO_FIRMWARE)
  target_compile_definitions(goldo_core PUBLIC GOLDOBOT_PLATFORM_OS_FreeRTOS)
  target_link_libraries(goldo_core PUBLIC 
                        middlewares_FreeRTOS
                        )
endif ()

if (GOLDO_BUILD_TESTS)
  add_subdirectory(tests)
endif ()

if (GOLDO_BUILD_FUZZERS)
  add_subdirectory(fuzzers)
endif ()