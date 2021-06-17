cmake_minimum_required(VERSION 3.10)

project(Carte_GR VERSION 0.0 LANGUAGES C CXX ASM)

# get git commit id
find_package(Git)

if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
    execute_process(COMMAND ${GIT_EXECUTABLE}  show -s --format=%H OUTPUT_VARIABLE GOLDO_GIT_COMMIT_ID OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND ${GIT_EXECUTABLE}  show -s --format=%h OUTPUT_VARIABLE GOLDO_GIT_COMMIT_SHORT_ID OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND ${GIT_EXECUTABLE}  show -s --format=%ci OUTPUT_VARIABLE GOLDO_GIT_COMMIT_DATE OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND ${GIT_EXECUTABLE}  show -s --format=%D OUTPUT_VARIABLE GOLDO_GIT_COMMIT_REF_NAMES OUTPUT_STRIP_TRAILING_WHITESPACE)
    message("git version is: " ${GOLDO_GIT_COMMIT_ID})
else()
    message(WARNING "git not found, cannot set version info")
    set(GOLDO_GIT_COMMIT_ID "")
    set(GOLDO_GIT_COMMIT_SHORT_ID "")
    set(GOLDO_GIT_COMMIT_DATE "")
    set(GOLDO_GIT_COMMIT_REF_NAMES "")
endif()

option(GOLDO_BUILD_TESTS "Build the unit tests" OFF)
option(GOLDO_BUILD_FUZZERS "Build the fuzzers" OFF)
option(GOLDO_BUILD_NUCLEO_FIRMWARE "Build the Nucleo firmware" ON)
option(GOLDO_NUCLEO_HAL_TRACE "Enable HAL trace functions in the Nucleo firmware" OFF)

include("cmake/goldo_git_version.cmake")

if (GOLDO_BUILD_TESTS)
  add_subdirectory(googletest-download)
  enable_testing()
endif ()

if (GOLDO_BUILD_NUCLEO_FIRMWARE)
  set(GOLDOBOT_PLATFORM_OS FreeRTOS)  

  include_directories(firmware/Inc)

  add_subdirectory(Middlewares)
  add_subdirectory(Drivers)
  add_subdirectory(libs)
  add_subdirectory(goldobot)

  add_subdirectory(firmware)
else ()
  add_subdirectory(libs)
endif ()

