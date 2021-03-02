cmake_minimum_required(VERSION 3.10)

project(Carte_GR VERSION 0.0 LANGUAGES C CXX ASM)

option(GOLDO_BUILD_TESTS "Build the unit tests" OFF)
option(GOLDO_BUILD_NUCLEO_FIRMWARE "Build the Nucleo firmware" ON)

include("cmake/goldo_git_version.cmake")

if (GOLDO_BUILD_TESTS)
  add_subdirectory(googletest-download)
  enable_testing()
endif ()

if (GOLDO_BUILD_NUCLEO_FIRMWARE)
  set(GOLDOBOT_PLATFORM_OS FreeRTOS)
  goldo_git_version(GOLDO_GIT_VERSION)
  message("git version is: " ${GOLDO_GIT_VERSION})

  add_definitions("-DGOLDO_GIT_VERSION=\"${GOLDO_GIT_VERSION}\"")

  include_directories(firmware/Inc)

  add_subdirectory(Middlewares)
  add_subdirectory(Drivers)
  add_subdirectory(libs)
  add_subdirectory(goldobot)

  add_subdirectory(firmware)
else ()
  add_subdirectory(libs)
endif ()

