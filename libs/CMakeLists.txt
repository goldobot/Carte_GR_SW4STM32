add_subdirectory(goldo_core)
add_subdirectory(goldo_math)
add_subdirectory(goldo_odrive)
add_subdirectory(goldo_propulsion)

if (GOLDO_BUILD_TESTS)
  add_subdirectory(third_party)
endif ()
