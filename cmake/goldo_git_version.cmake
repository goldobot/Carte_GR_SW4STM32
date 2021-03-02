function(goldo_git_version OutVariable)
  execute_process(
    COMMAND git describe --all
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  execute_process(
    COMMAND git describe --abbrev=8 --dirty --always
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_REF
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(${OutVariable} "${GIT_BRANCH} ${GIT_REF}" PARENT_SCOPE)
endfunction()