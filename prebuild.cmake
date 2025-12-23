execute_process(
  COMMAND git status --porcelain
  OUTPUT_VARIABLE GIT_STATUS
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

if(GIT_STATUS)
  execute_process(
    COMMAND git add -A
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  execute_process(
    COMMAND git commit -m "[automated-commit] Pre-build commit for binary metadata generation"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  message(STATUS "Changes committed")
else()
  message(STATUS "No changes to commit")
endif()
