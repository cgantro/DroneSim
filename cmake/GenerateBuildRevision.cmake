set(revision "unknown")
set(worktree_dirty FALSE)

if(GIT_EXECUTABLE AND EXISTS "${SOURCE_DIR}/.git")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE revision
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  string(REPLACE "," ";" revision_paths "${REVISION_PATHS}")
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" status --porcelain -- ${revision_paths}
    WORKING_DIRECTORY "${SOURCE_DIR}"
    OUTPUT_VARIABLE worktree_status
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  if(worktree_status)
    set(worktree_dirty TRUE)
  endif()
endif()

if(NOT revision)
  set(revision "unknown")
endif()
if(worktree_dirty)
  string(APPEND revision "-dirty")
endif()

set(header "#pragma once\n\n#include <string_view>\n\nnamespace dronesim::build_info\n{\ninline constexpr std::string_view sourceRevision = \"${revision}\";\n}\n")
if(EXISTS "${OUTPUT}")
  file(READ "${OUTPUT}" existing_header)
endif()
if(NOT existing_header STREQUAL header)
  get_filename_component(output_directory "${OUTPUT}" DIRECTORY)
  file(MAKE_DIRECTORY "${output_directory}")
  file(WRITE "${OUTPUT}" "${header}")
endif()
