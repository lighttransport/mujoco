# Apply a git patch once, while accepting a source tree that already contains
# the patch.  This is used by FetchContent and by the local/offline dependency
# path, so it intentionally avoids shell-specific syntax.

if(NOT DEFINED PATCH_FILE OR "${PATCH_FILE}" STREQUAL "")
  message(FATAL_ERROR "PATCH_FILE is required")
endif()

if(NOT DEFINED SOURCE_DIR OR "${SOURCE_DIR}" STREQUAL "")
  set(SOURCE_DIR ".")
endif()

set(_git_options
    -c core.autocrlf=false
    -c core.whitespace=cr-at-eol)

execute_process(
  COMMAND git ${_git_options} apply --quiet --reverse --check
          --whitespace=nowarn --ignore-space-change "${PATCH_FILE}"
  WORKING_DIRECTORY "${SOURCE_DIR}"
  RESULT_VARIABLE _reverse_result
)
if(_reverse_result EQUAL 0)
  return()
endif()

execute_process(
  COMMAND git ${_git_options} apply --check
          --whitespace=nowarn --ignore-space-change "${PATCH_FILE}"
  WORKING_DIRECTORY "${SOURCE_DIR}"
  RESULT_VARIABLE _check_result
  OUTPUT_VARIABLE _check_output
  ERROR_VARIABLE _check_error
)
if(NOT _check_result EQUAL 0)
  message(FATAL_ERROR
    "Patch cannot be applied or is not already applied: ${PATCH_FILE}\n"
    "${_check_output}${_check_error}"
  )
endif()

execute_process(
  COMMAND git ${_git_options} apply --verbose
          --whitespace=nowarn --ignore-space-change "${PATCH_FILE}"
  WORKING_DIRECTORY "${SOURCE_DIR}"
  RESULT_VARIABLE _apply_result
)
if(NOT _apply_result EQUAL 0)
  message(FATAL_ERROR "Failed to apply patch: ${PATCH_FILE}")
endif()
