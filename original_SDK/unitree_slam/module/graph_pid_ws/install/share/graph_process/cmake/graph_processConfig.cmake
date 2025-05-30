# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_graph_process_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED graph_process_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(graph_process_FOUND FALSE)
  elseif(NOT graph_process_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(graph_process_FOUND FALSE)
  endif()
  return()
endif()
set(_graph_process_CONFIG_INCLUDED TRUE)

# output package information
if(NOT graph_process_FIND_QUIETLY)
  message(STATUS "Found graph_process: 0.0.0 (${graph_process_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'graph_process' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${graph_process_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(graph_process_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${graph_process_DIR}/${_extra}")
endforeach()
