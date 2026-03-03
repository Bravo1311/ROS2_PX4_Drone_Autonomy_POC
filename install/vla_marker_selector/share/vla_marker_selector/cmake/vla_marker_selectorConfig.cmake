# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vla_marker_selector_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vla_marker_selector_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vla_marker_selector_FOUND FALSE)
  elseif(NOT vla_marker_selector_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vla_marker_selector_FOUND FALSE)
  endif()
  return()
endif()
set(_vla_marker_selector_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vla_marker_selector_FIND_QUIETLY)
  message(STATUS "Found vla_marker_selector: 0.0.0 (${vla_marker_selector_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vla_marker_selector' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT vla_marker_selector_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vla_marker_selector_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${vla_marker_selector_DIR}/${_extra}")
endforeach()
