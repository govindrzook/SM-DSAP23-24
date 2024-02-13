# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_digital_architecture_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED digital_architecture_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(digital_architecture_FOUND FALSE)
  elseif(NOT digital_architecture_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(digital_architecture_FOUND FALSE)
  endif()
  return()
endif()
set(_digital_architecture_CONFIG_INCLUDED TRUE)

# output package information
if(NOT digital_architecture_FIND_QUIETLY)
  message(STATUS "Found digital_architecture: 0.0.0 (${digital_architecture_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'digital_architecture' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${digital_architecture_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(digital_architecture_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${digital_architecture_DIR}/${_extra}")
endforeach()
