# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_xycar_ultrasonic_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED xycar_ultrasonic_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(xycar_ultrasonic_FOUND FALSE)
  elseif(NOT xycar_ultrasonic_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(xycar_ultrasonic_FOUND FALSE)
  endif()
  return()
endif()
set(_xycar_ultrasonic_CONFIG_INCLUDED TRUE)

# output package information
if(NOT xycar_ultrasonic_FIND_QUIETLY)
  message(STATUS "Found xycar_ultrasonic: 0.0.0 (${xycar_ultrasonic_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'xycar_ultrasonic' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${xycar_ultrasonic_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(xycar_ultrasonic_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${xycar_ultrasonic_DIR}/${_extra}")
endforeach()
