# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rplidar_camera_calibration_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rplidar_camera_calibration_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rplidar_camera_calibration_FOUND FALSE)
  elseif(NOT rplidar_camera_calibration_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rplidar_camera_calibration_FOUND FALSE)
  endif()
  return()
endif()
set(_rplidar_camera_calibration_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rplidar_camera_calibration_FIND_QUIETLY)
  message(STATUS "Found rplidar_camera_calibration: 0.0.1 (${rplidar_camera_calibration_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rplidar_camera_calibration' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rplidar_camera_calibration_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rplidar_camera_calibration_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rplidar_camera_calibration_DIR}/${_extra}")
endforeach()
