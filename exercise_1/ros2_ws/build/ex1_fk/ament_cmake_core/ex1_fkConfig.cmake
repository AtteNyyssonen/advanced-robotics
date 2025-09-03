# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ex1_fk_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ex1_fk_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ex1_fk_FOUND FALSE)
  elseif(NOT ex1_fk_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ex1_fk_FOUND FALSE)
  endif()
  return()
endif()
set(_ex1_fk_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ex1_fk_FIND_QUIETLY)
  message(STATUS "Found ex1_fk: 0.0.0 (${ex1_fk_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ex1_fk' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ex1_fk_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ex1_fk_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ex1_fk_DIR}/${_extra}")
endforeach()
