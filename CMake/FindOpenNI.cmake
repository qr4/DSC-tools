# This module defines
# OpenNI_FOUND
# OpenNI_INCLUDE_DIRS
# OpenNI_LIBRARIES



# LINUX
if (CMAKE_SYSTEM MATCHES Linux)

  set(OpenNI_FOUND TRUE)

  # Find the include directories
  find_path(
    OpenNI_INCLUDE_DIRS
    NAMES XnOS.h
    PATHS /usr/include/ni
    )
  if ("${OpenNI_INCLUDE_DIRS}" STREQUAL "")
    set(OpenNI_FOUND FALSE)
  endif()

  # Find the library
  find_library(
    OpenNI_LIBRARIES
    NAMES OpenNI
    PATHS /usr/lib
    )
  if ("${OpenNI_LIBRARIES}" STREQUAL "")
    set(OpenNI_FOUND FALSE)
  endif()

endif(CMAKE_SYSTEM MATCHES Linux)

