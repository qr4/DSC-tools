# This module defines
# OpenNI2_ROOT_DIR      Set this variable to point to the openni root
# OpenNI2_FOUND
# OpenNI2_INCLUDE_DIRS
# OpenNI2_LIBRARIES

# LINUX
if (CMAKE_SYSTEM MATCHES Linux)
 
  find_path(OpenNI2_ROOT_DIR
      NAMES
          include/OpenNI.h
      HINTS
          ${OpenNI2_ROOT_DIR}
          $ENV{OpenNI2_ROOT_DIR}
  )

  # Find the include directories
  find_path(
    OpenNI2_INCLUDE_DIRS
    NAMES OpenNI.h
    PATHS $ENV{OpenNI2_ROOT_DIR} ${OpenNI2_ROOT_DIR}
    PATH_SUFFIXES Include
    )

  # Find the library
  find_library(
    OpenNI2_LIBRARIES
    NAMES OpenNI2
    PATHS $ENV{OpenNI2_ROOT_DIR} ${OpenNI2_ROOT_DIR}
    PATH_SUFFIXES Redist Bin/x64-Release
    )
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(OpenNI2  DEFAULT_MSG
                                  OpenNI2_LIBRARIES OpenNI2_INCLUDE_DIRS)
endif(CMAKE_SYSTEM MATCHES Linux)

