###############################################################################
# In-tree build prevention
###############################################################################
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "Building in the source directory is not supported!
          Remove CMakeCache.txt from the source directory and use an out-of-tree build.")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})



###############################################################################
# Project declaration and general settings
###############################################################################
cmake_minimum_required(VERSION 2.8)

# Project name and version number
project("dsc-tools")
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
set(${PROJECT_NAME}_VERSION_MAJOR "1")
set(${PROJECT_NAME}_VERSION_MINOR "0")
set(${PROJECT_NAME}_VERSION_PATCH "0")
set(${PROJECT_NAME}_VERSION "${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH}")

# Local CMake macros
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake)

# Configure output paths for binaries and libraries
set(INCLUDE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/include)
set(LIBRARY_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/lib)
set(EXECUTABLE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/bin)
set(SHARE_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME_LOWER})
set(DATA_INSTALL_PATH ${SHARE_INSTALL_PATH}/data)

###############################################################################
# Install path
###############################################################################
# Set the root path where the libraries are installed.
# This can be either done with the environment variable LIBRARY_ROOT_PATH or
# by using the -D option (-DLIBRARY_ROOT_PATH=/path/to/libraries).
# The -D option has a higher priority than environment variables.
if("${LIBRARY_ROOT_PATH}" STREQUAL "")
  if(NOT "$ENV{LIBRARY_ROOT_PATH}" STREQUAL "")
    set(LIBRARY_ROOT_PATH $ENV{LIBRARY_ROOT_PATH})
  endif()
endif()

# Also set the install prefix to the library root path if it is set.
# In this case, also append the project name to install prefix.
if(NOT "${LIBRARY_ROOT_PATH}" STREQUAL "")
  if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX_TMP ${LIBRARY_ROOT_PATH}/${CMAKE_PROJECT_NAME})
    string(TOLOWER ${CMAKE_INSTALL_PREFIX_TMP} CMAKE_INSTALL_PREFIX_TMP)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX_TMP} CACHE PATH "Install path prefix, prepended onto install directories." FORCE)
  endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
endif()



###############################################################################
# Specify the rpath handling
# original version from: http://www.vtk.org/Wiki/CMake_RPATH_handling
###############################################################################
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH ${LIBRARY_INSTALL_PATH})

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES ${LIBRARY_INSTALL_PATH} isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH ${LIBRARY_INSTALL_PATH})
endif()



###############################################################################
# Configurable build options
###############################################################################

# Set default build type to "release with debug information".
# original version: http://www.cmake.org/Wiki/CMake_FAQ#How_can_I_change_the_default_build_mode_and_see_it_reflected_in_the_GUI.3F
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()


###############################################################################
# Libraries
###############################################################################

  # PCL library
find_package(PCL 1.7 REQUIRED)
if(PCL_FOUND)
  list(APPEND EXTERNAL_PROJECT_INCLUDES ${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif(PCL_FOUND)

# Boost
set(Boost_USE_MULTITHREADED ON)
find_package(Boost COMPONENTS filesystem system thread program_options REQUIRED)

if(Boost_FOUND)
  list(APPEND SYSTEM_INCLUDES ${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)

# OpenCV
set(OpenCV_DIR /usr/ CACHE FILEPATH "OpenCV root directory")
find_package(OpenCV PATHS ${OpenCV_DIR} ${OpenCV_DIR}/share/OpenCV ${OpenCV_DIR}/share/opencv REQUIRED NO_MODULE)
if(OpenCV_FOUND)
  link_directories(${OpenCV_LIB_DIR})
  list(APPEND EXTERNAL_PROJECT_INCLUDES ${OpenCV_INCLUDE_DIRS})
endif(OpenCV_FOUND)

# OpenNI2
find_package(OpenNI2 REQUIRED)
if(OPENNI2_FOUND)
  # Include and link directories
  list(APPEND EXTERNAL_PROJECT_INCLUDES ${OpenNI2_INCLUDE_DIRS})
  link_directories(${OPENNI2_LIBRARY_DIRS})
endif(OPENNI2_FOUND)

#############################################################################
# Project includes and library dirs
###############################################################################

list(APPEND PROJECT_INCLUDES ${CMAKE_SOURCE_DIR}/include)

include_directories(${PROJECT_INCLUDES})
include_directories(${EXTERNAL_PROJECT_INCLUDES})


###############################################################################
# Platform-specific settings
###############################################################################

# The special flags (e.g. debug and release) are added to the general flags
if (CMAKE_COMPILER_IS_GNUCXX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wno-unused-parameter -pthread -fPIC")
endif (CMAKE_COMPILER_IS_GNUCXX)


###############################################################################
# Print status info
###############################################################################

string(TOUPPER ${CMAKE_BUILD_TYPE} TMP_BUILD_TYPE)
message(STATUS "CXXFLAGS: ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${TMP_BUILD_TYPE}}")
message(STATUS "Install directory: " ${CMAKE_INSTALL_PREFIX})


###############################################################################
# Add the source folder
# This folder contains the library, applications
###############################################################################
add_subdirectory(src)
add_subdirectory(app)

###############################################################################
# Install targets
###############################################################################
install(DIRECTORY include/ DESTINATION ${INCLUDE_INSTALL_PATH})
install(DIRECTORY data/ DESTINATION ${DATA_INSTALL_PATH})

###############################################################################
# Generate CMake project files
###############################################################################
# Version information
set(CONFIG_VERSION "${${PROJECT_NAME}_VERSION}")
set(CONFIG_VERSION_MAJOR "${${PROJECT_NAME}_VERSION_MAJOR}")
set(CONFIG_VERSION_MINOR "${${PROJECT_NAME}_VERSION_MINOR}")
set(CONFIG_VERSION_PATCH "${${PROJECT_NAME}_VERSION_PATCH}")

# Include directories
list(APPEND ALL_INCLUDES ${INCLUDE_INSTALL_PATH})
list(APPEND ALL_INCLUDES ${EXTERNAL_PROJECT_INCLUDES})
string(REPLACE " " "\\ " CONFIG_INCLUDE_DIRS "${ALL_INCLUDES}")

# Library directories
set(CONFIG_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib")
set(CONFIG_PACKAGE_FILE "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME_LOWER}/${PROJECT_NAME}Config.cmake")

# Configure file in temporary directory
configure_file("${CMAKE_SOURCE_DIR}/CMake/Config.cmake.in"
               "${CMAKE_BINARY_DIR}/install-config/${PROJECT_NAME}Config.cmake" @ONLY)

# Install Config.cmake file
install(FILES ${CMAKE_BINARY_DIR}/install-config/${PROJECT_NAME}Config.cmake DESTINATION ${SHARE_INSTALL_PATH})

