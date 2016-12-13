# vtk says: http://www.vtk.org/Wiki/CMake:How_To_Find_Libraries
# <NAME>_FOUND
# <NAME>_INCLUDE_DIRS or <NAME>_INCLUDES
# <NAME>_LIBRARIES or <NAME>_LIBRARIES or <NAME>_LIBS
# <NAME>_DEFINITIONS

# This module defines
# ZEROMQ_FOUND
# ZEROMQ_INCLUDE_DIRS
# ZEROMQ_LIBRARIES
# ZEROMQ_LIBRARY_DIRS
# ZEROMQ_ROOT_DIR

# The root directory specifies where to look first.
# Its value will be ZEROMQ_ROOT_DIR-NOTFOUND if the library cannot be found.
set(ZEROMQ_ROOT_DIR "ZEROMQ_ROOT_DIR-NOTFOUND" CACHE FILEPATH "Root directory of zeromq")

# Define the folders where to look for the library.
set(
  ZEROMQ_FIND_DIRS
    ${ZEROMQ_ROOT_DIR}
    /usr
)



# Clear the variables if the root directory changed.
# CMake does not search for files or libraries if they were already found.
# However, it does not detect if the root directory changed. Therefore, clear
# the variables if the root directory differs.
# Only do this if the root directory was at least set once. This allows to set
# the libraries and include paths manually without having to use the root directory.
if (NOT "${ZEROMQ_ROOT_DIR}" STREQUAL "ZEROMQ_ROOT_DIR-NOTFOUND")
  # Clear share variable
  string(REGEX MATCH "${ZEROMQ_ROOT_DIR}" ROOT_DIR_CMP "${ZEROMQ_CONFIG_FILE}")
  if ("${ROOT_DIR_CMP}" STREQUAL "")
    set(ZEROMQ_CONFIG_FILE ZEROMQ_CONFIG_FILE-NOTFOUND)
  endif ()

  # Clear include variable
  string(REGEX MATCH "${ZEROMQ_ROOT_DIR}" ROOT_DIR_CMP "${ZEROMQ_INCLUDE_DIRS}")
  if ("${ROOT_DIR_CMP}" STREQUAL "")
    set(ZEROMQ_INCLUDE_DIRS ZEROMQ_INCLUDE_DIRS-NOTFOUND)
  endif ()

  # Clear library dir variable
  string(REGEX MATCH "${ZEROMQ_ROOT_DIR}" ROOT_DIR_CMP "${ZEROMQ_LIBRARY_DIRS}")
  if ("${ROOT_DIR_CMP}" STREQUAL "")
    set(ZEROMQ_LIBRARY_DIRS ZEROMQ_LIBRARY_DIRS-NOTFOUND)
  endif ()

  # Clear library variable
  string(REGEX MATCH "${ZEROMQ_ROOT_DIR}" ROOT_DIR_CMP "${ZEROMQ_LIBRARIES}")
  if ("${ROOT_DIR_CMP}" STREQUAL "")
    set(ZEROMQ_LIBRARIES ZEROMQ_LIBRARIES-NOTFOUND)
  endif ()
endif()



# LINUX
if (CMAKE_SYSTEM MATCHES Linux)

  # Build the search paths for the include directory and library
  foreach (path ${ZEROMQ_FIND_DIRS})
    list(APPEND ZEROMQ_FIND_SHARE ${path}/share)
    list(APPEND ZEROMQ_FIND_INCLUDES ${path}/include)
    list(APPEND ZEROMQ_FIND_LIBRARIES ${path}/lib)
  endforeach(path ${ZEROMQ_FIND_DIRS})

  # First of all, try to find a configuration file.
  find_file(
    ZEROMQ_CONFIG_FILE
    NAMES zeromq/ZEROMQConfig.cmake
    PATHS ${ZEROMQ_FIND_SHARE}
  )

  if (ZEROMQ_CONFIG_FILE)
    include(${ZEROMQ_CONFIG_FILE})

    # Set the variables manually. Otherwise, they do not show up in the gui.
    set (ZEROMQ_FOUND true)
    set (ZEROMQ_INCLUDE_DIRS ${ZEROMQ_INCLUDE_DIRS} CACHE FILEPATH "Path to a file." FORCE)
    set (ZEROMQ_LIBRARY_DIRS ${ZEROMQ_LIBRARY_DIRS} CACHE FILEPATH "Path to a file." FORCE)

  else ()

    # Find the path to the header file
    find_path(
      ZEROMQ_INCLUDE_DIRS
      NAMES zmq.h
      PATHS ${ZEROMQ_FIND_INCLUDES}
    )

    # Find the library directory and the single libraries
    find_path(
      ZEROMQ_LIBRARY_DIRS
      NAMES libzmq.so libzmq.a
      PATHS ${ZEROMQ_FIND_LIBRARIES}
    )

    find_library(
      ZEROMQ_LIBRARIES
      NAMES zmq
      PATHS ${ZEROMQ_FIND_LIBRARIES}
    )
  endif (ZEROMQ_CONFIG_FILE)

  # Hide variables in the main gui
  #mark_as_advanced(
  #  ZEROMQ_INCLUDE_DIRS
  #)

  # Record if the find script was successful
  if (ZEROMQ_INCLUDE_DIRS AND ZEROMQ_LIBRARY_DIRS AND ZEROMQ_LIBRARIES)
    set (ZEROMQ_FOUND true)
  else ()
    set (ZEROMQ_FOUND false)
  endif ()

  # Let the user know if something was not found
  if (NOT ZEROMQ_INCLUDE_DIRS)
    message("Could not find ZEROMQ_INCLUDE_DIRS")
  endif ()

  if (NOT ZEROMQ_LIBRARY_DIRS)
    message("Could not find ZEROMQ_LIBRARY_DIR")
  endif ()

  if (NOT ZEROMQ_LIBRARIES)
    message("Could not find ZEROMQ_LIBRARIES")
  endif ()

  # If the library was found without using the root directory,
  # set it manually afterwards
  if (ZEROMQ_FOUND)
    if ("${ZEROMQ_ROOT_DIR}" STREQUAL "ZEROMQ_ROOT_DIR-NOTFOUND")
      foreach (path ${ZEROMQ_FIND_DIRS})
        find_path(
          ZEROMQ_FIND_ROOT_DIR
          NAMES zmq.h
          PATHS ${path}/include
        )

        if (NOT "${ZEROMQ_FIND_ROOT_DIR}" STREQUAL "ZEROMQ_FIND_ROOT_DIR-NOTFOUND")
          set (ZEROMQ_ROOT_DIR ${path} CACHE FILEPATH "Root directory of zeromq" FORCE)
        endif ()
        # Set the find root variable to not found to prevent it being count multiple
        # times as being found.
        set (ZEROMQ_FIND_ROOT_DIR ZEROMQ_FIND_ROOT_DIR-NOTFOUND)
      endforeach(path ${ZEROMQ_FIND_DIRS})
      # Hide the cache variable
      set (ZEROMQ_FIND_ROOT_DIR ${ZEROMQ_FIND_ROOT_DIR} CACHE INTERNAL "temporary variable")
    endif ()
  endif ()

endif(CMAKE_SYSTEM MATCHES Linux)

