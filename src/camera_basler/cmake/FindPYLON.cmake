# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindPYLON
-------

Finds the PYLON library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``PYLON::PYLON``
  The PYLON library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``PYLON_FOUND``
  True if the system has the PYLON library.
``PYLON_VERSION`` and ``PYLON_VERSION_STRING``
  The version of the PYLON library which was found.
``PYLON_INCLUDE_DIRS``
  The PYLON include directories.
``PYLON_LIBRARIES``
  The PYLON libraries.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``PYLON_INCLUDE_DIR``
  The directory containing GxIAPI.h.
``PYLON_LIBRARY``
  The path to the GxIAPI library.

#]=======================================================================]
# Try to find the PYLON library

# Find the include path which includes inc/GxIAPI.h
find_path(PYLON_INCLUDE_DIR
  NAMES "PylonIncludes.h"
  PATH_SUFFIXES "include"
  DOC "The directory containing PylonIncludes.h."
)
if(NOT　PYLON_INCLUDE_DIR)
  set(PYLON_INCLUDE_DIR "/opt/pylon/include/pylon")
endif()

# Find the specific libary under <prefix>/lib/[x64|x86]/
if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CMAKE_LIBRARY_ARCHITECTURE x64)
  else()
    set(CMAKE_LIBRARY_ARCHITECTURE x86)
  endif()
endif()

# Find the specific libary
find_library(PYLON_LIBRARY
  NAMES "pylonbase"
  DOC "The path to the pylonbase library."
)
if(NOT PYLON_LIBRARY)
  set(PYLON_LIBRARY "/opt/pylon/lib")
endif()

message("PYLON_LIBRARY: ${PYLON_LIBRARY} PYLON_INCLUDE_DIR :${PYLON_INCLUDE_DIR}")

unset(CMAKE_LIBRARY_ARCHITECTURE)

set(PYLON_ROOT $ENV{PYLON_ROOT})
message(STATUS "[env-PYLON_ROOT]: ${PYLON_ROOT}")
if (NOT DEFINED ENV{PYLON_ROOT})
    # set(PYLON_ROOT "/opt/pylon5")
    if(EXISTS "/opt/pylon/bin/pylon-config")
        string(REGEX REPLACE ".*/\(.*\)" "\\1" PYLON_ROOT "/opt/pylon")
        message(STATUS "PYLON_ROOT: ${PYLON_ROOT}")
    else()
        message(FATAL_ERROR  "/opt/pylon lib flodermake is NOT exits!")
    endif()
endif()


message(STATUS "PYLON_ROOT-->: ${PYLON_ROOT}")
set(_PYLON_CONFIG "/opt/pylon/bin/pylon-config")
    if (EXISTS "${_PYLON_CONFIG}")
        execute_process(COMMAND ${_PYLON_CONFIG} --version OUTPUT_VARIABLE PYLON_VERSION)        
        execute_process(COMMAND ${_PYLON_CONFIG} --cflags-only-I OUTPUT_VARIABLE HEADERS_OUT)
        execute_process(COMMAND ${_PYLON_CONFIG} --libs-only-l OUTPUT_VARIABLE LIBS_OUT)
        execute_process(COMMAND ${_PYLON_CONFIG} --libs-only-L OUTPUT_VARIABLE LIBDIRS_OUT)
        # Extract version information
        string(REGEX MATCH "[0-9.]+" PYLON_VERSION "${PYLON_VERSION}")        
        string(REPLACE " " ";" HEADERS_OUT "${HEADERS_OUT}")
        string(REPLACE "-I" "" HEADERS_OUT "${HEADERS_OUT}")
        string(REPLACE "\n" "" PYLON_INCLUDE_DIR "${HEADERS_OUT}")

        # Extract depends libraries information
        # message (STATUS "LIBS_OUT: ${LIBS_OUT}")
        string(REPLACE " " ";" LIBS_OUT "${LIBS_OUT}")
        string(REPLACE "-l" "" LIBS_OUT "${LIBS_OUT}")
        string(REPLACE "\n" "" PYLON_LIBRARY "${LIBS_OUT}")
        # message (STATUS "PYLON_LIBRARY : ${PYLON_LIBRARY}")

        # Extract depends libraries's absolute path information
        string(REPLACE " " ";" LIBDIRS_OUT "${LIBDIRS_OUT}")
        string(REPLACE "-L" "" LIBDIRS_OUT "${LIBDIRS_OUT}")
        string(REPLACE "\n" "" LIBDIRS_OUT "${LIBDIRS_OUT}")
    else()
        set(PYLON_FOUND FALSE)
        message(FATAL_ERROR "pylon-config NOT Found")
    endif()

# Conventional find package operation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PYLON
  REQUIRED_VARS
    PYLON_LIBRARY
    PYLON_INCLUDE_DIR
  VERSION_VAR PYLON_VERSION
)

# Setup import target which can be utilized by target_link_libraries
if(PYLON_FOUND AND NOT TARGET PYLON::PYLON)
  add_library(PYLON::PYLON INTERFACE IMPORTED)
  target_include_directories(PYLON::PYLON INTERFACE ${PYLON_INCLUDE_DIR})
  target_link_libraries(PYLON::PYLON INTERFACE ${PYLON_LIBRARY})
  target_link_directories(PYLON::PYLON INTERFACE ${LIBDIRS_OUT})
  message("ok \r\n")
endif()

# For systems depend on INCLUDE_DIRS and LIBRARIES
set(PYLON_INCLUDE_DIRS ${PYLON_INCLUDE_DIR})
set(PYLON_LIBRARIES ${PYLON_LIBRARY})

mark_as_advanced(
  PYLON_INCLUDE_DIR
  PYLON_LIBRARY
)

# compatibility variables
set(PYLON_VERSION_STRING ${PYLON_VERSION})