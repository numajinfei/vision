# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindGALAXY
-------

Finds the GALAXY library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``GALAXY::GALAXY``
  The GALAXY library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``GALAXY_FOUND``
  True if the system has the GALAXY library.
``GALAXY_VERSION`` and ``GALAXY_VERSION_STRING``
  The version of the GALAXY library which was found.
``GALAXY_INCLUDE_DIRS``
  The GALAXY include directories.
``GALAXY_LIBRARIES``
  The GALAXY libraries.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``GALAXY_INCLUDE_DIR``
  The directory containing GxIAPI.h.
``GALAXY_LIBRARY``
  The path to the GxIAPI library.

#]=======================================================================]
# Try to find the GALAXY library

# Find the include path which includes inc/GxIAPI.h
find_path(GALAXY_INCLUDE_DIR
  NAMES "GxIAPI.h"
  #HINTS "/opt/Galaxy_camera/inc/"
  HINTS "/opt/GALAXY/inc"
  DOC "The directory containing GxIAPI.h."
)
if(NOT GALAXY_INCLUDE_DIR)
  set(GALAXY_INCLUDE_DIR "/opt/GALAXY/inc")
endif()
# Find the specific libary under <prefix>/lib/[x64|x86]/
#if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(CMAKE_LIBRARY_ARCHITECTURE x64)
  else()
    set(CMAKE_LIBRARY_ARCHITECTURE x86)
  endif()
#endif()

# Get the specific hardware platform under <prefix>/lib/[platform]/
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
  message(STATUS "current platform: Linux")
  message(STATUS "--> ${CMAKE_HOST_SYSTEM_PROCESSOR}")

  if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(PLATFORM armv8)
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(PLATFORM x86_64)
  else()
    message(STATUS "Host platform is: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
  endif()
else()
  message(STATUS "Host system is: ${CMAKE_SYSTEM_NAME}")
endif()

# Find the specific libary
find_library(GALAXY_LIBRARY
  NAMES "gxiapi"
  HINTS "/opt/GALAXY/lib/${PLATFORM}"
  DOC "The path to the GxIAPI library."
)

#if(NOT GALAXY_LIBRARY)
#  set(GALAXY_LIBRARY "/opt/GALAXY/lib/armv8/libgxiapi.so")
#endif()
unset(CMAKE_LIBRARY_ARCHITECTURE)
unset(PLATFORM)

# Extract version information
file(STRINGS "${GALAXY_INCLUDE_DIR}/GxIAPI.h" GALAXY_VERSION REGEX "@Version")
string(REGEX MATCH "[0-9.]+$" GALAXY_VERSION "${GALAXY_VERSION}")

# Conventional find package operation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GALAXY
  REQUIRED_VARS
    GALAXY_LIBRARY
    GALAXY_INCLUDE_DIR
  VERSION_VAR GALAXY_VERSION
)

# Setup import target which can be utilized by target_link_libraries
if(GALAXY_FOUND AND NOT TARGET GALAXY::GALAXY)
  add_library(GALAXY::GALAXY UNKNOWN IMPORTED)
  set_target_properties(GALAXY::GALAXY PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GALAXY_INCLUDE_DIR}"
    IMPORTED_LOCATION "${GALAXY_LIBRARY}"
  )
endif()

# For systems depend on INCLUDE_DIRS and LIBRARIES
set(GALAXY_INCLUDE_DIRS ${GALAXY_INCLUDE_DIR})
set(GALAXY_LIBRARIES ${GALAXY_LIBRARY})

mark_as_advanced(
  GALAXY_INCLUDE_DIR
  GALAXY_LIBRARY
)

# compatibility variables
set(GALAXY_VERSION_STRING ${GALAXY_VERSION})
