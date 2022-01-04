# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

#[=======================================================================[.rst:
FindSpinnaker
-------

Finds the Spinnaker library.

Imported Targets
^^^^^^^^^^^^^^^^

This module provides the following imported targets, if found:

``Spinnaker::Spinnaker``
  The Spinnaker library

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``Spinnaker_FOUND`` and ``SPINNAKER_FOUND``
  True if the system has the Spinnaker library.
``Spinnaker_VERSION`` and ``Spinnaker_VERSION_STRING``
  The version of the Spinnaker library which was found.
``Spinnaker_INCLUDE_DIRS``
  The include directories.
``Spinnaker_LIBRARIES``
  The Spinnaker libraries.

Cache Variables
^^^^^^^^^^^^^^^

The following cache variables may also be set:

``Spinnaker_INCLUDE_DIR``
  The directory containing Spinnaker.h.
``Spinnaker_LIBRARY``
  The path to the Spinnaker library.

#]=======================================================================]
# Try to find the Spinnaker library

# Find the include path
find_path(Spinnaker_INCLUDE_DIR
  NAMES "Spinnaker.h"
  HINTS "/opt/spinnaker/include/"
  DOC "The directory containing Spinnaker.h."
)

# Find the specific library
find_library(Spinnaker_LIBRARY
  NAMES "Spinnaker"
  HINTS "/opt/spinnaker/lib/"
  DOC "The path to the Spinnaker library."
)

# Extract version information
file(STRINGS "${Spinnaker_INCLUDE_DIR}/System.h" Spinnaker_VERSION REGEX "FLIR_SPINNAKER_VERSION")
foreach(_str IN LISTS Spinnaker_VERSION)
  string(FIND "${_str}" "MAJOR" _num)
  if(NOT _num EQUAL -1)
    string(REGEX MATCH "[0-9.]+$" Spinnaker_VERSION_MAJOR "${_str}")
    continue()
  endif()

  string(FIND "${_str}" "MINOR" _num)
  if(NOT _num EQUAL -1)
    string(REGEX MATCH "[0-9.]+$" Spinnaker_VERSION_MINOR "${_str}")
    continue()
  endif()

  string(FIND "${_str}" "TYPE" _num)
  if(NOT _num EQUAL -1)
    string(REGEX MATCH "[0-9.]+$" Spinnaker_VERSION_PATCH "${_str}")
    continue()
  endif()

  string(FIND "${_str}" "BUILD" _num)
  if(NOT _num EQUAL -1)
    string(REGEX MATCH "[0-9.]+$" Spinnaker_VERSION_TWEAK "${_str}")
    continue()
  endif()
endforeach()

unset(_str)
unset(_num)

set(Spinnaker_VERSION ${Spinnaker_VERSION_MAJOR}.${Spinnaker_VERSION_MINOR}.${Spinnaker_VERSION_PATCH}.${Spinnaker_VERSION_TWEAK})

# Conventional find package operation
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Spinnaker
  REQUIRED_VARS
    Spinnaker_LIBRARY
    Spinnaker_INCLUDE_DIR
  VERSION_VAR Spinnaker_VERSION
)

# Setup import target which can be utilized by target_link_libraries
if(Spinnaker_FOUND AND NOT TARGET Spinnaker::Spinnaker)
  add_library(Spinnaker::Spinnaker UNKNOWN IMPORTED)
  set_target_properties(Spinnaker::Spinnaker PROPERTIES
    INTERFACE_COMPILE_DEFINITIONS "LINUX"
    INTERFACE_INCLUDE_DIRECTORIES "${Spinnaker_INCLUDE_DIR}"
    IMPORTED_LOCATION "${Spinnaker_LIBRARY}"
  )
endif()

# For systems depend on INCLUDE_DIRS and LIBRARIES
set(Spinnaker_INCLUDE_DIRS ${Spinnaker_INCLUDE_DIR})
set(Spinnaker_LIBRARIES ${Spinnaker_LIBRARY})

mark_as_advanced(
  Spinnaker_INCLUDE_DIR
  Spinnaker_LIBRARY
)

# compatibility variables
set(Spinnaker_VERSION_STRING ${Spinnaker_VERSION})
