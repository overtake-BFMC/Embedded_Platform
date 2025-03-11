# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.8)
   message(FATAL_ERROR "CMake >= 2.8.3 required")
endif()
if(CMAKE_VERSION VERSION_LESS "2.8.3")
   message(FATAL_ERROR "CMake >= 2.8.3 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.8.3...3.29)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_cmake_targets_defined "")
set(_cmake_targets_not_defined "")
set(_cmake_expected_targets "")
foreach(_cmake_expected_target IN ITEMS greentea::client greentea::client_userio)
  list(APPEND _cmake_expected_targets "${_cmake_expected_target}")
  if(TARGET "${_cmake_expected_target}")
    list(APPEND _cmake_targets_defined "${_cmake_expected_target}")
  else()
    list(APPEND _cmake_targets_not_defined "${_cmake_expected_target}")
  endif()
endforeach()
unset(_cmake_expected_target)
if(_cmake_targets_defined STREQUAL _cmake_expected_targets)
  unset(_cmake_targets_defined)
  unset(_cmake_targets_not_defined)
  unset(_cmake_expected_targets)
  unset(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT _cmake_targets_defined STREQUAL "")
  string(REPLACE ";" ", " _cmake_targets_defined_text "${_cmake_targets_defined}")
  string(REPLACE ";" ", " _cmake_targets_not_defined_text "${_cmake_targets_not_defined}")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_cmake_targets_defined_text}\nTargets not yet defined: ${_cmake_targets_not_defined_text}\n")
endif()
unset(_cmake_targets_defined)
unset(_cmake_targets_not_defined)
unset(_cmake_expected_targets)


# Create imported target greentea::client
add_library(greentea::client STATIC IMPORTED)

set_target_properties(greentea::client PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "C:/Users/marko/Documents/BoschChall/Embedded_Platform/cmake_build/NUCLEO_F401RE/develop/GCC_ARM/_deps/greentea-client-src/include"
)

# Create imported target greentea::client_userio
add_library(greentea::client_userio STATIC IMPORTED)

set_target_properties(greentea::client_userio PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "C:/Users/marko/Documents/BoschChall/Embedded_Platform/cmake_build/NUCLEO_F401RE/develop/GCC_ARM/_deps/greentea-client-src/include"
)

# Import target "greentea::client" for configuration "develop"
set_property(TARGET greentea::client APPEND PROPERTY IMPORTED_CONFIGURATIONS DEVELOP)
set_target_properties(greentea::client PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEVELOP "C;CXX"
  IMPORTED_LOCATION_DEVELOP "C:/Users/marko/Documents/BoschChall/Embedded_Platform/cmake_build/NUCLEO_F401RE/develop/GCC_ARM/_deps/greentea-client-build/libclient.a"
  )

# Import target "greentea::client_userio" for configuration "develop"
set_property(TARGET greentea::client_userio APPEND PROPERTY IMPORTED_CONFIGURATIONS DEVELOP)
set_target_properties(greentea::client_userio PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEVELOP "CXX"
  IMPORTED_LOCATION_DEVELOP "C:/Users/marko/Documents/BoschChall/Embedded_Platform/cmake_build/NUCLEO_F401RE/develop/GCC_ARM/_deps/greentea-client-build/libclient_userio.a"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
