#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "IvyQt" for configuration "Release"
set_property(TARGET IvyQt APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(IvyQt PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libIvyQt.a"
  )

list(APPEND _cmake_import_check_targets IvyQt )
list(APPEND _cmake_import_check_files_for_IvyQt "${_IMPORT_PREFIX}/lib/x86_64-linux-gnu/libIvyQt.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
