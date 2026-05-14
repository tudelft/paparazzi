#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "IvyQt" for configuration ""
set_property(TARGET IvyQt APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(IvyQt PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libIvyQt.a"
  )

list(APPEND _cmake_import_check_targets IvyQt )
list(APPEND _cmake_import_check_files_for_IvyQt "${_IMPORT_PREFIX}/lib/libIvyQt.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
