# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "CMakeFiles/equinoxgcs_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/equinoxgcs_autogen.dir/ParseCache.txt"
  "equinoxgcs_autogen"
  "ivyqt/CMakeFiles/IvyQt_autogen.dir/AutogenUsed.txt"
  "ivyqt/CMakeFiles/IvyQt_autogen.dir/ParseCache.txt"
  "ivyqt/IvyQt_autogen"
  "pprzlinkqt/CMakeFiles/pprzlinkQt_autogen.dir/AutogenUsed.txt"
  "pprzlinkqt/CMakeFiles/pprzlinkQt_autogen.dir/ParseCache.txt"
  "pprzlinkqt/pprzlinkQt_autogen"
  )
endif()
