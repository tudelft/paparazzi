# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "CMakeFiles/messages_qt_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/messages_qt_autogen.dir/ParseCache.txt"
  "ivyqt/CMakeFiles/IvyQt_autogen.dir/AutogenUsed.txt"
  "ivyqt/CMakeFiles/IvyQt_autogen.dir/ParseCache.txt"
  "ivyqt/IvyQt_autogen"
  "messages_qt_autogen"
  "pprzlinkqt/CMakeFiles/pprzlinkQt_autogen.dir/AutogenUsed.txt"
  "pprzlinkqt/CMakeFiles/pprzlinkQt_autogen.dir/ParseCache.txt"
  "pprzlinkqt/pprzlinkQt_autogen"
  )
endif()
