# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/orangepi/paparazzi/sw/ext/sixdof_system/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/orangepi/paparazzi/sw/ext/sixdof_system/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/heartbeat_manager.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/heartbeat_manager.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/heartbeat_manager.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/heartbeat_manager.dir/flags.make

CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o: CMakeFiles/heartbeat_manager.dir/flags.make
CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o: ../heartbeat_manager.cpp
CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o: CMakeFiles/heartbeat_manager.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orangepi/paparazzi/sw/ext/sixdof_system/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o -MF CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o.d -o CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o -c /home/orangepi/paparazzi/sw/ext/sixdof_system/examples/heartbeat_manager.cpp

CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/orangepi/paparazzi/sw/ext/sixdof_system/examples/heartbeat_manager.cpp > CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.i

CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/orangepi/paparazzi/sw/ext/sixdof_system/examples/heartbeat_manager.cpp -o CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.s

# Object files for target heartbeat_manager
heartbeat_manager_OBJECTS = \
"CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o"

# External object files for target heartbeat_manager
heartbeat_manager_EXTERNAL_OBJECTS =

heartbeat_manager: CMakeFiles/heartbeat_manager.dir/heartbeat_manager.cpp.o
heartbeat_manager: CMakeFiles/heartbeat_manager.dir/build.make
heartbeat_manager: CMakeFiles/heartbeat_manager.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/orangepi/paparazzi/sw/ext/sixdof_system/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable heartbeat_manager"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/heartbeat_manager.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/heartbeat_manager.dir/build: heartbeat_manager
.PHONY : CMakeFiles/heartbeat_manager.dir/build

CMakeFiles/heartbeat_manager.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/heartbeat_manager.dir/cmake_clean.cmake
.PHONY : CMakeFiles/heartbeat_manager.dir/clean

CMakeFiles/heartbeat_manager.dir/depend:
	cd /home/orangepi/paparazzi/sw/ext/sixdof_system/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/orangepi/paparazzi/sw/ext/sixdof_system/examples /home/orangepi/paparazzi/sw/ext/sixdof_system/examples /home/orangepi/paparazzi/sw/ext/sixdof_system/examples/build /home/orangepi/paparazzi/sw/ext/sixdof_system/examples/build /home/orangepi/paparazzi/sw/ext/sixdof_system/examples/build/CMakeFiles/heartbeat_manager.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/heartbeat_manager.dir/depend

