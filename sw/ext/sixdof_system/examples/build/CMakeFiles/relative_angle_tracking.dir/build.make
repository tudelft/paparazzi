# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ppz/paparazzi/sw/ext/sixdof_system/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ppz/paparazzi/sw/ext/sixdof_system/examples/build

# Include any dependencies generated for this target.
include CMakeFiles/relative_angle_tracking.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/relative_angle_tracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/relative_angle_tracking.dir/flags.make

CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.o: CMakeFiles/relative_angle_tracking.dir/flags.make
CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.o: ../relative_angle_tracking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ppz/paparazzi/sw/ext/sixdof_system/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.o -c /home/ppz/paparazzi/sw/ext/sixdof_system/examples/relative_angle_tracking.cpp

CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ppz/paparazzi/sw/ext/sixdof_system/examples/relative_angle_tracking.cpp > CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.i

CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ppz/paparazzi/sw/ext/sixdof_system/examples/relative_angle_tracking.cpp -o CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.s

# Object files for target relative_angle_tracking
relative_angle_tracking_OBJECTS = \
"CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.o"

# External object files for target relative_angle_tracking
relative_angle_tracking_EXTERNAL_OBJECTS =

relative_angle_tracking: CMakeFiles/relative_angle_tracking.dir/relative_angle_tracking.cpp.o
relative_angle_tracking: CMakeFiles/relative_angle_tracking.dir/build.make
relative_angle_tracking: CMakeFiles/relative_angle_tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ppz/paparazzi/sw/ext/sixdof_system/examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable relative_angle_tracking"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/relative_angle_tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/relative_angle_tracking.dir/build: relative_angle_tracking

.PHONY : CMakeFiles/relative_angle_tracking.dir/build

CMakeFiles/relative_angle_tracking.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/relative_angle_tracking.dir/cmake_clean.cmake
.PHONY : CMakeFiles/relative_angle_tracking.dir/clean

CMakeFiles/relative_angle_tracking.dir/depend:
	cd /home/ppz/paparazzi/sw/ext/sixdof_system/examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ppz/paparazzi/sw/ext/sixdof_system/examples /home/ppz/paparazzi/sw/ext/sixdof_system/examples /home/ppz/paparazzi/sw/ext/sixdof_system/examples/build /home/ppz/paparazzi/sw/ext/sixdof_system/examples/build /home/ppz/paparazzi/sw/ext/sixdof_system/examples/build/CMakeFiles/relative_angle_tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/relative_angle_tracking.dir/depend

