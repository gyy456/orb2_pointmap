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
CMAKE_SOURCE_DIR = /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build

# Include any dependencies generated for this target.
include g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/depend.make

# Include the progress variables for this target.
include g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/flags.make

g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.o: g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/flags.make
g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.o: ../g2o/examples/interactive_slam/g2o_incremental/g2o_incremental.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.o"
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/interactive_slam/g2o_incremental && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.o -c /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/examples/interactive_slam/g2o_incremental/g2o_incremental.cpp

g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.i"
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/interactive_slam/g2o_incremental && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/examples/interactive_slam/g2o_incremental/g2o_incremental.cpp > CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.i

g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.s"
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/interactive_slam/g2o_incremental && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/examples/interactive_slam/g2o_incremental/g2o_incremental.cpp -o CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.s

# Object files for target g2o_incremental_application
g2o_incremental_application_OBJECTS = \
"CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.o"

# External object files for target g2o_incremental_application
g2o_incremental_application_EXTERNAL_OBJECTS =

../bin/g2o_incremental: g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/g2o_incremental.cpp.o
../bin/g2o_incremental: g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/build.make
../bin/g2o_incremental: ../lib/libg2o_incremental.so
../bin/g2o_incremental: ../lib/libg2o_interactive.so
../bin/g2o_incremental: ../lib/libg2o_types_slam2d.so
../bin/g2o_incremental: ../lib/libg2o_types_slam3d.so
../bin/g2o_incremental: ../lib/libg2o_opengl_helper.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/g2o_incremental: ../lib/libg2o_interface.so
../bin/g2o_incremental: ../lib/libg2o_parser.so
../bin/g2o_incremental: ../lib/libg2o_solver_cholmod.so
../bin/g2o_incremental: ../lib/libg2o_core.so
../bin/g2o_incremental: ../lib/libg2o_stuff.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libcholmod.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libamd.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libcolamd.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libccolamd.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libblas.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/liblapack.so
../bin/g2o_incremental: /usr/lib/x86_64-linux-gnu/libcamd.so
../bin/g2o_incremental: g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../../../bin/g2o_incremental"
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/interactive_slam/g2o_incremental && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/g2o_incremental_application.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/build: ../bin/g2o_incremental

.PHONY : g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/build

g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/clean:
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/interactive_slam/g2o_incremental && $(CMAKE_COMMAND) -P CMakeFiles/g2o_incremental_application.dir/cmake_clean.cmake
.PHONY : g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/clean

g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/depend:
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2 /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/examples/interactive_slam/g2o_incremental /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/interactive_slam/g2o_incremental /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/examples/interactive_slam/g2o_incremental/CMakeFiles/g2o_incremental_application.dir/depend

