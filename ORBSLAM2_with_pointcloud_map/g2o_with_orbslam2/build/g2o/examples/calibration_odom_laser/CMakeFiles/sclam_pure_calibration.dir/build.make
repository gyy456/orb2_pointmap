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
include g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/depend.make

# Include the progress variables for this target.
include g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/flags.make

g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.o: g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/flags.make
g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.o: ../g2o/examples/calibration_odom_laser/sclam_pure_calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.o"
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/calibration_odom_laser && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.o -c /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/examples/calibration_odom_laser/sclam_pure_calibration.cpp

g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.i"
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/calibration_odom_laser && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/examples/calibration_odom_laser/sclam_pure_calibration.cpp > CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.i

g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.s"
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/calibration_odom_laser && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/examples/calibration_odom_laser/sclam_pure_calibration.cpp -o CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.s

# Object files for target sclam_pure_calibration
sclam_pure_calibration_OBJECTS = \
"CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.o"

# External object files for target sclam_pure_calibration
sclam_pure_calibration_EXTERNAL_OBJECTS =

../bin/sclam_pure_calibration: g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/sclam_pure_calibration.cpp.o
../bin/sclam_pure_calibration: g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/build.make
../bin/sclam_pure_calibration: ../lib/libg2o_calibration_odom_laser.so
../bin/sclam_pure_calibration: ../lib/libg2o_solver_csparse.so
../bin/sclam_pure_calibration: ../lib/libg2o_csparse_extension.so
../bin/sclam_pure_calibration: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/sclam_pure_calibration: ../lib/libg2o_types_sclam2d.so
../bin/sclam_pure_calibration: ../lib/libg2o_types_data.so
../bin/sclam_pure_calibration: ../lib/libg2o_types_slam2d.so
../bin/sclam_pure_calibration: ../lib/libg2o_core.so
../bin/sclam_pure_calibration: ../lib/libg2o_stuff.so
../bin/sclam_pure_calibration: ../lib/libg2o_ext_freeglut_minimal.so
../bin/sclam_pure_calibration: ../lib/libg2o_opengl_helper.so
../bin/sclam_pure_calibration: /usr/lib/x86_64-linux-gnu/libGL.so
../bin/sclam_pure_calibration: /usr/lib/x86_64-linux-gnu/libGLU.so
../bin/sclam_pure_calibration: g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../../bin/sclam_pure_calibration"
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/calibration_odom_laser && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sclam_pure_calibration.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/build: ../bin/sclam_pure_calibration

.PHONY : g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/build

g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/clean:
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/calibration_odom_laser && $(CMAKE_COMMAND) -P CMakeFiles/sclam_pure_calibration.dir/cmake_clean.cmake
.PHONY : g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/clean

g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/depend:
	cd /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2 /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/g2o/examples/calibration_odom_laser /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/calibration_odom_laser /home/gyy/Downloads/orb_map/ORBSLAM2_with_pointcloud_map/g2o_with_orbslam2/build/g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/examples/calibration_odom_laser/CMakeFiles/sclam_pure_calibration.dir/depend
