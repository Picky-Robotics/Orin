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
CMAKE_SOURCE_DIR = /home/orin/Orin/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/orin/Orin/catkin_ws/build

# Include any dependencies generated for this target.
include geometric_shapes/test/CMakeFiles/test_create_mesh.dir/depend.make

# Include the progress variables for this target.
include geometric_shapes/test/CMakeFiles/test_create_mesh.dir/progress.make

# Include the compile flags for this target's objects.
include geometric_shapes/test/CMakeFiles/test_create_mesh.dir/flags.make

geometric_shapes/test/CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.o: geometric_shapes/test/CMakeFiles/test_create_mesh.dir/flags.make
geometric_shapes/test/CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.o: /home/orin/Orin/catkin_ws/src/geometric_shapes/test/test_create_mesh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/orin/Orin/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object geometric_shapes/test/CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.o"
	cd /home/orin/Orin/catkin_ws/build/geometric_shapes/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.o -c /home/orin/Orin/catkin_ws/src/geometric_shapes/test/test_create_mesh.cpp

geometric_shapes/test/CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.i"
	cd /home/orin/Orin/catkin_ws/build/geometric_shapes/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/orin/Orin/catkin_ws/src/geometric_shapes/test/test_create_mesh.cpp > CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.i

geometric_shapes/test/CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.s"
	cd /home/orin/Orin/catkin_ws/build/geometric_shapes/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/orin/Orin/catkin_ws/src/geometric_shapes/test/test_create_mesh.cpp -o CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.s

# Object files for target test_create_mesh
test_create_mesh_OBJECTS = \
"CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.o"

# External object files for target test_create_mesh
test_create_mesh_EXTERNAL_OBJECTS =

/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: geometric_shapes/test/CMakeFiles/test_create_mesh.dir/test_create_mesh.cpp.o
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: geometric_shapes/test/CMakeFiles/test_create_mesh.dir/build.make
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: gtest/lib/libgtest.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /home/orin/Orin/catkin_ws/devel/lib/libgeometric_shapes.so.0.7.5
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/librandom_numbers.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/libresource_retriever.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/librostime.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/libcpp_common.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libassimp.so.5
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libqhull_r.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/librandom_numbers.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/libresource_retriever.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/librostime.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/libcpp_common.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/aarch64-linux-gnu/libfcl.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libccd.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /usr/lib/aarch64-linux-gnu/libm.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/liboctomap.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: /opt/ros/noetic/lib/liboctomath.so
/home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh: geometric_shapes/test/CMakeFiles/test_create_mesh.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/orin/Orin/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh"
	cd /home/orin/Orin/catkin_ws/build/geometric_shapes/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_create_mesh.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
geometric_shapes/test/CMakeFiles/test_create_mesh.dir/build: /home/orin/Orin/catkin_ws/devel/lib/geometric_shapes/test_create_mesh

.PHONY : geometric_shapes/test/CMakeFiles/test_create_mesh.dir/build

geometric_shapes/test/CMakeFiles/test_create_mesh.dir/clean:
	cd /home/orin/Orin/catkin_ws/build/geometric_shapes/test && $(CMAKE_COMMAND) -P CMakeFiles/test_create_mesh.dir/cmake_clean.cmake
.PHONY : geometric_shapes/test/CMakeFiles/test_create_mesh.dir/clean

geometric_shapes/test/CMakeFiles/test_create_mesh.dir/depend:
	cd /home/orin/Orin/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/orin/Orin/catkin_ws/src /home/orin/Orin/catkin_ws/src/geometric_shapes/test /home/orin/Orin/catkin_ws/build /home/orin/Orin/catkin_ws/build/geometric_shapes/test /home/orin/Orin/catkin_ws/build/geometric_shapes/test/CMakeFiles/test_create_mesh.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : geometric_shapes/test/CMakeFiles/test_create_mesh.dir/depend

