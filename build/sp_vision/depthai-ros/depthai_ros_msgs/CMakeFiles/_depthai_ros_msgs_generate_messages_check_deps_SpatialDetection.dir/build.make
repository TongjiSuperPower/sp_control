# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/adminpc/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/adminpc/catkin_ws/src/build

# Utility rule file for _depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.

# Include any custom commands dependencies for this target.
include sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/compiler_depend.make

# Include the progress variables for this target.
include sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/progress.make

sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection:
	cd /home/adminpc/catkin_ws/src/build/sp_vision/depthai-ros/depthai_ros_msgs && ../../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genmsg/cmake/../../../lib/genmsg/genmsg_check_deps.py depthai_ros_msgs /home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg vision_msgs/BoundingBox2D:geometry_msgs/Pose2D:vision_msgs/ObjectHypothesis:geometry_msgs/Point

_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection: sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection
_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection: sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/build.make
.PHONY : _depthai_ros_msgs_generate_messages_check_deps_SpatialDetection

# Rule to build all files generated by this target.
sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/build: _depthai_ros_msgs_generate_messages_check_deps_SpatialDetection
.PHONY : sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/build

sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/clean:
	cd /home/adminpc/catkin_ws/src/build/sp_vision/depthai-ros/depthai_ros_msgs && $(CMAKE_COMMAND) -P CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/cmake_clean.cmake
.PHONY : sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/clean

sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/depend:
	cd /home/adminpc/catkin_ws/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/adminpc/catkin_ws/src /home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs /home/adminpc/catkin_ws/src/build /home/adminpc/catkin_ws/src/build/sp_vision/depthai-ros/depthai_ros_msgs /home/adminpc/catkin_ws/src/build/sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sp_vision/depthai-ros/depthai_ros_msgs/CMakeFiles/_depthai_ros_msgs_generate_messages_check_deps_SpatialDetection.dir/depend

