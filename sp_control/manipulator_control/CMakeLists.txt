cmake_minimum_required(VERSION 3.10.2)
add_compile_options(-std=c++14)
project(manipulator_control)

# Common cmake code applied to all moveit packages
find_package(catkin REQUIRED COMPONENTS
moveit_ros_planning_interface
moveit_ros_move_group
sp_common
moveit_visual_tools
)
find_package(Eigen3 REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)

find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mrobot_bringup
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
set(SRC_LIST
      src/${PROJECT_NAME}.cpp
      src/arm_control.cpp
      src/scene_generate.cpp
      )


add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES})

