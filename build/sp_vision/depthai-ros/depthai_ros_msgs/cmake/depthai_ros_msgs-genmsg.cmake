# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "depthai_ros_msgs: 5 messages, 2 services")

set(MSG_I_FLAGS "-Idepthai_ros_msgs:/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Ivision_msgs:/opt/ros/noetic/share/vision_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(depthai_ros_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg" NAME_WE)
add_custom_target(_depthai_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "depthai_ros_msgs" "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg" ""
)

get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg" NAME_WE)
add_custom_target(_depthai_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "depthai_ros_msgs" "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg" "geometry_msgs/Pose2D:geometry_msgs/Point"
)

get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg" NAME_WE)
add_custom_target(_depthai_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "depthai_ros_msgs" "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg" "std_msgs/Header:depthai_ros_msgs/HandLandmark:geometry_msgs/Pose2D:geometry_msgs/Point"
)

get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg" NAME_WE)
add_custom_target(_depthai_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "depthai_ros_msgs" "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg" "vision_msgs/BoundingBox2D:geometry_msgs/Pose2D:vision_msgs/ObjectHypothesis:geometry_msgs/Point"
)

get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg" NAME_WE)
add_custom_target(_depthai_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "depthai_ros_msgs" "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg" "geometry_msgs/Point:vision_msgs/ObjectHypothesis:std_msgs/Header:geometry_msgs/Pose2D:vision_msgs/BoundingBox2D:depthai_ros_msgs/SpatialDetection"
)

get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv" NAME_WE)
add_custom_target(_depthai_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "depthai_ros_msgs" "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv" ""
)

get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv" NAME_WE)
add_custom_target(_depthai_ros_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "depthai_ros_msgs" "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv" "geometry_msgs/Pose2D"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_cpp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_cpp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_cpp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_cpp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Services
_generate_srv_cpp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_srv_cpp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Module File
_generate_module_cpp(depthai_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(depthai_ros_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(depthai_ros_msgs_generate_messages depthai_ros_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_cpp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_cpp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_cpp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_cpp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_cpp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_cpp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_cpp _depthai_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depthai_ros_msgs_gencpp)
add_dependencies(depthai_ros_msgs_gencpp depthai_ros_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depthai_ros_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_eus(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_eus(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_eus(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_eus(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Services
_generate_srv_eus(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
)
_generate_srv_eus(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Module File
_generate_module_eus(depthai_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(depthai_ros_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(depthai_ros_msgs_generate_messages depthai_ros_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_eus _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_eus _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_eus _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_eus _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_eus _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_eus _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_eus _depthai_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depthai_ros_msgs_geneus)
add_dependencies(depthai_ros_msgs_geneus depthai_ros_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depthai_ros_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_lisp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_lisp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_lisp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_lisp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Services
_generate_srv_lisp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
)
_generate_srv_lisp(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Module File
_generate_module_lisp(depthai_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(depthai_ros_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(depthai_ros_msgs_generate_messages depthai_ros_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_lisp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_lisp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_lisp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_lisp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_lisp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_lisp _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_lisp _depthai_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depthai_ros_msgs_genlisp)
add_dependencies(depthai_ros_msgs_genlisp depthai_ros_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depthai_ros_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_nodejs(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_nodejs(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_nodejs(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_nodejs(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Services
_generate_srv_nodejs(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
)
_generate_srv_nodejs(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Module File
_generate_module_nodejs(depthai_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(depthai_ros_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(depthai_ros_msgs_generate_messages depthai_ros_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_nodejs _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_nodejs _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_nodejs _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_nodejs _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_nodejs _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_nodejs _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_nodejs _depthai_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depthai_ros_msgs_gennodejs)
add_dependencies(depthai_ros_msgs_gennodejs depthai_ros_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depthai_ros_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_py(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_py(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_py(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
)
_generate_msg_py(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/ObjectHypothesis.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/noetic/share/vision_msgs/cmake/../msg/BoundingBox2D.msg;/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Services
_generate_srv_py(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
)
_generate_srv_py(depthai_ros_msgs
  "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose2D.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
)

### Generating Module File
_generate_module_py(depthai_ros_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(depthai_ros_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(depthai_ros_msgs_generate_messages depthai_ros_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/AutoFocusCtrl.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_py _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmark.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_py _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/HandLandmarkArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_py _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetection.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_py _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/msg/SpatialDetectionArray.msg" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_py _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/TriggerNamed.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_py _depthai_ros_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_vision/depthai-ros/depthai_ros_msgs/srv/NormalizedImageCrop.srv" NAME_WE)
add_dependencies(depthai_ros_msgs_generate_messages_py _depthai_ros_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(depthai_ros_msgs_genpy)
add_dependencies(depthai_ros_msgs_genpy depthai_ros_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS depthai_ros_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/depthai_ros_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(depthai_ros_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(depthai_ros_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(depthai_ros_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET vision_msgs_generate_messages_cpp)
  add_dependencies(depthai_ros_msgs_generate_messages_cpp vision_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/depthai_ros_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(depthai_ros_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(depthai_ros_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(depthai_ros_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET vision_msgs_generate_messages_eus)
  add_dependencies(depthai_ros_msgs_generate_messages_eus vision_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/depthai_ros_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(depthai_ros_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(depthai_ros_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(depthai_ros_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET vision_msgs_generate_messages_lisp)
  add_dependencies(depthai_ros_msgs_generate_messages_lisp vision_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/depthai_ros_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(depthai_ros_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(depthai_ros_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(depthai_ros_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET vision_msgs_generate_messages_nodejs)
  add_dependencies(depthai_ros_msgs_generate_messages_nodejs vision_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/depthai_ros_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(depthai_ros_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(depthai_ros_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(depthai_ros_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET vision_msgs_generate_messages_py)
  add_dependencies(depthai_ros_msgs_generate_messages_py vision_msgs_generate_messages_py)
endif()
