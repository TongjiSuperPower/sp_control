# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "sp_common: 3 messages, 0 services")

set(MSG_I_FLAGS "-Isp_common:/home/adminpc/catkin_ws/src/sp_common/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(sp_common_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg" NAME_WE)
add_custom_target(_sp_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sp_common" "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg" ""
)

get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg" NAME_WE)
add_custom_target(_sp_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sp_common" "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg" NAME_WE)
add_custom_target(_sp_common_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "sp_common" "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_common
)
_generate_msg_cpp(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_common
)
_generate_msg_cpp(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_common
)

### Generating Services

### Generating Module File
_generate_module_cpp(sp_common
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_common
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(sp_common_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(sp_common_generate_messages sp_common_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_cpp _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_cpp _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_cpp _sp_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sp_common_gencpp)
add_dependencies(sp_common_gencpp sp_common_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sp_common_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sp_common
)
_generate_msg_eus(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sp_common
)
_generate_msg_eus(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sp_common
)

### Generating Services

### Generating Module File
_generate_module_eus(sp_common
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sp_common
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(sp_common_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(sp_common_generate_messages sp_common_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_eus _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_eus _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_eus _sp_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sp_common_geneus)
add_dependencies(sp_common_geneus sp_common_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sp_common_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_common
)
_generate_msg_lisp(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_common
)
_generate_msg_lisp(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_common
)

### Generating Services

### Generating Module File
_generate_module_lisp(sp_common
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_common
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(sp_common_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(sp_common_generate_messages sp_common_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_lisp _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_lisp _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_lisp _sp_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sp_common_genlisp)
add_dependencies(sp_common_genlisp sp_common_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sp_common_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sp_common
)
_generate_msg_nodejs(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sp_common
)
_generate_msg_nodejs(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sp_common
)

### Generating Services

### Generating Module File
_generate_module_nodejs(sp_common
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sp_common
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(sp_common_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(sp_common_generate_messages sp_common_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_nodejs _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_nodejs _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_nodejs _sp_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sp_common_gennodejs)
add_dependencies(sp_common_gennodejs sp_common_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sp_common_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_common
)
_generate_msg_py(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_common
)
_generate_msg_py(sp_common
  "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_common
)

### Generating Services

### Generating Module File
_generate_module_py(sp_common
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_common
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(sp_common_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(sp_common_generate_messages sp_common_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/ActuatorState.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_py _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/GpioData.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_py _sp_common_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/adminpc/catkin_ws/src/sp_common/msg/SingleJointWrite.msg" NAME_WE)
add_dependencies(sp_common_generate_messages_py _sp_common_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(sp_common_genpy)
add_dependencies(sp_common_genpy sp_common_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS sp_common_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/sp_common
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(sp_common_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sp_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/sp_common
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(sp_common_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/sp_common
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(sp_common_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sp_common)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/sp_common
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(sp_common_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_common)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_common\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/sp_common
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(sp_common_generate_messages_py std_msgs_generate_messages_py)
endif()
