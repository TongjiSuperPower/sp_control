# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "depthai_ros_msgs;camera_info_manager;roscpp;sensor_msgs;std_msgs;vision_msgs;image_transport;cv_bridge;stereo_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ldepthai_bridge".split(';') if "-ldepthai_bridge" != "" else []
PROJECT_NAME = "depthai_bridge"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "2.6.4"
