# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;sp_common;pluginlib;controller_interface;realtime_tools".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lgpio_controller".split(';') if "-lgpio_controller" != "" else []
PROJECT_NAME = "gpio_controller"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.1.7"
