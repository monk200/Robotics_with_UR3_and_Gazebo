# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "image_transport;roscpp;cv_bridge;sensor_msgs;nodelet;camera_info_manager".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lcv_camera".split(';') if "-lcv_camera" != "" else []
PROJECT_NAME = "cv_camera"
PROJECT_SPACE_DIR = "/home/ur3/Tower_of_Hanoi/install"
PROJECT_VERSION = "0.1.0"
