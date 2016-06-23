cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

find_package(Eigen)
find_package(PCL)
include_directories(${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(${ui_local_planner})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_boost_directories()

# move_base
rosbuild_add_executable(bin/ui_move_base src/ui_move_base.cpp)
rosbuild_link_boost(bin/ui_move_base system)
