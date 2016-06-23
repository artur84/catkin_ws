cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()

find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

#rosbuild_add_boost_directories()

rosbuild_add_library(social_costmap src/social_costmap.cpp src/observation_buffer.cpp src/social_costmap_ros.cpp src/social_costmap_publisher.cpp src/voxel_social_costmap.cpp)
target_link_libraries(social_costmap ${PCL_LIBRARIES})
#rosbuild_link_boost(social_costmap thread)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_executable(bin/social_costmap_markers src/social_costmap_markers.cpp)
rosbuild_add_executable(bin/social_costmap_cloud src/social_costmap_cloud.cpp)
rosbuild_add_executable(bin/social_grid src/social_grid.cpp)

rosbuild_add_executable(bin/social_costmap_node src/social_costmap_node.cpp)
target_link_libraries(bin/social_costmap_node social_costmap)

rosbuild_add_executable(test/costmap_tester test/costmap_tester.cpp)
target_link_libraries(test/costmap_tester social_costmap gtest ${PCL_LIBRARIES})

rosbuild_download_test_data(http://pr.willowgarage.com/data/social_costmap/simple_driving_test_indexed.bag test/simple_driving_test_indexed.bag 61168cff9425b11e093ea3a627c81c8d)
rosbuild_download_test_data(http://pr.willowgarage.com/data/social_costmap/willow-full-0.025.pgm test/willow-full-0.025.pgm e66b17ee374f2d7657972efcb3e2e4f7)

rosbuild_add_rostest(test/simple_driving_test.xml)

rosbuild_add_gtest(test/module_tests test/module_tests.cpp)
target_link_libraries(test/module_tests social_costmap)
