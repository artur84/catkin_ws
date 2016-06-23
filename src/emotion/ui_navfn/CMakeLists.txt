cmake_minimum_required (VERSION 2.4.6)

### If you're trying to debug include directives or so, this here
### might help:
#set (CMAKE_VERBOSE_MAKEFILE ON)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

### Debug mode implies slower runtime, "of course"
#set (ROS_BUILD_TYPE Debug)
set (ROS_BUILD_TYPE RelWithDebInfo)

add_definitions (-Wall)
rosbuild_init()
rosbuild_gensrv()
rosbuild_genmsg()

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}
                           ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_library (ui_navfn src/ui_navfn.cpp src/ui_navfn_ros.cpp)

target_link_libraries(ui_navfn)




