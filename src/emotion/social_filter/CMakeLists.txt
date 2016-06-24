cmake_minimum_required(VERSION 2.8.3)
project(social_filter)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
    std_msgs 
    roscpp 
    rospy 
    visualization_msgs 
    nav_msgs 
    tf 
    trajectory_simulator 
    move_base_msgs 
    actionlib 
    message_generation
)
    

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(
  FILES
    humanPose.msg
    humanPoses.msg
    humanSocialSpace.msg
    humanSocialSpaces.msg
    int_data.msg
    int_list.msg
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#add_executable(social_fcns src/social_fcns.cpp)
#target_link_libraries(social_fcns ${catkin_LIBRARIES})
#add_dependencies(social_fcns social_filter_gencpp)

#add_executable(human_proc  src/human_proc.cpp)
#target_link_libraries(human_proc ${catkin_LIBRARIES})
#add_dependencies(human_proc social_filter_gencpp)

#add_executable(fform_detect src/fform_detect.cpp)
#target_link_libraries(fform_detect ${catkin_LIBRARIES})
#add_dependencies(fform_detect social_filter_gencpp)

#add_executable(human_markers src/human_markers.cpp)
#target_link_libraries(human_markers ${catkin_LIBRARIES})
#add_dependencies(human_markers social_filter_gencpp)

#add_executable(kinect_human_proc src/kinect_human_proc.cpp)
#target_link_libraries(kinect_human_proc ${catkin_LIBRARIES})
#add_dependencies(kinect_human_proc social_filter_gencpp)



#rosbuild_find_ros_package(ghmm)
#include_directories (${ghmm_PACKAGE_PATH}/src/)
#link_directories(${ghmm_PACKAGE_PATH}/lib/)
#add_executable(human_grids src/HumanGridRos.cpp)
#target_link_libraries(human_grids social_fcns GridPublisher)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
     std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# fill in what other packages will need to use this package
catkin_package(
    #DEPENDS 
    CATKIN_DEPENDS 
        std_msgs 
        roscpp 
        rospy 
        visualization_msgs 
        nav_msgs 
        tf 
        trajectory_simulator 
        move_base_msgs 
        actionlib 
        message_runtime
    #INCLUDE_DIRS # TODO include
    #LIBRARIES # TODO
)
