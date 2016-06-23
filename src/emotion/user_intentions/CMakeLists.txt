# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(user_intentions)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS 
    nav_msgs 
    nav_core 
    geometry_msgs 
    visualization_msgs 
    roslib 
    roscpp 
    rospy 
    std_msgs 
    move_base_msgs 
    actionlib 
    tf 
    rosbag 
    dynamic_reconfigure 
    cv_bridge 
    trajectory_simulator 
    controller 
    sound_play 
    social_filter 
)



#uncomment if you have defined messages
add_message_files(
    FILES 
    LocalGoal.msg 
    LocalGoalArray.msg
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})




#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/UserIntentions.cfg
)

#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES 
    geometry_msgs 
    std_msgs
)



#fill in what other packages will need to use this package
catkin_package(
    #Non-catkin CMake projects that this project depends on
    DEPENDS 
        python-numpy 
        festival 
        libgstreamer0.10-dev 
        libgstreamer-plugins-base0.10-dev 
        python-gst 
        gstreamer0.10-plugins-ugly 
        gstreamer0.10-plugins-good
         
    CATKIN_DEPENDS 
        nav_msgs 
        nav_core 
        geometry_msgs 
        visualization_msgs 
        roslib 
        roscpp 
        rospy 
        std_msgs 
        move_base_msgs 
        actionlib 
        tf 
        rosbag 
        dynamic_reconfigure 
        cv_bridge 
        trajectory_simulator 
        controller 
        sound_play 
        social_filter
    #INCLUDE_DIRS 
    #LIBRARIES 
)

# make sure configure headers are built before any node using them
add_dependencies(top_map ${PROJECT_NAME}_gencfg)
add_executable(top_map src/top_map.cpp)
