cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rosbuild_init()
rosbuild_genmsg()

rosbuild_add_boost_directories()

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

#Add RiskRRT code
rosbuild_find_ros_package(RiskRRT REQUIRED)

include_directories(${RiskRRT_PACKAGE_PATH}/src/common
                    ${RiskRRT_PACKAGE_PATH}/src/gppredict
                    ${RiskRRT_PACKAGE_PATH}/src/planning
                    ${RiskRRT_PACKAGE_PATH}/src/roscommon
                    ${RiskRRT_PACKAGE_PATH}/src/rosdemo
                    ${RiskRRT_PACKAGE_PATH}/src/gppredict/gp
)

rosbuild_find_ros_package(social_filter)
include_directories (${social_filter_PACKAGE_PATH}/src/)
link_directories(${social_filter_PACKAGE_PATH}/lib/)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
include_directories(SYSTEM 
                    ${EIGEN_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${SOCIAL_INCLUDE_DIRS}
                    ${RISKRRT_INCLUDE_DIRS}
                    
)
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_library(riskrrt_local_planner 
	src/footprint_helper.cpp
	src/goal_functions.cpp
	src/map_cell.cpp
	src/map_grid.cpp
	src/map_grid_visualizer.cpp
	src/map_grid_cost_function.cpp
	src/latched_stop_rotate_controller.cpp
	src/local_planner_util.cpp
	src/odometry_helper_ros.cpp
	src/obstacle_cost_function.cpp
	src/oscillation_cost_function.cpp
	src/prefer_forward_cost_function.cpp
	src/point_grid.cpp
	src/costmap_model.cpp
	src/simple_scored_sampling_planner.cpp
	src/simple_trajectory_generator.cpp
	src/trajectory_planner.cpp
	src/trajectory.cpp
	src/trajectory_planner_ros.cpp
	src/trajectory.cpp
	src/user_vel_handler.cpp
	src/voxel_grid_model.cpp)
rosbuild_link_boost(riskrrt_local_planner thread)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_executable(point_grid src/point_grid.cpp)
