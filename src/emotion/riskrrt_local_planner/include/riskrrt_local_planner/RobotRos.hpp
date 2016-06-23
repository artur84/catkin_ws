#ifndef ROBOT_ROS_HPP
#define ROBOT_ROS_HPP

#include <ros/ros.h>
#include "robot.hpp"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class RobotRos{
public:

  RobotRos();

  virtual ~RobotRos();

  virtual void init(Robot*);

  virtual void setControl(const Control&);

  virtual Control readControl();

  /**
   * @brief Get the riskrrt robot state from ROS messages.
   * @param  global_pose: ROS tf::Stamped<tf::Pose>, contains the current global position of the robot in ROS format
   * @param base_odom: nav_msgs::Odometry, is the current odometry of the robot in ROS.
   */
  State get_riskrrt_robot_state(const tf::Stamped<tf::Pose>& global_pose,
		  const nav_msgs::Odometry & base_odom);

  void print_marker(const nav_msgs::Odometry& odom);

  void initLoc();
  State readState();
  timeval getTimeLoc();
  timeval getTimeOdo();
  bool ready_pose;
  bool ready_odom;

protected:
  ros::NodeHandle nodeHandle;
  ros::Publisher markerPublisher;
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::Twist twist_msg;
};

#endif // ROBOT_ROS_HPP
