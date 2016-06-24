#ifndef HUMAN_PROC_H
#define HUMAN_PROC_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "social_filter/humanPose.h" //this is for the msg defined
#include "social_filter/humanPoses.h"

#include <human_leader/TrajectoryObservation.h>

class odom_reader
{ 
public:
  odom_reader(std_msgs::String odom_topic, std_msgs::String pose_topic, int id, ros::NodeHandle* n);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void init();
  social_filter::humanPose getPose();
  ros::NodeHandle *local_n;
  ros::Subscriber odom_sub;
  ros::Subscriber pose_sub;
  social_filter::humanPose local_ped;
  human_leader::TrajectoryObservation ghmm_wrapper;
  std_msgs::String odom_name;
  std_msgs::String pose_name;
  bool ready;
};

class humanProc
{
 public:
  
  humanProc(int n); 
  ~humanProc();  
  void init();
  void pub();
  std::vector<odom_reader*> readers;
  bool ready();

 protected:
  ros::NodeHandle n;
  ros::Publisher pose_pub;
  social_filter::humanPose ped;
  social_filter::humanPoses list_ped;
  nav_msgs::Odometry odom_msg;
  unsigned int NoH; 
};

// struct ar_management{
//   ros::Time detect_time;
//   human_leader::TrajectoryObservation ghmm_wrapper;
// };

// ar_management markers_list[10];

ros::Publisher trajectory_pub;
int list_size = 10;

#endif // HUMAN_PROC_H

