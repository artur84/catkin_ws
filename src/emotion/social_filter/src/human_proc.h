#ifndef HUMAN_PROC_H
#define HUMAN_PROC_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "social_filter/humanPose.h" //this is for the msg defined
#include "social_filter/humanPoses.h"


class odom_reader
{ 
public:
  odom_reader(std_msgs::String topic_name,int id,ros::NodeHandle* n);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void init();
  social_filter::humanPose getPose();
  ros::NodeHandle *local_n;
  ros::Subscriber odom_sub;
  social_filter::humanPose local_ped;
  std_msgs::String name;
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


#endif // HUMAN_PROC_H

