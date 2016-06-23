#ifndef PAL_HUMAN_PROC_H
#define PAL_HUMAN_PROC_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include "social_filter/humanPose.h" //this is for the msg defined
#include "social_filter/humanPoses.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"

class PALHumanProc
{
public:

  PALHumanProc(int n); //This n is the identifier for each person
  ~PALHumanProc();
  void init();
  void pub();
  tf::TransformListener tf_listener;
  tf::StampedTransform transform_user; //To get the transformation relating /world and user1
  tf::StampedTransform transform_user_past;
  geometry_msgs::Twist velocity;
  social_filter::humanPose local_ped;
  double previous_x;
  double previous_y;
  ros::Time previous_stamp;
  ros::Duration d;
  tf::Point p;

protected:
  ros::NodeHandle n;
  ros::Publisher pose_pub;
  social_filter::humanPose ped;
  social_filter::humanPoses list_ped;
  unsigned int NoH;

};

#endif // PAL_HUMAN_PROC_H
