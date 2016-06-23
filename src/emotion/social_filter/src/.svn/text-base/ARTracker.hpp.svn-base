#ifndef __AR_TRACKER_HPP
#define __AR_TRACKER_HPP
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>
#include <std_msgs/String.h>

#include <social_filter/humanPose.h>
#include <social_filter/humanPoses.h>
#include <trajectory_simulator/TrajectoryObservation.h>
#include <geometry_msgs/PoseArray.h>

#include "ARTarget.hpp"

#include <map>

class ARTracker
{
protected:
  std::map<int, ARTarget> targetList;
  ros::NodeHandle nodeHandle;
  tf::TransformListener listener;
  ros::Subscriber arSub;
  ros::Publisher trajectoryPub;
  ros::Publisher posePub;
  ros::Publisher poseArrayPub;
  
public:
  ARTracker();
  
  void loop();

  void ARCallback(const ar_pose::ARMarkers::ConstPtr& markers);
};

#endif // __AR_TRACKER_HPP
