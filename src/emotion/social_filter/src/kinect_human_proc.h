#ifndef KINECT_HUMAN_PROC_H
#define KINECT_HUMAN_PROC_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include "social_filter/humanPose.h" //this is for the msg defined
#include "social_filter/humanPoses.h"
#include "tf/tf.h"

class KhumanProc {
public:

	KhumanProc(int n); //This n is the identifier for each person
	~KhumanProc();
	void init();
	void pub();
	tf::TransformListener kinect_listener;
	tf::StampedTransform transform_user; //To get the transformation relating /world and user1
	social_filter::humanPose local_ped;
  

protected:
	ros::NodeHandle n;
	ros::Publisher pose_pub;
	social_filter::humanPose ped;
	social_filter::humanPoses list_ped;unsigned int NoH;

};

#endif // KINECT_HUMAN_PROC_H
