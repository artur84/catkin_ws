#include "ros/ros.h"
#include <math.h>
//msg from hdetect
#include "hdetect/HumansFeat.h"
#include "hdetect/HumansFeatClass.h"
//msg to social filter
#include "social_filter/humanPose.h"
#include "social_filter/humanPoses.h"

social_filter::humanPose local_ped;
social_filter::humanPoses list_ped;

void poseCallback(const hdetect::HumansFeatClass::ConstPtr& msg)
{
  list_ped.humans.clear();
  for(int i=0;i<msg->HumansDetected.size();i++){
    local_ped.id = msg->HumansDetected[i].id;
    local_ped.header.frame_id = "/robot_0/base_laser_link";
    //local_ped.header.stamp = 0;
    local_ped.x = msg->HumansDetected[i].x;
    local_ped.y = msg->HumansDetected[i].y;
    local_ped.theta = atan2(msg->HumansDetected[i].vely, msg->HumansDetected[i].velx); //in radians
    local_ped.linear_velocity = sqrt(pow(msg->HumansDetected[i].velx,2)+pow(msg->HumansDetected[i].vely,2));
    local_ped.angular_velocity = 0;		
    list_ped.humans.push_back(local_ped);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hdetect_to_socialfilter");
  ros::NodeHandle n;
  
  ros::Subscriber pose_sub = n.subscribe("HumansDetected", 10, poseCallback);
  ros::Publisher pose_pub = n.advertise<social_filter::humanPoses>("human_poses", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    pose_pub.publish(list_ped);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
