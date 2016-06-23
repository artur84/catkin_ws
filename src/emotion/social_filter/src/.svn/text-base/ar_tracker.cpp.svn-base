#include "ARTracker.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ar_tracker");
  ros::start();

  ros::Rate loop_rate(10);
  
  ARTracker tracker;
  
  while (ros::ok())
  {      
    ros::spinOnce();
    loop_rate.sleep();

    tracker.loop();
  }
}
