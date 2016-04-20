#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>

std::string mi_string ("");
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  mi_string.assign(msg->data.c_str());
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
