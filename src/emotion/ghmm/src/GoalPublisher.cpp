#include "GoalPublisher.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <iomanip>

using namespace ghmm_ros;

GoalPublisher::GoalPublisher(
  const std::string & topic, 
  ros::NodeHandle & node,
  float x, 
  float y
) : goal_topic_( topic ),
    x_( x ),
    y_( y ),
    goal_publisher_( node.advertise<geometry_msgs::PoseStamped>(topic, 1000)),
    goal_publisher0_( node.advertise<geometry_msgs::PoseStamped>("leader_goal_pose_0", 10)),
    goal_publisher1_( node.advertise<geometry_msgs::PoseStamped>("leader_goal_pose_1", 10)),
    goal_publisher2_( node.advertise<geometry_msgs::PoseStamped>("leader_goal_pose_2", 10))
{}

void 
GoalPublisher::operator()(uint32_t id, double traj_num, double x_at_max, double y_at_max)
{

  geometry_msgs::PoseStamped goal_pose;

//   ROS_INFO("new max sum:%f @ x:%f, y:%f",max_p,x_at_max,y_at_max);

  //geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "/map";
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.pose.position.x = x_at_max;
  goal_pose.pose.position.y = y_at_max;
//   goal_pose.pose.position.z = id; //use height to store id TODO a better way
  goal_pose.pose.position.z = traj_num; //use height to store id TODO a better way
  goal_publisher_.publish( goal_pose );
  
  //publish individual goals
  switch(id){
  case 0:
    goal_publisher0_.publish( goal_pose );
    break;
  case 1:
    goal_publisher1_.publish( goal_pose );
    break;
  case 2:
    goal_publisher2_.publish( goal_pose );
    break;
  }
}

