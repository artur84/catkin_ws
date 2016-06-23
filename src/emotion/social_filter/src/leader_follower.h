#ifndef HUMAN_PROC_H
#define HUMAN_PROC_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>

#include "std_msgs/String.h"
#include "social_filter/humanPose.h" //this is for the msg defined
#include "social_filter/humanPoses.h"

#include "trajectory_simulator/TrajectoryObservation.h"

geometry_msgs::PoseStamped leader_pose;
geometry_msgs::PoseStamped leader_goal[4];
geometry_msgs::PoseStamped robot_goal;
geometry_msgs::PoseStamped next_pose;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
move_base_msgs::MoveBaseGoal goal;
nav_msgs::Path leader_path;
geometry_msgs::Twist cmd_vel;
trajectory_simulator::TrajectoryObservation candidate_; //number of markers

tf::TransformListener* listener = NULL;
tf::StampedTransform transform;
geometry_msgs::PoseStamped source_pose;
geometry_msgs::PoseStamped target_pose;

ros::Subscriber pose_subscriber;
ros::Subscriber ar_pose_sub;
ros::Subscriber dyn_objects_subscriber;
ros::Subscriber leader_goal_subscriber;
ros::Subscriber robot_goal_subscriber;
ros::Subscriber robot_pose_subscriber;

ros::Publisher pathPublisher;
ros::Publisher robot_cmd_vel;
ros::Publisher next_posePublisher;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int num_markers;
int marker_id, marker_type;
uint leader_id = 99; //must be different from any id at initialization
double goals_distance = 99;
double robot2goal_distance, leader2goal_distance;
bool path_initialized = false;
bool send_goal = true;
bool start_to_follow = false;
bool leader_found = false;
double dist, theta, theta_max = M_PI, dmax = 15.0, dmin = 1.5;
double d , tnow, tlast;

void target_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
void ar_pose_callback(const ar_pose::ARMarkers::ConstPtr& markers);
void dyn_objects_callback(const trajectory_simulator::TrajectoryObservation::ConstPtr & dyn_objects);
void leader_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void robot_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg);
void robot_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg);

double euclidean_dist(double x1,double y1,double x2, double y2){
  double dist2 = ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2));
  return  (sqrt(dist2));
}


#endif // HUMAN_PROC_H

