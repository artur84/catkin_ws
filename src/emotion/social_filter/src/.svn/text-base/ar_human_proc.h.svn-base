#ifndef AR_HUMAN_PROC_H
#define AR_HUMAN_PROC_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ar_pose/ARMarkers.h>
#include <ar_pose/ARMarker.h>

#include "std_msgs/String.h"

#include "social_filter/humanPose.h" //this is for the msg defined
#include "social_filter/humanPoses.h"

#include "trajectory_simulator/TrajectoryObservation.h"

social_filter::humanPose ped;
social_filter::humanPoses list_ped;
ros::Publisher trajectory_pub;
int marker_id, marker_type;
ros::Duration duration_since_detection;
int list_size = 10;

struct ar_management{
  ros::Time detect_time;
  trajectory_simulator::TrajectoryObservation ghmm_wrapper;
};

ar_management markers_list[10];

void manage_list(int marker_id);

double euclidean_dist(double x1,double y1,double x2, double y2){
  double dist2 = ((x1-x2)*(x1-x2))+((y1-y2)*(y1-y2));
  return  (sqrt(dist2));
}

class ar_pose_reader
{ 
public:
  ar_pose_reader(std_msgs::String topic_name, ros::NodeHandle* n);
  void ar_pose_callback(const ar_pose::ARMarkers::ConstPtr&); //for multi markers
  void init();
  social_filter::humanPose getPose();
  ros::NodeHandle *local_n;
  ros::Subscriber ar_pose_sub;
  social_filter::humanPose local_ped;
  std_msgs::String name;
  int num_markers;
  
  tf::TransformListener listener;
  geometry_msgs::PoseStamped source_pose;
  geometry_msgs::PoseStamped target_pose;
};

class ar_humanProc
{
 public:
  
  ar_humanProc(); 
  ~ar_humanProc();  
  void init();
  void pub();
  std::vector<double> vec_x;
  std::vector<double> vec_y;
  std::vector<double> vec_theta;
  std::vector<ar_pose_reader*> readers;

 protected:
  ros::NodeHandle n;
  ros::Publisher pose_pub;
  ar_pose::ARMarkers ar_pose_msg; //for multi markers
  unsigned int init_index;
  unsigned int NoH;
};

#endif // HUMAN_PROC_H

