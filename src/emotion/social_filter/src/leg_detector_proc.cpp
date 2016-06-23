#include "ros/ros.h"
#include "social_filter/leg_detector_proc.hpp"
#define PI 3.14159

using namespace std;

legProc::legProc(){
  pos_sub = n.subscribe("/people_tracker_measurements", 10, &legProc::posCallback, this);
  pos_pub = n.advertise<social_filter::humanPoses>("human_poses", 10);
  //prev_speed.push_back(0.0);
}

legProc::~legProc(){

}

void legProc::posCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
{
  pos_array.humans.clear();
  social_filter::humanPose pose;
  for(int i=0;i<msg->people.size();i++){
    pose.header = msg->people[i].header;
    //pose.header.stamp = ros::Time::now();
    //pose.header.frame_id = "base_laser";
    pose.id = i+1;
    pose.x = msg->people[i].pose.position.x;
    pose.y = msg->people[i].pose.position.y;

    double sum_cos;
    double sum_sin;
    double angle = tf::getYaw(msg->people[i].pose.orientation);
    prev_theta.push_back(angle);
    if(prev_theta.size() > 50) prev_theta.erase(prev_theta.begin());
    for(int j=0;j<prev_theta.size();j++){
      sum_cos += cos(prev_theta[j]);
      sum_sin += sin(prev_theta[j]);
    }
    pose.theta = atan2(sum_sin, sum_cos);

    double sum = 0;
    double speed = sqrt(pow(msg->people[i].velocity.linear.x,2)+pow(msg->people[i].velocity.linear.y,2));
    prev_speed.push_back(speed);
    if(prev_speed.size() > 50) prev_speed.erase(prev_speed.begin());
    for(int i=0;i<prev_speed.size();i++){
      sum += prev_speed[i];
    }
    pose.linear_velocity = sum / prev_speed.size();

    if(pose.linear_velocity<0.01) pose.linear_velocity=0;
    pose.angular_velocity = 0;// msg->people[i].velocity.angular.z;
    pos_array.humans.push_back(pose);
  }
  pos_pub.publish(pos_array);
}

void legProc::pub(){
  pos_pub.publish(pos_array);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leg_detector_proc");

  legProc legproc;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    //legproc.pub();

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
