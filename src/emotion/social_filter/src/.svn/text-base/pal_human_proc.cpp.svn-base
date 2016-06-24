#include "ros/ros.h"
#include "social_filter/pal_human_proc.h"

#define DEG2RAD 0.01745
#define pi 3.1416

using namespace std;

/*******
 This node gets data from tf and publishes structures for humanPoses.
 ********/

//here nh is the maximum number of id we can process
PALHumanProc::PALHumanProc(int nh)
{
  //to publish human positions
  pose_pub = n.advertise<social_filter::humanPoses>("human_poses", 15);
  NoH = nh;
}

void PALHumanProc::init()
{
//	ready[0]=false;
  ROS_INFO(" %d humans subscribed", NoH);
  previous_x = 0;
  previous_y = 0;
  previous_stamp = ros::Time::now();
}

void PALHumanProc::pub()
{

  int i;
  //we suppose that if torso_i exists then human i exists

  list_ped.humans.clear();
  for (i = 0; i < NoH; i++)
  {

    stringstream ss;
    ss << "/torso_" << i + 1;
    //cout<<ss.str()<<endl;
    if (tf_listener.canTransform("/map", ss.str(), ros::Time(0)))
    {
      tf_listener.lookupTransform("/map", ss.str(), ros::Time(0),
          transform_user);
      //The TF will be considered just if it is not older than a given time ---ros::Duration()---
      if (transform_user.stamp_ >= ros::Time::now() - ros::Duration(5.0))
      {
        local_ped.id = i + 1;
        local_ped.header.frame_id = transform_user.frame_id_;
        local_ped.header.stamp = transform_user.stamp_;
        local_ped.x = transform_user.getOrigin().x();
        local_ped.y = transform_user.getOrigin().y();
        local_ped.theta = tf::getYaw(transform_user.getRotation()); //in radians
        d = transform_user.stamp_ - previous_stamp;
        local_ped.linear_velocity = sqrt(
            pow(velocity.linear.x, 2) + pow(velocity.linear.y, 2));
        local_ped.angular_velocity = velocity.angular.z;
        list_ped.humans.push_back(local_ped);

        p[0] = 0; //transform_user.getOrigin().x();
        p[1] = 0; //transform_user.getOrigin().y();
        p[2] = 0; //transform_user.getOrigin().z();

        previous_x = transform_user.getOrigin().x();
        previous_y = transform_user.getOrigin().y();
        previous_stamp = transform_user.stamp_;
      }
    }
  }
  pose_pub.publish(list_ped);

}

PALHumanProc::~PALHumanProc()
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pal_human_proc");

  if (argc != 2)
  {
    ROS_INFO("USAGE: human_proc [max_number_of_humans]");
    return 0;
  }

  int nh = atoi(argv[1]);
  PALHumanProc h_processor(nh);
  h_processor.init();
  ros::Rate loop_rate(30);

  while (ros::ok())
  {

    h_processor.pub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

