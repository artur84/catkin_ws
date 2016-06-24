#include "ros/ros.h"
#include "human_proc.h"
#include "tf/tf.h"

#define DEG2RAD 0.01745



/*******
This node gets data from sensors(stage) and publishes structures for humanPoses
There are two classes humanProc and odom_reader. We have one odom_reader instance for each robot_i/odom
robot_0 is the wheelchair
********/

odom_reader::odom_reader(std_msgs::String topic_name,int id, ros::NodeHandle* n)
{
  local_ped.id=id;
  local_n=n;
  name.data.assign(topic_name.data);
  ready=false;
  
}

void odom_reader::init()
{
  local_ped.x=0.0;
  local_ped.y=0.0;
  local_ped.theta= 0.0; //in degrees
  local_ped.linear_velocity=0.0;
  local_ped.angular_velocity=0.0;
   
 odom_sub=local_n->subscribe(name.data.c_str(), 100, &odom_reader::odom_callback,this);
}

void odom_reader::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
 local_ped.header.frame_id=msg->header.frame_id;
 local_ped.header.stamp= msg->header.stamp;
 local_ped.x=msg->pose.pose.position.x;
 local_ped.y=msg->pose.pose.position.y;
 local_ped.theta= tf::getYaw(msg->pose.pose.orientation); //in radians
 local_ped.linear_velocity=msg->twist.twist.linear.x;
 local_ped.angular_velocity=msg->twist.twist.angular.z;
 ready=true; 
}


humanProc::humanProc(int nh)
{
 //to publish human positions 
  pose_pub = n.advertise<social_filter::humanPoses>("human_poses", 100);
  NoH=nh;
}

void humanProc::init()
{
 
  //to subscribe human positions
  for(unsigned int i=1; i<= NoH ; i++)
    {
      std_msgs::String msg;
      std::stringstream ss;
      ss<<"/robot_"<<i<<"/odom";
      msg.data = ss.str();
      odom_reader* reader_ptr = new odom_reader(msg,i,&n);
      reader_ptr->init();
      readers.push_back(reader_ptr);     
    }
 ROS_INFO(" %d humans subscribed",NoH);
}


void humanProc::pub()
{
 //take data from stage and publish as humanPose msg
 list_ped.humans.clear();

  for(unsigned int i=0; i< readers.size();i++)
     list_ped.humans.push_back(readers[i]->local_ped);

    pose_pub.publish(list_ped);
 
}

humanProc::~humanProc()
{

 for(unsigned int i=0; i< readers.size();i++)
   delete readers[i];

}

bool humanProc::ready(){
  unsigned int i;
  for( i=0; i< readers.size();i++)
    if(!readers[i]->ready) break;

  //TODO:verify the case i=0
  if(i == readers.size())
    return true;
  else return false;

}
int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "human_proc");

  if (argc != 2){
    ROS_INFO("USAGE: human_proc [number_of_humans]");
    return 0;
  }

 int nh= atoi(argv[1]);
 humanProc h_processor(nh);
 
 h_processor.init();
 ros::Rate loop_rate(3);

 while (ros::ok())
   {  
     
     if(h_processor.ready()){   
       //   ROS_INFO("publishing human poses");
       h_processor.pub();
     }

     ros::spinOnce();
     loop_rate.sleep();
   }

  return 0;
}
 
