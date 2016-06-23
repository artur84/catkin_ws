#include "ros/ros.h"
#include "social_filter/old_human_proc.h"
#include "tf/tf.h"

#define DEG2RAD 0.01745



/*******
This node gets data from sensors(stage) and publishes structures for humanPoses
There are two classes humanProc and odom_reader. We have one odom_reader instance for each robot_i/odom
robot_0 is the wheelchair
********/

odom_reader::odom_reader(std_msgs::String odom_topic, std_msgs::String pose_topic, int id, ros::NodeHandle* n)
{
  local_ped.id = id;
  local_n = n;
  odom_name.data.assign(odom_topic.data);
  pose_name.data.assign(pose_topic.data);
  ready = false;
}

void odom_reader::init()
{
  local_ped.x=0.0;
  local_ped.y=0.0;
  local_ped.theta= 0.0; //in degrees
  local_ped.linear_velocity=0.0;
  local_ped.angular_velocity=0.0;
  odom_sub = local_n->subscribe(odom_name.data.c_str(), 15, &odom_reader::odom_callback,this);

  //ghmm wrapper part
  pose_sub = local_n->subscribe(pose_name.data.c_str(), 15, &odom_reader::pose_callback,this);
  ghmm_wrapper.type = 2; //track ended (no track)
}

void odom_reader::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  local_ped.header.frame_id = msg->header.frame_id;
  local_ped.header.stamp = msg->header.stamp;
  local_ped.x = msg->pose.pose.position.x;
  local_ped.y = msg->pose.pose.position.y;
  local_ped.theta= (local_ped.theta + atan2(msg->twist.twist.linear.y,msg->twist.twist.linear.x))/2;//tf::getYaw(msg->pose.pose.orientation); //in radians
  double velocity = sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));
  local_ped.linear_velocity = (local_ped.linear_velocity + velocity)/2;/*0.4;*/ //procopio force because of noise
  local_ped.angular_velocity=msg->twist.twist.angular.z;
  ready=true;
}

void odom_reader::pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if(/*local_ped.id <= 3*/true){ //artifficial publish only 3 agents to ghmm
    //ghmm wrapper
    ghmm_wrapper.object_id = local_ped.id;
    ghmm_wrapper.header.frame_id = msg->header.frame_id;
    ghmm_wrapper.header.stamp = msg->header.stamp;
    ghmm_wrapper.pose.x = -msg->pose.pose.position.y;
    ghmm_wrapper.pose.y = msg->pose.pose.position.x;
    ghmm_wrapper.velocity = msg->twist.twist;

    //only for GHMM
    //change type according to are
//     if (ghmm_wrapper.pose.x < -11.5 || ghmm_wrapper.pose.x > 11.5){ //out of zone
//       if (ghmm_wrapper.type == 0){
//         ghmm_wrapper.type = 2; //track ended
//       }
//     }
//     else{ //inside zone
//       if (ghmm_wrapper.type == 2){
//         ghmm_wrapper.type = 1; //track started
//       }
//       else if (ghmm_wrapper.type == 1){
//         ghmm_wrapper.type = 0; //track middle
//       }
//     }

    trajectory_pub.publish(ghmm_wrapper);
  }
}

humanProc::humanProc(int nh)
{
 //to publish human positions
  pose_pub = n.advertise<social_filter::humanPoses>("human_poses", 15);
  NoH = nh - 1; //as robot count as human too

  //advertise topic to publish human positions for GHMM
  trajectory_pub = n.advertise<human_leader::TrajectoryObservation>("dynamic_objects", 1);
}

void humanProc::init()
{
  //to subscribe human positions
  for(unsigned int i = 1; i <= NoH; i++){
    std_msgs::String sub_rbt_odom;
    std_msgs::String sub_rbt_pos;

    std::stringstream oo;
    oo<<"/robot_"<<i<<"/odom";
    sub_rbt_odom.data = oo.str();

    std::stringstream pp;
    pp<<"/robot_"<<i<<"/base_pose_ground_truth";
    sub_rbt_pos.data = pp.str();

    odom_reader* reader_ptr = new odom_reader(sub_rbt_odom, sub_rbt_pos, i, &n);

    reader_ptr->init();
    readers.push_back(reader_ptr);

  }
  ROS_INFO(" %d humans subscribed!",NoH);
}


void humanProc::pub()
{
 //take data from stage and publish as humanPose msg
  list_ped.humans.clear();

  for(unsigned int i=0; i< readers.size();i++){
    list_ped.humans.push_back(readers[i]->local_ped);
    pose_pub.publish(list_ped);
  }
}

humanProc::~humanProc()
{
  for(unsigned int i=0; i< readers.size();i++){
    delete readers[i];
  }
}

bool humanProc::ready(){
  unsigned int i;
  for( i=0; i< readers.size();i++){
    if(!readers[i]->ready) break;
  }
  //TODO:verify the case i=0
  if(i == readers.size()){
    return true;
  }
  else{
    return false;
  }
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
  ros::Rate loop_rate(10);

  while (ros::ok()){

    if(h_processor.ready()){
      h_processor.pub();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

