#include "ros/ros.h"
#include "kinect_human_proc.h"

#define DEG2RAD 0.01745
#define pi 3.1416

/*******
 This node gets data from sensors(kinect) openni_tracker and publishes structures for humanPoses.
 human_proc is prepared to read messages from openni_kinect and publish humanPoses
 ********/

//here nh is the maximum number of id we can process
KhumanProc::KhumanProc(int nh) {
	//to publish human positions
	pose_pub = n.advertise<social_filter::humanPoses>("human_poses", 15);
	NoH = nh;
}

void KhumanProc::init() {
//	ready[0]=false;
	ROS_INFO(" %d humans subscribed", NoH);
}

void KhumanProc::pub() {

  int i;
  //we suppose that if torso_i exists then human i exists

  list_ped.humans.clear();
  for(i=0;i<NoH;i++){

    std::stringstream ss;
    ss<<"/torso_"<<i+1;
    //std::cout<<ss.str()<<std::endl;
    if(kinect_listener.canTransform("/map", ss.str(), ros::Time(0) ) ) {            
      kinect_listener.lookupTransform("/map", ss.str(), ros::Time(0),
				      transform_user);
      local_ped.id=i+1;
      local_ped.header.frame_id = transform_user.frame_id_;
      local_ped.header.stamp = transform_user.stamp_;
      local_ped.x = transform_user.getOrigin().x();
      local_ped.y = transform_user.getOrigin().y();
      local_ped.theta = tf::getYaw(transform_user.getRotation()) + pi / 2; //in radians
      local_ped.linear_velocity = 0.0; //TODO: check how to get velocity from odometry and use kalman filter from Amaury
      local_ped.angular_velocity = 0.0;		
      list_ped.humans.push_back(local_ped);

    }
    //else
      //std::cout<<"no se pudo transformar"<<std::endl;
  }
  pose_pub.publish(list_ped);

}

KhumanProc::~KhumanProc() {

}


int main(int argc, char **argv) {
	ros::init(argc, argv, "kinect_human_proc");

	if (argc != 2) {
		ROS_INFO("USAGE: human_proc [max_number_of_humans]");
		return 0;
	}

	int nh = atoi(argv[1]);
	KhumanProc h_processor(nh);

	h_processor.init();
	ros::Rate loop_rate(3);

	while (ros::ok()) {

		h_processor.pub();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

