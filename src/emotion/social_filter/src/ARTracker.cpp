#include "ARTracker.hpp"

#include <iostream>

ARTracker::ARTracker()
{
  arSub = nodeHandle.subscribe<ar_pose::ARMarkers>("/ar_pose_marker", 10, boost::bind(&ARTracker::ARCallback, this, _1));

  posePub = nodeHandle.advertise<social_filter::humanPoses>("human_poses", 15);
  poseArrayPub = nodeHandle.advertise<geometry_msgs::PoseArray>("human_poses_array", 15);
  trajectoryPub = nodeHandle.advertise<trajectory_simulator::TrajectoryObservation>("dynamic_objects", 15);
  
}

void ARTracker::loop()
{
  //std::cout << "loop" << std::endl;
  ros::Time t = ros::Time::now();
  
  social_filter::humanPoses list_ped;

  geometry_msgs::PoseArray poseArray;
  poseArray.header.stamp = t;
  poseArray.header.frame_id = "/map";
  
  for(std::map<int, ARTarget>::iterator it = targetList.begin();
      it != targetList.end();
      )
  {
    double dt = ros::Duration(t - it->second.lastDetectTime).toSec();
    //std::cout<< "dt : " << dt << std::endl;
    if(dt<5.)
    {
      it->second.predict(t);
      
      social_filter::humanPose ped;
      
      ped.id = it->first;
      ped.header.frame_id = "/map";
      ped.header.stamp = t;
      ped.x = it->second.getX();
      ped.y = it->second.getY();
      ped.theta = it->second.getTheta();
      ped.linear_velocity = it->second.getLinearSpeed();
      ped.angular_velocity = it->second.getAngularSpeed();
      list_ped.humans.push_back(ped);
      
      geometry_msgs::Pose pose;
      pose.position.x = it->second.getX();
      pose.position.y = it->second.getY();
      pose.orientation = tf::createQuaternionMsgFromYaw(it->second.getTheta());
      poseArray.poses.push_back(pose);
      it++;
    }else{
      // remove target
      targetList.erase(it++);
    }
  }
  
  posePub.publish(list_ped);
  poseArrayPub.publish(poseArray);
}

void ARTracker::ARCallback(const ar_pose::ARMarkers::ConstPtr& markers)
{
  //listener.waitForTransform("/prosilica_optical_frame", "/map", markers->header.stamp, ros::Duration(10.0));

  //std::cout << "markers : " << markers->markers.size() << std::endl;
  
  for(unsigned int i=0; i< markers->markers.size(); i++)
  {
    int id = markers->markers[i].id;
    geometry_msgs::PoseStamped pose_cam;
    pose_cam.header = markers->markers[i].header;
    pose_cam.pose = markers->markers[i].pose.pose;
    
    geometry_msgs::PoseStamped pose_map;
    
    listener.transformPose("/map", pose_cam, pose_map);
//     pose_map = pose_cam;
    
    std::map<int, ARTarget>::iterator it = targetList.find(id);
    if(it == targetList.end())
    {
      targetList[id].init(markers->markers[i].header.stamp,
                          pose_map.pose.position.x,
                          pose_map.pose.position.y,
                          tf::getYaw(pose_map.pose.orientation));
    }else{
      it->second.update(markers->markers[i].header.stamp,
                        pose_map.pose.position.x,
                        pose_map.pose.position.y,
                        tf::getYaw(pose_map.pose.orientation));      
    }   
  }
}
