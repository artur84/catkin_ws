#include "RobotRos.hpp"
#include <cmath>
#include <iostream>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
RobotRos::RobotRos()
{
	ready_pose=false;
	ready_odom=false;
}

void RobotRos::init(Robot*)
{
	markerPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>( "marker_robot", 1 );
}

State RobotRos::get_riskrrt_robot_state(const tf::Stamped<tf::Pose>& global_pose,
		const nav_msgs::Odometry& base_odom )
{
	State s;
	geometry_msgs::PoseStamped pose_msg;
	tf::poseStampedTFToMsg(global_pose, pose_msg);
	s.x=pose_msg.pose.position.x;
	s.y=pose_msg.pose.position.y;
	s.theta=tf::getYaw(pose_msg.pose.orientation);
	s.v = base_odom.twist.twist.linear.x;
	s.omega = base_odom.twist.twist.angular.z;
	return s;
}
void RobotRos::setControl(const Control& c)
{
	twist_msg.linear.x = c.u1;
	twist_msg.linear.y = 0.;
	twist_msg.linear.z = 0.;
	twist_msg.angular.x = 0.;
	twist_msg.angular.y = 0.;
	twist_msg.angular.z = c.u2;
}

Control RobotRos::readControl()
{
	Control c(twist_msg.linear.x, twist_msg.angular.z);
	return c;
}

void RobotRos::print_marker(const nav_msgs::Odometry& odom)
{
	visualization_msgs::Marker marker;
	visualization_msgs::MarkerArray ma;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.1f;
	marker.color.g = 0.1f;
	marker.color.b = 0.1f;
	marker.color.a = 0.4;
	marker.lifetime = ros::Duration(1.0);
	marker.id = 1;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose=odom.pose.pose;

	// Set the frame ID and timestamp.
	marker.header.frame_id =odom.header.frame_id;
	marker.header.stamp = odom.header.stamp;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "robot_shape";
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.mesh_resource = "package://social_filter/meshes/wheelchair/Wheelchair1.dae";
	marker.mesh_use_embedded_materials = true;
	marker.scale.x = 1.1;
	marker.scale.y = 1.1;
	marker.scale.z = 1.1;
	ma.markers.push_back(marker);
	//this is for the person sitted down in the wheelchair
	marker.id = 2;
	marker.mesh_resource = "package://social_filter/meshes/human/3D_Man_Seated.dae";
	marker.mesh_use_embedded_materials = true;
	marker.scale.x = .02;
	marker.scale.y = .02;
	marker.scale.z = .02;
	marker.pose.position.x+=0.0;
	marker.pose.position.z+=.25;

	// marker.pose.orientation = tf::createQuaternionFromYaw(1.57 + tf::getYaw( marker.pose.orientation));
	ma.markers.push_back(marker);
	//*********************8
	markerPublisher.publish(ma);
}


void RobotRos::initLoc()
{
}

State RobotRos::readState()
{
	State s;
	s.x = pose_msg.pose.pose.position.x;
	s.y = pose_msg.pose.pose.position.y;
	s.theta = tf::getYaw(pose_msg.pose.pose.orientation);
	s.v = odom_msg.twist.twist.linear.x;
	s.omega = odom_msg.twist.twist.angular.z;
	return s;
}

timeval RobotRos::getTimeLoc()
{
	timeval t;
	t.tv_usec = pose_msg.header.stamp.nsec/1000;
	t.tv_sec = pose_msg.header.stamp.sec;
	return t;
}

timeval RobotRos::getTimeOdo()
{
	timeval t;
	t.tv_usec = odom_msg.header.stamp.nsec/1000;
	t.tv_sec = odom_msg.header.stamp.sec;
	return t;
}
