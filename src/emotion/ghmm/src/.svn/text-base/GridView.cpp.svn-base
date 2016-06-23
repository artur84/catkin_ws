#include "GridView.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <iomanip>

using namespace ghmm_ros;

GridView::GridView(
  const std::string & topic, 
  ros::NodeHandle & node,
  float x, 
  float y,
  float step,
  uint16_t width,
  uint16_t height
) : topic_( topic ),
    x_( x ),
    y_( y ),
    step_( step ),
    width_( width ),
    height_( height ),
    publisher_( node.advertise<visualization_msgs::MarkerArray>( topic, 1000 ) )
{}

void 
GridView::operator()( uint32_t id, float * grid)
{
  geometry_msgs::PoseStamped goal_pose;
  
  visualization_msgs::MarkerArray markers;
  uint32_t index = 0;
  float * p = grid;
  float y = y_ + step_ / 2.0;
  for ( uint8_t j = 0; j < height_; ++j ) {
    float x = x_ + step_ / 2.0;
    for ( uint8_t i = 0; i < width_; ++i ) {
      float height = *p * step_ * 10;
      visualization_msgs::Marker m;
      m.header.frame_id = "/map";
      m.header.stamp = ros::Time::now();
      m.ns = "ghmm";
      m.id =  index++;
      m.type = visualization_msgs::Marker::CUBE;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = height / 2.0;
      m.pose.orientation.x = 0;
      m.pose.orientation.y = 0;
      m.pose.orientation.z = 0;
      m.pose.orientation.w = 1;
      m.scale.x = step_;
      m.scale.y = step_;
      m.scale.z = height;
      m.color.r = 1;
      m.color.g = 0;
      m.color.b = 0;
      m.color.a = sqrt( *p ) * 2;
      m.lifetime = ros::Duration();
      markers.markers.push_back( m );
      p++;
      x += step_;
    }
    y += step_;
  }
  
  publisher_.publish( markers );
}

