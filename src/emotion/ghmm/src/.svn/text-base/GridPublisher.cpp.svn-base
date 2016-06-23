#include "GridPublisher.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "ghmm/ProbabilisticGrid.h"
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <iostream>
#include <iomanip>

using namespace ghmm_ros;

GridPublisher::GridPublisher(
  const std::string & topic, 
  ros::NodeHandle & node,
  float x, 
  float y,
  float step,
  uint8_t steps,
  float timeStep,
  uint16_t width,
  uint16_t height
) : topic_( topic ),
    x_( x ),
    y_( y ),
    step_( step ),
    steps_( steps ),
    timeStep_( timeStep ),
    width_( width ),
    height_( height ),
    publisher_( node.advertise<ghmm::ProbabilisticGrid>( topic, 1000 ) )
{}

void 
GridPublisher::operator()( 
  uint32_t id, 
  float * begin,
  float * end
)
{  
  ghmm::ProbabilisticGrid p_grid;
  
  p_grid.header.frame_id = "/map";
  p_grid.header.stamp = ros::Time::now();
  
  p_grid.info.width = width_;
  p_grid.info.height = height_;
  p_grid.info.resolution = step_;
  p_grid.info.origin.position.x = x_;
  p_grid.info.origin.position.y = y_;
  
  p_grid.entity = id;
  p_grid.n_steps   = steps_;
  p_grid.time_step = timeStep_;  
  
  //initialize data structure
  p_grid.data.resize( ( steps_ + 1 )* p_grid.info.width * p_grid.info.height, -1);
  copy( begin, end, p_grid.data.begin() );
  
  
  publisher_.publish( p_grid );
}

