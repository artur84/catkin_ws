#ifndef GRID_PUBLISHER_HPP_
#define GRID_PUBLISHER_HPP_


#include "ros/ros.h"
#include <string>
#include <stdint.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace ghmm_ros
{


class GridPublisher
{
public:
  GridPublisher(
    const std::string & topic, 
    ros::NodeHandle & node,
    float x, 
    float y,
    float step,
    uint8_t steps,
    float timeStep,
    uint16_t width,
    uint16_t height
  );

  void operator()( uint32_t id, float * begin, float * end );

private:
  std::string   topic_;
  float         x_; 
  float         y_;
  float         step_;
  uint8_t       steps_;
  ros::Duration timeStep_;
  uint16_t      width_;
  uint16_t      height_;
  ros::Publisher publisher_;
};


}


#endif //GRID_PUBLISHER_HPP_

