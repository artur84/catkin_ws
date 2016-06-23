#ifndef GRID_VIEW_HPP_
#define GRID_VIEW_HPP_


#include "ros/ros.h"
#include <string>
#include <stdint.h>


namespace ghmm_ros
{


class GridView
{
public:
  GridView(
    const std::string & topic, 
    ros::NodeHandle & node,
    float x, 
    float y,
    float step,
    uint16_t width,
    uint16_t height
  );

  void operator()( uint32_t id, float * grid );

private:
  std::string topic_;
  float       x_; 
  float       y_;
  float       step_;
  uint16_t    width_;
  uint16_t    height_;
  ros::Publisher publisher_;
};


}


#endif //GRID_VIEW_HPP_

