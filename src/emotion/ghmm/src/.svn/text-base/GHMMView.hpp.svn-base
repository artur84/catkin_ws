#ifndef GHMM_VIEW_HPP_
#define GHMM_VIEW_HPP_


#include "GHMM.hpp"
#include "ros/ros.h"
#include <string>
#include <stdint.h>


namespace ghmm_ros
{

class GHMMView
{
public:
  GHMMView(
    const std::string & topic,
    ros::NodeHandle & node
  );

  void operator()( uint32_t id, GHMM::graph_type & graph ); 
private:
  std::string topic_;
  ros::Publisher publisher_;
};


}


#endif //GHMM_VIEW_HPP_

