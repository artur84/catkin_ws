#ifndef GOAL_PUBLISHER_HPP_
#define GOAL_PUBLISHER_HPP_


#include "ros/ros.h"
#include <string>
#include <stdint.h>


namespace ghmm_ros
{


class GoalPublisher
{
public:
  GoalPublisher(
    const std::string & goal_topic, 
    ros::NodeHandle & node,
    float x, 
    float y
  );

  void operator()(  uint32_t id, double max_p, double x_at_max, double y_at_max);

private:
  std::string goal_topic_;
  float       x_; 
  float       y_;
  ros::Publisher goal_publisher_;
  ros::Publisher goal_publisher0_;
  ros::Publisher goal_publisher1_;
  ros::Publisher goal_publisher2_;
};


}


#endif //GOAL_PUBLISHER_HPP_

