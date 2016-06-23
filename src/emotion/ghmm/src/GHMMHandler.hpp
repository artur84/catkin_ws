#ifndef GHMM_HANDLER_HPP_
#define GHMM_HANDLER_HPP_


#include "GHMM.hpp"
#include "GridView.hpp"
#include "GHMMView.hpp"
#include "GridPublisher.hpp"
#include "GoalPublisher.hpp"
#include "trajectory_simulator/TrajectoryObservation.h"
#include "ros/ros.h"
#include <map>
#include <string>
#include <stdint.h>


namespace ghmm_ros
{


class GHMMHandler
{
public:
  typedef ghmm::GHMM<float, 2, 4> ghmm_type;
  typedef ghmm_type::graph_type track_type;
  typedef std::map< uint32_t, track_type > tracks_type;

  GHMMHandler( 
    const std::string & gridTopic,
    const std::string & goalTopic,
    const std::string & pgridTopic,
    ros::NodeHandle & node,
    const GHMM::full_matrix_type & fullSigma,
    const GHMM::observation_matrix_type & observationSigma,
    const GHMM::goal_matrix_type & goalSigma,
    float insertionDistance, 
    float epsilon,
    float statePrior,
    float transitionPrior,
    float observationInterval,
    float messageInterval,
    int   steps,
    float x1,
    float y1, 
    float x2, 
    float y2,
    float cellSize
  );

  void operator()( 
    const trajectory_simulator::TrajectoryObservation::ConstPtr & o
  );

private:
  typedef std::vector< trajectory_simulator::TrajectoryObservation > sequence_type;
  typedef std::map< uint32_t, sequence_type > sequences_type;
  typedef std::map< uint32_t, ros::Time> times_type;

  void sendMessages(const trajectory_simulator::TrajectoryObservation::ConstPtr & o);

  ghmm_type      ghmm_;
  float          observationInterval_;
  float          messageInterval_;
  ros::Time      lastTime_;
  int            steps_;
  float          x1_;
  float          y1_;
  float          x2_;
  float          y2_;
  float          cellSize_;
  uint16_t       gridWidth_;
  uint16_t       gridHeight_;
  GridView       gridView_;
  GHMMView       ghmmView_;
  GridPublisher  gridPublisher_;
  GoalPublisher  goalPublisher_;
  sequences_type sequences_;
  tracks_type    tracks_;
  times_type     times_;
  double         max_p;
  double         x_at_max;
  double         y_at_max;
  int            state_counter;
};


}


#endif //GHMM_HANDLER_HPP_

