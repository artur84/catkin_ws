#include "ros/ros.h"
#include "GHMMHandler.hpp" 
#include "trajectory_simulator/TrajectoryObservation.h"
//#include "dynamic_reconfigure/server.h"

ghmm_ros::GHMMHandler * ghmmHandler = NULL;

void callback( const trajectory_simulator::TrajectoryObservation::ConstPtr & o )
{
  (*ghmmHandler)( o );
}

int main( int argc, char * argv[] ) 
{
  // Configure ROS
  ros::init( argc, argv, "ghmm" );
  ros::NodeHandle node;
  ros::Subscriber subscriber = 
    node.subscribe( "dynamic_objects", 1000, callback );
  
  // Configure GHMM os=0.5 gs=2
  ghmm_ros::GHMM::observation_matrix_type observationSigma;
  observationSigma << 0.5, 0.0, 
                      0.0, 0.5;

  ghmm_ros::GHMM::goal_matrix_type goalSigma;
  goalSigma << 5.0, 0.0, 
               0.0, 5.0;

  ghmm_ros::GHMM::full_matrix_type fullSigma;
  fullSigma << 0.5, 0.0, 0.0, 0.0,
               0.0, 0.5, 0.0, 0.0,
               0.0, 0.0, 5.0, 0.0,
               0.0, 0.0, 0.0, 5.0;

  ghmm_ros::GHMMHandler handler( 
    "/ghmmGrid",
    "/leader_goal_pose",
    "/probabilistic_grid",                                
    node,
    fullSigma, 
    observationSigma,
    goalSigma,
    1, /*0.01*/ //Insertion distance
    0,
    1E-10,//state prior
    1E-10,
    0.5, //observation interval
    0.5, //message interval
    0,//20,//using 0 time steps for testing
    //map size: x1,y1,x2,y2,cellsize
    -3, -3, 17, 13, 0.5
    // -3, -3, 13, 13, 0.5
  );

  ghmmHandler = & handler;

  //dynamic_reconfigure::Server<image_threshold::ImageConfig> server;
  //dynamic_reconfigure::Server<image_threshold::ImageConfig>::CallbackType f;
  //f = boost::bind(&callback, _1, _2);
  //server.setCallback(f);


  // Let it roll
  ros::spin();
  return 0;
}

