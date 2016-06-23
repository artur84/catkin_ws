#include "GHMMHandler.hpp"

using namespace ghmm_ros;
int counter = 0;
double last_x, last_y;

GHMMHandler::GHMMHandler( 
  const std::string & topic,
  const std::string & goal_topic,
  const std::string & grid_topic,
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
) : ghmm_( fullSigma, 
           observationSigma,
           goalSigma,
           insertionDistance,
           epsilon,
           statePrior,
           transitionPrior
    ),
    observationInterval_( observationInterval ),
    messageInterval_( messageInterval ),
    lastTime_( ros::Time::now() ),
    steps_( steps ),
    x1_( x1 ),
    y1_( y1 ),
    x2_( x2 ),
    y2_( y2 ),
    cellSize_( cellSize ),
    gridWidth_( floor( ( x2_ - x1_ ) / cellSize_ ) ),
    gridHeight_( floor( ( y2_ - y1_ ) / cellSize_ ) ),
    gridView_( topic, node, x1, y1, cellSize, gridWidth_, gridHeight_ ),
    ghmmView_( "ghmm" + topic, node ),
    gridPublisher_( grid_topic, node, x1, y1, cellSize, steps, observationInterval_, gridWidth_, gridHeight_ ),
	  goalPublisher_( goal_topic, node, x1, y1)
{
  counter = 0;
  last_x = 0;
  last_y = 0;
}

void 
GHMMHandler::operator()( 
  const trajectory_simulator::TrajectoryObservation::ConstPtr & o
) 
{
  ros::Time time = ros::Time::now();
  bool first = o->type == trajectory_simulator::TrajectoryObservation::FIRST;
  if ( first ) {
    ghmm_.initTrack( tracks_[o->object_id] );
  }
  
  if ( tracks_.find( o->object_id ) != tracks_.end() ) {
    sequence_type & observations = sequences_[o->object_id];
    track_type & track = tracks_[o->object_id];

    if (    times_.find( o->object_id ) == times_.end()
         || time.toNSec() - times_[o->object_id].toNSec() >= observationInterval_ * 1E9 
         || o->type == trajectory_simulator::TrajectoryObservation::LAST
    ) {
      times_[o->object_id] = time;
      observations.push_back( *o );

      if ( o->type == trajectory_simulator::TrajectoryObservation::LAST ) {
        std::vector< ghmm_type::full_observation_type > sequence;
        float x1 = observations[observations.size() - 1].pose.x;
        float y1 = observations[observations.size() - 1].pose.y;
        for ( uint32_t i = 0; i < observations.size(); ++i ) {
          ghmm_type::full_observation_type fo;
          fo << observations[i].pose.x, observations[i].pose.y, x1, y1;
          std::cout << counter << " " << fo << std::endl;
          sequence.push_back( fo );
          last_x = observations[i].pose.x;
          last_y = observations[i].pose.y;
        }
        
        counter++;
        goalPublisher_( o->object_id/*iTrack->first*/, counter-1, last_x, last_y);
        
        //counting states, and limiting ghmm growth
        std::cerr << "states:" << state_counter << std::endl;
        if(state_counter < 200){
          state_counter = 0;
          GHMM::itm_type::node_iterator n;
          GHMM::itm_type::node_iterator nodeEnd;
          for ( boost::tie( n, nodeEnd ) = boost::vertices( track );
                n != nodeEnd; ++n
          ) {
              state_counter++;
          }
          //ROS_INFO("states:%d",state_counter);
        
          // Learning
          ghmm_.learn( sequence.begin(), sequence.end() );
        }
        // erasing time, sequence and track
        times_.erase( times_.find( o->object_id ) );
        if ( sequences_.find( o->object_id ) != sequences_.end() ) {
          sequences_.erase( sequences_.find( o->object_id ) );
        }
        tracks_.erase( tracks_.find( o->object_id ) );
      } else {
        ghmm_type::observation_type po;
        po << o->pose.x, o->pose.y;
        ghmm_.update( track, po );
        ghmm_.predict( track, steps_ );
        if ( time - lastTime_ > ros::Duration( messageInterval_ ) ) {
          sendMessages(o); //only send prediction associated with rx msg
        }
      }
    }
  }
}

void
GHMMHandler::sendMessages(const trajectory_simulator::TrajectoryObservation::ConstPtr & o)
{
  
  track_type & track = tracks_[o->object_id];
//   counter ++;
//   ros::Time ttime = ros::Time::now();
//   ROS_INFO("time begin:%f",ros::Time::now().toSec()-ttime.toSec());
//   ttime = ros::Time::now();
  tracks_type::iterator iTrack;
  tracks_type::iterator eTracks = tracks_.end();
   
// procopio commented the iteration over tracks, as a track that was
// interrupted but not finished, was being drawn. the best imo would 
// be to only process the message associated with the callback.
//   for ( iTrack = tracks_.begin(); iTrack != eTracks; ++iTrack ) {
    float stateGrid[gridWidth_ * gridHeight_ * ( steps_ + 1 )];
    float goalGrid[gridWidth_ * gridHeight_];
    for ( uint8_t t = 0; t <= steps_; ++t ) {
      float * sp = &( stateGrid[t * gridWidth_ * gridHeight_] );
      float * gp = &( goalGrid[0] );
      max_p = 0;
      float sSum = 0;
      float gSum = 0;
      float y = y1_ + cellSize_ / 2.0;
      for ( uint8_t j = 0; j < gridHeight_; ++j ) {
        float x = x1_ + cellSize_ / 2.0;
        for ( uint8_t i = 0; i < gridWidth_; ++i ) {
          ghmm_type::observation_type center;
          center << x, y;
          *sp = ghmm_.observationPdf(track /*iTrack->second*/, t, center );

	  //goal prediction
          if ( t == 0 ) {
            *gp = ghmm_.goalPdf(track /*iTrack->second*/, center );
            if(*gp > max_p){
              max_p = *gp;
              x_at_max = x;
              y_at_max = y;
            }
            gSum += *gp++;
          }

          sSum += *sp++;
          x += cellSize_; 
        }
        y += cellSize_;
      }

      sp = &( stateGrid[t * gridWidth_ * gridHeight_] );
      gp = &( goalGrid[0] );

      for ( uint8_t j = 0; j < gridHeight_; ++j ) {
        for ( uint8_t i = 0; i < gridWidth_; ++i ) {
          *sp++ /= sSum;
	  *gp++ /= gSum; 
        }
      }
    }
//     ROS_INFO("time aft for:%f",ros::Time::now().toSec()-ttime.toSec());
//     std::cout << counter << std::endl;
//     std::cerr << ros::Time::now().toSec()-ttime.toSec() << std::endl;
//     ttime = ros::Time::now();

//     gridView_( iTrack->first, stateGrid ); //publish visual state or goal
    gridView_( o->object_id /*iTrack->first*/, goalGrid ); //publish visual state or goal
    ghmmView_( 0, track ); //publish GHMM structure
    
    gridPublisher_( 
      o->object_id/*iTrack->first*/,  
      &( stateGrid[0] ),
      &( stateGrid[gridWidth_ * gridHeight_ * ( steps_ + 1 )] )
    ); //publish grid message
    goalPublisher_( o->object_id/*iTrack->first*/, counter, x_at_max, y_at_max); //publish goal message
//   }
//   debugging time
//   ttime = ros::Time::now();
}
