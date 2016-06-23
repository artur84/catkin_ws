#ifndef COSTMAP_SOCIAL_COSTMAP_PUBLISHER_H_
#define COSTMAP_SOCIAL_COSTMAP_PUBLISHER_H_
#include <ros/ros.h>
#include <social_costmap/social_costmap.h>
#include <nav_msgs/GridCells.h>

#include <tf/transform_datatypes.h>

#include <boost/thread.hpp>

namespace social_costmap {
  /**
   * @class SocialCostmapPublisher
   * @brief A tool to periodically publish visualization data from a SocialCostmap
   */
  class SocialCostmapPublisher {
    public:
      /**
       * @brief  Constructor for the SocialCostmapPublisher
       * @param  ros_node The node under which to publish the visualization output
       * @param  global_frame The frame in which to publish the visualization output
       */
      SocialCostmapPublisher(ros::NodeHandle ros_node, double publish_frequency, std::string global_frame);

      /**
       * @brief  Destructor
       */
      ~SocialCostmapPublisher();

      /**
       * @brief  Publishes footprint visualization data over ROS
       */
      void publishFootprint();

      /**
       * @brief  Publishes the visualization data over ROS
       */
      void publishCostmap();

      /**
       * @brief  Update the visualization data from a SocialCostmap
       * @param costmap The SocialCostmap object to create visualization messages from 
       * @param footprint The footprint of the robot associated with the costmap
       */
      void updateCostmapData(const SocialCostmap& costmap, 
          const std::vector<geometry_msgs::Point>& footprint = std::vector<geometry_msgs::Point>(),
          const tf::Stamped<tf::Pose>& global_pose = tf::Stamped<tf::Pose>());

      /**
       * @brief Check if the publisher is active
       * @return True if the frequency for the publisher is non-zero, false otherwise
       */
      bool active() {return active_;}

    private:
      void mapPublishLoop(double frequency);

      std::string global_frame_;
      boost::thread* visualizer_thread_; ///< @brief A thread for publising to the visualizer
      std::vector< std::pair<double, double> > raw_obstacles_, inflated_obstacles_, unknown_space_;
      boost::recursive_mutex lock_; ///< @brief A lock
      bool active_, new_data_;
      ros::Publisher obs_pub_, inf_obs_pub_, unknown_space_pub_, footprint_pub_;
      double resolution_, inscribed_radius_;
      std::vector<geometry_msgs::Point> footprint_;
      tf::Stamped<tf::Pose> global_pose_;
      bool visualizer_thread_shutdown_;
  };
};
#endif
