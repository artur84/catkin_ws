#ifndef COSTMAP_OBSERVATION_H_
#define COSTMAP_OBSERVATION_H_


#include <geometry_msgs/Point.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace social_costmap {

  /**
   * @brief Stores an observation in terms of a point cloud and the origin of the source
   * @note Tried to make members and constructor arguments const but the compiler would not accept the default
   * assignment operator for vector insertion!
   */
  class Observation {
  public:
    /**
     * @brief  Creates an empty observation
     */
    Observation() : cloud_(), obstacle_range_(0.0), raytrace_range_(0.0){}
    /**
     * @brief  Creates an observation from an origin point and a point cloud
     * @param origin The origin point of the observation
     * @param cloud The point cloud of the observation
     * @param obstacle_range The range out to which an observation should be able to insert obstacles
     * @param raytrace_range The range out to which an observation should be able to clear via raytracing
     */
    Observation(geometry_msgs::Point& origin, pcl::PointCloud<pcl::PointXYZ> cloud, double obstacle_range, double raytrace_range): origin_(origin), 
    cloud_(cloud), obstacle_range_(obstacle_range), raytrace_range_(raytrace_range) {}

    /**
     * @brief  Copy constructor
     * @param obs The observation to copy
     */
    Observation(const Observation& obs): origin_(obs.origin_), cloud_(obs.cloud_), obstacle_range_(obs.obstacle_range_), raytrace_range_(obs.raytrace_range_){}

    /**
     * @brief  Creates an observation from a point cloud
     * @param cloud The point cloud of the observation
     * @param obstacle_range The range out to which an observation should be able to insert obstacles
     */
    Observation(pcl::PointCloud<pcl::PointXYZ> cloud, double obstacle_range): cloud_(cloud), obstacle_range_(obstacle_range), raytrace_range_(0.0){}

    geometry_msgs::Point origin_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    double obstacle_range_, raytrace_range_;
  };

}
#endif
