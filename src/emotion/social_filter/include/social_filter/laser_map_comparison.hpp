#include <math.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//map
#include "sensor_msgs/LaserScan.h"
//laser
#include "nav_msgs/OccupancyGrid.h"

struct Pos{
  double x;
  double y;
};

class laserMapComparator{
public:

  sensor_msgs::LaserScan laser_scan;
  sensor_msgs::LaserScan mod_laser_scan;
  nav_msgs::OccupancyGrid occ_grid;
	ros::NodeHandle n;
  ros::Subscriber laser_sub;
  ros::Subscriber map_sub;
  ros::Publisher laser_pub;
  ros::Publisher map_pub;//test
  tf::TransformListener map_to_laser_listener;
  tf::StampedTransform map_to_laser;
  
	laserMapComparator();
	~laserMapComparator();
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void getLaserTransform();
  Pos laserToPos(int i);
  int posToCell(Pos pos);
  bool isCloseToOccupiedCell(int index);
  void copyLaser();
  void modifyLaser(int i);
	void pub();
  
};
