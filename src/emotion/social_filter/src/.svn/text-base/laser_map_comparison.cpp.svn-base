#include "ros/ros.h"
#include "social_filter/laser_map_comparison.hpp"
#define PI 3.14159

using namespace std;

laserMapComparator::laserMapComparator(){
  map_sub = n.subscribe("/map", 10, &laserMapComparator::mapCallback, this);
  //laser_sub = n.subscribe("/robot_0/base_scan", 10, &laserMapComparator::laserCallback, this);
    laser_sub = n.subscribe("/base_scan", 10, &laserMapComparator::laserCallback, this);

  laser_pub = n.advertise<sensor_msgs::LaserScan>("mod_scan", 10);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("mod_map", 10);
}

laserMapComparator::~laserMapComparator(){

}

void laserMapComparator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  occ_grid = *msg;
}

void laserMapComparator::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laser_scan = *msg;
}

void laserMapComparator::getLaserTransform(){
  if(map_to_laser_listener.canTransform("/map", "/base_link"/*"/robot_0/base_laser_link"*/, ros::Time(0))){
    map_to_laser_listener.lookupTransform("/map", "/base_link"/*"/robot_0/base_laser_link"*/, ros::Time(0), map_to_laser);
  }
}

Pos laserMapComparator::laserToPos(int i){
  Pos pos;
  pos.x = laser_scan.ranges[i] * cos(i * laser_scan.angle_increment - PI/2.0 + tf::getYaw(map_to_laser.getRotation())) + map_to_laser.getOrigin().x();
  pos.y = laser_scan.ranges[i] * sin(i * laser_scan.angle_increment - PI/2.0 + tf::getYaw(map_to_laser.getRotation())) + map_to_laser.getOrigin().y();
  return pos;
}

int laserMapComparator::posToCell(Pos pos){
  int index;
  int cell_x;
  int cell_y;
  cell_x = (int)((pos.x - occ_grid.info.origin.position.x) / occ_grid.info.resolution);
  cell_y = (int)((pos.y - occ_grid.info.origin.position.y) / occ_grid.info.resolution);
  index = cell_y * occ_grid.info.width + cell_x;
  return index;
}

bool laserMapComparator::isCloseToOccupiedCell(int index){
  bool closeEnough = false;
  int new_index;
  for(int i=0;i<11;i++){
    for(int j=0;j<11;j++){
      new_index = index + (i-5) + (j-5)*occ_grid.info.width;
      if(new_index < occ_grid.info.width * occ_grid.info.height && new_index > 0){
        if(occ_grid.data[new_index] > 40){
          closeEnough = true;
          occ_grid.data[new_index] = 50;//test
        }
      }
    }
  }
  return closeEnough;
}

void laserMapComparator::copyLaser(){
  mod_laser_scan = laser_scan;
  for (int i=0;i<mod_laser_scan.intensities.size();i++){
    mod_laser_scan.intensities[i] = 0.0;
  }
}

void laserMapComparator::modifyLaser(int i){
  mod_laser_scan.intensities[i] = 1.0;
}

void laserMapComparator::pub(){
  laser_pub.publish(mod_laser_scan);
  map_pub.publish(occ_grid);//test
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_map_comparator");

  laserMapComparator lmp;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    lmp.getLaserTransform();
    lmp.copyLaser();
    for(int i=0;i<lmp.mod_laser_scan.ranges.size();i++){
      if(lmp.isCloseToOccupiedCell(lmp.posToCell(lmp.laserToPos(i)))){
        lmp.modifyLaser(i);
      }
    }
    lmp.pub();

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
