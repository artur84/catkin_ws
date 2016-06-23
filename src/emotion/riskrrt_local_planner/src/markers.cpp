#include <riskrrt_local_planner/markers.h>
#include <ros/console.h>
#include <sys/time.h>
#include <math.h>
#include <cstdio>

using namespace std;


void printPoint(pcl::PointXYZ pt){
  printf("(%.2f, %.2f, %.2f)", pt.x, pt.y, pt.z);
}

void printPSHeader(){
  printf("%%!PS\n");
  printf("%%%%Creator: Eitan Marder-Eppstein (Willow Garage)\n");
  printf("%%%%EndComments\n");
}

void printPSFooter(){
  printf("showpage\n%%%%EOF\n");
}

void printPolygonPS(const vector<geometry_msgs::Point>& poly, double line_width){
  if(poly.size() < 2)
    return;

  printf("%.2f setlinewidth\n", line_width);
  printf("newpath\n");
  printf("%.4f\t%.4f\tmoveto\n", poly[0].x * 10, poly[0].y * 10);
  for(unsigned int i = 1; i < poly.size(); ++i)
    printf("%.4f\t%.4f\tlineto\n", poly[i].x * 10, poly[i].y * 10);
  printf("%.4f\t%.4f\tlineto\n", poly[0].x * 10, poly[0].y * 10);
  printf("closepath stroke\n");

}

namespace riskrrt_local_planner {

PointGrid::PointGrid(double size_x, double size_y, double resolution, geometry_msgs::Point origin, double max_z, double obstacle_range, double min_seperation) :
  resolution_(resolution), origin_(origin), max_z_(max_z), sq_obstacle_range_(obstacle_range * obstacle_range), sq_min_separation_(min_seperation * min_seperation)
  {
    width_ = (int) (size_x / resolution_);
    height_ = (int) (size_y / resolution_);
    cells_.resize(width_ * height_);
  }

  double PointGrid::footprintCost(const geometry_msgs::Point& position, const vector<geometry_msgs::Point>& footprint, 
      double inscribed_radius, double circumscribed_radius){
    //the half-width of the circumscribed sqaure of the robot is equal to the circumscribed radius
    double outer_square_radius = circumscribed_radius;

    //get all the points inside the circumscribed square of the robot footprint
    geometry_msgs::Point c_lower_left, c_upper_right;
    c_lower_left.x = position.x - outer_square_radius;
    c_lower_left.y = position.y - outer_square_radius;

    c_upper_right.x = position.x + outer_square_radius;
    c_upper_right.y = position.y + outer_square_radius;

    //This may return points that are still outside of the cirumscribed square because it returns the cells
    //contained by the range
    getPointsInRange(c_lower_left, c_upper_right, points_);

    //if there are no points in the circumscribed square... we don't have to check against the footprint
    if(points_.empty())
      return 1.0;

    //compute the half-width of the inner square from the inscribed radius of the robot
    double inner_square_radius = sqrt((inscribed_radius * inscribed_radius) / 2.0);

    //we'll also check against the inscribed square
    geometry_msgs::Point i_lower_left, i_upper_right;
    i_lower_left.x = position.x - inner_square_radius;
    i_lower_left.y = position.y - inner_square_radius;

    i_upper_right.x = position.x + inner_square_radius;
    i_upper_right.y = position.y + inner_square_radius;

    //if there are points, we have to do a more expensive check
    for(unsigned int i = 0; i < points_.size(); ++i){
      list<pcl::PointXYZ>* cell_points = points_[i];
      if(cell_points != NULL){
        for(list<pcl::PointXYZ>::iterator it = cell_points->begin(); it != cell_points->end(); ++it){
          const pcl::PointXYZ& pt = *it;
          //first, we'll check to make sure we're in the outer square
          //printf("(%.2f, %.2f) ... l(%.2f, %.2f) ... u(%.2f, %.2f)\n", pt.x, pt.y, c_lower_left.x, c_lower_left.y, c_upper_right.x, c_upper_right.y);
          if(pt.x > c_lower_left.x && pt.x < c_upper_right.x && pt.y > c_lower_left.y && pt.y < c_upper_right.y){
            //do a quick check to see if the point lies in the inner square of the robot
            if(pt.x > i_lower_left.x && pt.x < i_upper_right.x && pt.y > i_lower_left.y && pt.y < i_upper_right.y)
              return -1.0;

            //now we really have to do a full footprint check on the point
            if(ptInPolygon(pt, footprint))
              return -1.0;
          }
        }
      }
    }

    //if we get through all the points and none of them are in the footprint it's legal
    return 1.0;
  }

  bool PointGrid::ptInPolygon(const pcl::PointXYZ& pt, const vector<geometry_msgs::Point>& poly){
    if(poly.size() < 3)
      return false;

    //a point is in a polygon iff the orientation of the point
    //with respect to sides of the polygon is the same for every
    //side of the polygon
    bool all_left = false;
    bool all_right = false;
    for(unsigned int i = 0; i < poly.size() - 1; ++i){
      //if pt left of a->b
      if(orient(poly[i], poly[i + 1], pt) > 0){
        if(all_right)
          return false;
        all_left = true;
      }
      //if pt right of a->b
      else{
        if(all_left)
          return false;
        all_right = true;
      }
    }
    //also need to check the last point with the first point
    if(orient(poly[poly.size() - 1], poly[0], pt) > 0){
      if(all_right)
        return false;
    }
    else{
      if(all_left)
        return false;
    }

    return true;
  }

  void PointGrid::getPointsInRange(const geometry_msgs::Point& lower_left, const geometry_msgs::Point& upper_right, vector< list<pcl::PointXYZ>* >& points){
    points.clear();

    //compute the other corners of the box so we can get cells indicies for them
    geometry_msgs::Point upper_left, lower_right;
    upper_left.x = lower_left.x;
    upper_left.y = upper_right.y;
    lower_right.x = upper_right.x;
    lower_right.y = lower_left.y;

    //get the grid coordinates of the cells matching the corners of the range
    unsigned int gx, gy;

    //if the grid coordinates are outside the bounds of the grid... return
    if(!gridCoords(lower_left, gx, gy))
      return;
    //get the associated index
    unsigned int lower_left_index = gridIndex(gx, gy);

    if(!gridCoords(lower_right, gx, gy))
      return;
    unsigned int lower_right_index = gridIndex(gx, gy);

    if(!gridCoords(upper_left, gx, gy))
      return;
    unsigned int upper_left_index = gridIndex(gx, gy);

    //compute x_steps and y_steps
    unsigned int x_steps = lower_right_index - lower_left_index + 1;
    unsigned int y_steps = (upper_left_index - lower_left_index) / width_ + 1;
    /*
     * (0, 0) ---------------------- (width, 0)
     *  |                               |
     *  |                               |
     *  |                               |
     *  |                               |
     *  |                               |
     * (0, height) ----------------- (width, height)
     */
    //get an iterator
    vector< list<pcl::PointXYZ> >::iterator cell_iterator = cells_.begin() + lower_left_index;
    //printf("Index: %d, Width: %d, x_steps: %d, y_steps: %d\n", lower_left_index, width_, x_steps, y_steps);
    for(unsigned int i = 0; i < y_steps; ++i){
      for(unsigned int j = 0; j < x_steps; ++j){
        list<pcl::PointXYZ>& cell = *cell_iterator;
        //if the cell contains any points... we need to push them back to our list
        if(!cell.empty()){
          points.push_back(&cell);
        }
        if(j < x_steps - 1)
          cell_iterator++; //move a cell in the x direction
      }
      cell_iterator += width_ - (x_steps - 1); //move down a row
    }
  }

  void PointGrid::insert(pcl::PointXYZ pt){
    //get the grid coordinates of the point
    unsigned int gx, gy;

    //if the grid coordinates are outside the bounds of the grid... return
    if(!gridCoords(pt, gx, gy))
      return;

    //if the point is too close to its nearest neighbor... return
    if(nearestNeighborDistance(pt) < sq_min_separation_)
      return;

    //get the associated index
    unsigned int pt_index = gridIndex(gx, gy);

    //insert the point into the grid at the correct location
    cells_[pt_index].push_back(pt);
    //printf("Index: %d, size: %d\n", pt_index, cells_[pt_index].size());
  }

  double PointGrid::getNearestInCell(pcl::PointXYZ& pt, unsigned int gx, unsigned int gy){
    unsigned int index = gridIndex(gx, gy);
    double min_sq_dist = DBL_MAX;
    //loop through the points in the cell and find the minimum distance to the passed point
    for(list<pcl::PointXYZ>::iterator it = cells_[index].begin(); it != cells_[index].end(); ++it){
      min_sq_dist = min(min_sq_dist, sq_distance(pt, *it));
    }
    return min_sq_dist;
  }


  double PointGrid::nearestNeighborDistance(pcl::PointXYZ& pt){
    //get the grid coordinates of the point
    unsigned int gx, gy;

    gridCoords(pt, gx, gy);

    //get the bounds of the grid cell in world coords
    geometry_msgs::Point lower_left, upper_right;
    getCellBounds(gx, gy, lower_left, upper_right);

    //now we need to check what cells could contain the nearest neighbor
    pcl::PointXYZ check_point;
    double sq_dist = DBL_MAX;
    double neighbor_sq_dist = DBL_MAX;
    
    //left
    if(gx > 0){
      check_point.x = lower_left.x;
      check_point.y = pt.y;
      sq_dist = sq_distance(pt, check_point);
      if(sq_dist < sq_min_separation_)
        neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx - 1, gy));
    }

    //upper left
    if(gx > 0 && gy < height_ - 1){
      check_point.x = lower_left.x;
      check_point.y = upper_right.y;
      sq_dist = sq_distance(pt, check_point);
      if(sq_dist < sq_min_separation_)
        neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx - 1, gy + 1));
    }

    //top
    if(gy < height_ - 1){
      check_point.x = pt.x;
      check_point.y = upper_right.y;
      sq_dist = sq_distance(pt, check_point);
      if(sq_dist < sq_min_separation_)
        neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx, gy + 1));
    }

    //upper right
    if(gx < width_ - 1 && gy < height_ - 1){
      check_point.x = upper_right.x;
      check_point.y = upper_right.y;
      sq_dist = sq_distance(pt, check_point);
      if(sq_dist < sq_min_separation_)
        neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx + 1, gy + 1));
    }

    //right
    if(gx < width_ - 1){
      check_point.x = upper_right.x;
      check_point.y = pt.y;
      sq_dist = sq_distance(pt, check_point);
      if(sq_dist < sq_min_separation_)
        neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx + 1, gy));
    }

    //lower right
    if(gx < width_ - 1 && gy > 0){
      check_point.x = upper_right.x;
      check_point.y = lower_left.y;
      sq_dist = sq_distance(pt, check_point);
      if(sq_dist < sq_min_separation_)
        neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx + 1, gy - 1));
    }

    //bottom
    if(gy > 0){
      check_point.x = pt.x;
      check_point.y = lower_left.y;
      sq_dist = sq_distance(pt, check_point);
      if(sq_dist < sq_min_separation_)
        neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx, gy - 1));
    }

    //lower left
    if(gx > 0 && gy > 0){
      check_point.x = lower_left.x;
      check_point.y = lower_left.y;
      sq_dist = sq_distance(pt, check_point);
      if(sq_dist < sq_min_separation_)
        neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx - 1, gy - 1));
    }

    //we must also check within the cell we're in for a nearest neighbor
    neighbor_sq_dist = min(neighbor_sq_dist, getNearestInCell(pt, gx, gy));

    return neighbor_sq_dist;
  }

  void PointGrid::updateWorld(const vector<geometry_msgs::Point>& footprint, 
      const vector<Observation>& observations, const vector<PlanarLaserScan>& laser_scans){
    //for our 2D point grid we only remove freespace based on the first laser scan
    if(laser_scans.empty())
      return;

    removePointsInScanBoundry(laser_scans[0]);

    //iterate through all observations and update the grid
    for(vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it){
      const Observation& obs = *it;
      const pcl::PointCloud<pcl::PointXYZ>& cloud = (obs.cloud_);
      for(unsigned int i = 0; i < cloud.points.size(); ++i){
        //filter out points that are too high
        if(cloud.points[i].z > max_z_)
          continue;

        //compute the squared distance from the hitpoint to the pointcloud's origin
        double sq_dist = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
          + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y) 
          + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

        if(sq_dist >= sq_obstacle_range_)
          continue;

        //insert the point
        insert(cloud.points[i]);
      }
    }

    //remove the points that are in the footprint of the robot
    removePointsInPolygon(footprint);
  }

  void PointGrid::removePointsInScanBoundry(const PlanarLaserScan& laser_scan){
    if(laser_scan.cloud.points.size() == 0)
      return;

    //compute the containing square of the scan
    geometry_msgs::Point lower_left, upper_right;
    lower_left.x = laser_scan.origin.x;
    lower_left.y = laser_scan.origin.y;
    upper_right.x = laser_scan.origin.x;
    upper_right.y = laser_scan.origin.y;

    for(unsigned int i = 0; i < laser_scan.cloud.points.size(); ++i){
      lower_left.x = min(lower_left.x, (double)laser_scan.cloud.points[i].x);
      lower_left.y = min(lower_left.y, (double)laser_scan.cloud.points[i].y);
      upper_right.x = max(upper_right.x, (double)laser_scan.cloud.points[i].x);
      upper_right.y = max(upper_right.y, (double)laser_scan.cloud.points[i].y);
    }

    getPointsInRange(lower_left, upper_right, points_);

    //if there are no points in the containing square... we don't have to do anything
    if(points_.empty())
      return;

    //if there are points, we have to check them against the scan explicitly to remove them
    for(unsigned int i = 0; i < points_.size(); ++i){
      list<pcl::PointXYZ>* cell_points = points_[i];
      if(cell_points != NULL){
        list<pcl::PointXYZ>::iterator it = cell_points->begin();
        while(it != cell_points->end()){
          const pcl::PointXYZ& pt = *it;

          //check if the point is in the polygon and if it is, erase it from the grid
          if(ptInScan(pt, laser_scan)){
            it = cell_points->erase(it);
          }
          else
            it++;
        }
      }
    }
  }

  bool PointGrid::ptInScan(const pcl::PointXYZ& pt, const PlanarLaserScan& laser_scan){
    if(!laser_scan.cloud.points.empty()){
      //compute the angle of the point relative to that of the scan
      double v1_x = laser_scan.cloud.points[0].x - laser_scan.origin.x;
      double v1_y = laser_scan.cloud.points[0].y - laser_scan.origin.y;
      double v2_x = pt.x - laser_scan.origin.x;
      double v2_y = pt.y - laser_scan.origin.y;

      double perp_dot = v1_x * v2_y - v1_y * v2_x;
      double dot = v1_x * v2_x + v1_y * v2_y;

      //get the signed angle
      double vector_angle = atan2(perp_dot, dot);

      //we want all angles to be between 0 and 2PI
      if(vector_angle < 0)
        vector_angle = 2 * M_PI + vector_angle;

      double total_rads = laser_scan.angle_max - laser_scan.angle_min; 

      //if this point lies outside of the scan field of view... it is not in the scan
      if(vector_angle < 0 || vector_angle >= total_rads)
        return false;

      //compute the index of the point in the scan
      unsigned int index = (unsigned int) (vector_angle / laser_scan.angle_increment);

      //make sure we have a legal index... we always should at this point, but just in case
      if(index >= laser_scan.cloud.points.size() - 1){
        return false;
      }

      //if the point lies to the left of the line between the two scan points bounding it, it is within the scan
      if(orient(laser_scan.cloud.points[index], laser_scan.cloud.points[index + 1], pt) > 0){
        return true;
      }

      //otherwise it is not
      return false;
    }
    else
      return false;
  }

  void PointGrid::getPoints(pcl::PointCloud<pcl::PointXYZ>& cloud){
    for(unsigned int i = 0; i < cells_.size(); ++i){
      for(list<pcl::PointXYZ>::iterator it = cells_[i].begin(); it != cells_[i].end(); ++it){
        cloud.points.push_back(*it);
      }
    }
  }

  void PointGrid::removePointsInPolygon(const vector<geometry_msgs::Point> poly){
    if(poly.size() == 0)
      return;

    geometry_msgs::Point lower_left, upper_right;
    lower_left.x = poly[0].x;
    lower_left.y = poly[0].y;
    upper_right.x = poly[0].x;
    upper_right.y = poly[0].y;

    //compute the containing square of the polygon
    for(unsigned int i = 1; i < poly.size(); ++i){
      lower_left.x = min(lower_left.x, poly[i].x);
      lower_left.y = min(lower_left.y, poly[i].y);
      upper_right.x = max(upper_right.x, poly[i].x);
      upper_right.y = max(upper_right.y, poly[i].y);
    }

    ROS_DEBUG("Lower: (%.2f, %.2f), Upper: (%.2f, %.2f)\n", lower_left.x, lower_left.y, upper_right.x, upper_right.y);
    getPointsInRange(lower_left, upper_right, points_);

    //if there are no points in the containing square... we don't have to do anything
    if(points_.empty())
      return;

    //if there are points, we have to check them against the polygon explicitly to remove them
    for(unsigned int i = 0; i < points_.size(); ++i){
      list<pcl::PointXYZ>* cell_points = points_[i];
      if(cell_points != NULL){
        list<pcl::PointXYZ>::iterator it = cell_points->begin();
        while(it != cell_points->end()){
          const pcl::PointXYZ& pt = *it;

          //check if the point is in the polygon and if it is, erase it from the grid
          if(ptInPolygon(pt, poly)){
            it = cell_points->erase(it);
          }
          else
            it++;
        }
      }
    }
  }

  void PointGrid::intersectionPoint(const geometry_msgs::Point& v1, const geometry_msgs::Point& v2, 
      const geometry_msgs::Point& u1, const geometry_msgs::Point& u2, geometry_msgs::Point& result){
    //generate the equation for line 1
    double a1 = v2.y - v1.y;
    double b1 = v1.x - v2.x;
    double c1 = a1 * v1.x + b1 * v1.y;

    //generate the equation for line 2
    double a2 = u2.y - u1.y;
    double b2 = u1.x - u2.x;
    double c2 = a2 * u1.x + b2 * u1.y;

    double det = a1 * b2 - a2 * b1;

    //the lines are parallel
    if(det == 0)
      return;

    result.x = (b2 * c1 - b1 * c2) / det;
    result.y = (a1 * c2 - a2 * c1) / det;
  }

};


using namespace riskrrt_local_planner;

int main(int argc, char** argv){
  geometry_msgs::Point origin;
  origin.x = 0.0;
  origin.y = 0.0;
  PointGrid pg(50.0, 50.0, 0.2, origin, 2.0, 3.0, 0.0);
  /*
     double x = 10.0;
     double y = 10.0;
     for(int i = 0; i < 100; ++i){
     for(int j = 0; j < 100; ++j){
     pcl::PointXYZ pt;
     pt.x = x;
     pt.y = y;
     pt.z = 1.0;
     pg.insert(pt);
     x += .03;
     }
     y += .03;
     x = 10.0;
     }
     */
  vector<geometry_msgs::Point> footprint;
  geometry_msgs::Point pt;

  pt.x = 1.0;
  pt.y = 1.0;
  footprint.push_back(pt);

  pt.x = 1.0;
  pt.y = 1.65;
  footprint.push_back(pt);

  pt.x = 1.325;
  pt.y = 1.75;
  footprint.push_back(pt);

  pt.x = 1.65;
  pt.y = 1.65;
  footprint.push_back(pt);

  pt.x = 1.65;
  pt.y = 1.0;
  footprint.push_back(pt);

  vector<geometry_msgs::Point> footprint2;

  pt.x = 1.325;
  pt.y = 1.00;
  footprint2.push_back(pt);

  pt.x = 1.325;
  pt.y = 1.75;
  footprint2.push_back(pt);

  pt.x = 1.65;
  pt.y = 1.75;
  footprint2.push_back(pt);

  pt.x = 1.65;
  pt.y = 1.00;
  footprint2.push_back(pt);

  vector<geometry_msgs::Point> footprint3;

  pt.x = 0.99;
  pt.y = 0.99;
  footprint3.push_back(pt);

  pt.x = 0.99;
  pt.y = 1.66;
  footprint3.push_back(pt);

  pt.x = 1.3255;
  pt.y = 1.85;
  footprint3.push_back(pt);

  pt.x = 1.66;
  pt.y = 1.66;
  footprint3.push_back(pt);

  pt.x = 1.66;
  pt.y = 0.99;
  footprint3.push_back(pt);

  pt.x = 1.325;
  pt.y = 1.325;

  pcl::PointXYZ point;
  point.x = 1.2;
  point.y = 1.2;
  point.z = 1.0;

  struct timeval start, end;
  double start_t, end_t, t_diff;

  printPSHeader();

  gettimeofday(&start, NULL);
  for(unsigned int i = 0; i < 2000; ++i){
    pg.insert(point);
  }
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  printf("%%Insertion Time: %.9f \n", t_diff);

  vector<Observation> obs;
  vector<PlanarLaserScan> scan;

  gettimeofday(&start, NULL);
  pg.updateWorld(footprint, obs, scan);
  double legal = pg.footprintCost(pt, footprint, 0.0, .95);
  pg.updateWorld(footprint, obs, scan);
  double legal2 = pg.footprintCost(pt, footprint, 0.0, .95);
  gettimeofday(&end, NULL);
  start_t = start.tv_sec + double(start.tv_usec) / 1e6;
  end_t = end.tv_sec + double(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;

  printf("%%Footprint calc: %.9f \n", t_diff);

  if(legal >= 0.0)
    printf("%%Legal footprint %.4f, %.4f\n", legal, legal2);
  else
    printf("%%Illegal footprint\n");

  printPSFooter();

  return 0;
}
