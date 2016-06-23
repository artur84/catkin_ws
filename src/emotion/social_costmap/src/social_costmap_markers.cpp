#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <social_costmap/VoxelGrid.h>
#include <voxel_grid/voxel_grid.h>

#include "social_costmap/voxel_social_costmap.h"

struct Cell
{
  double x;
  double y;
  double z;
  voxel_grid::VoxelStatus status;
};
typedef std::vector<Cell> V_Cell;

float g_colors_r[] = { 0.0f, 0.0f, 1.0f };
float g_colors_g[] = { 0.0f, 0.0f, 0.0f };
float g_colors_b[] = { 0.0f, 1.0f, 0.0f };
float g_colors_a[] = { 0.0f, 0.5f, 1.0f };

std::string g_marker_ns;
V_Cell g_cells;
void voxelCallback(const ros::Publisher& pub, const social_costmap::VoxelGridConstPtr& grid)
{
  if (grid->data.empty())
  {
    ROS_ERROR("Received empty voxel grid");
    return;
  }

  ros::WallTime start = ros::WallTime::now();

  ROS_DEBUG("Received voxel grid");
  const std::string frame_id = grid->header.frame_id;
  const ros::Time stamp = grid->header.stamp;
  const uint32_t* data = &grid->data.front();
  const double x_origin = grid->origin.x;
  const double y_origin = grid->origin.y;
  const double z_origin = grid->origin.z;
  const double x_res = grid->resolutions.x;
  const double y_res = grid->resolutions.y;
  const double z_res = grid->resolutions.z;
  const uint32_t x_size = grid->size_x;
  const uint32_t y_size = grid->size_y;
  const uint32_t z_size = grid->size_z;

  g_cells.clear();
  uint32_t num_markers = 0;
  for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid)
  {
    for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid)
    {
      for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid)
      {
        voxel_grid::VoxelStatus status = voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid, z_grid, x_size, y_size, z_size, data);

        if (status == voxel_grid::MARKED)
        {
          Cell c;
          c.status = status;
          social_costmap::VoxelSocialCostmap::mapToWorld3D(x_grid, y_grid, z_grid, x_origin, y_origin, z_origin, x_res, y_res, z_res, c.x, c.y, c.z);

          g_cells.push_back(c);

          ++num_markers;
        }
      }
    }
  }

  visualization_msgs::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = g_marker_ns;
  m.id = 0;
  m.type = visualization_msgs::Marker::CUBE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = x_res;
  m.scale.y = y_res;
  m.scale.z = z_res;
  m.color.r = g_colors_r[voxel_grid::MARKED];
  m.color.g = g_colors_g[voxel_grid::MARKED];
  m.color.b = g_colors_b[voxel_grid::MARKED];
  m.color.a = g_colors_a[voxel_grid::MARKED];
  m.points.resize(num_markers);
  for (uint32_t i = 0; i < num_markers; ++i)
  {
    Cell& c = g_cells[i];
    geometry_msgs::Point& p = m.points[i];
    p.x = c.x;
    p.y = c.y;
    p.z = c.z;
  }

  pub.publish(m);

  ros::WallTime end = ros::WallTime::now();
  ROS_DEBUG("Published %d markers in %f seconds", num_markers, (end - start).toSec());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "social_costmap_markers");
  ros::NodeHandle n;

  ROS_DEBUG("Startup");

  ros::Publisher pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe<social_costmap::VoxelGrid>("voxel_grid", 1, boost::bind(voxelCallback, pub, _1));
  g_marker_ns = n.resolveName("voxel_grid");

  ros::spin();
}
