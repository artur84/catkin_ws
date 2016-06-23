#include <riskrrt_local_planner/obstacle_cost_function.h>
#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

namespace riskrrt_local_planner {

ObstacleCostFunction::ObstacleCostFunction(costmap_2d::Costmap2D* costmap) : costmap_(costmap) {
  if (costmap != NULL) {
    world_model_ = new riskrrt_local_planner::CostmapModel(*costmap_);
  }
}

ObstacleCostFunction::~ObstacleCostFunction() {
  if (world_model_ != NULL) {
    delete world_model_;
  }
}


void ObstacleCostFunction::setParams(double max_trans_vel, double max_scaling_factor, double scaling_speed) {
  // TODO: move this to prepare if possible
  max_trans_vel_ = max_trans_vel;
  max_scaling_factor_ = max_scaling_factor;
  scaling_speed_ = scaling_speed;
}

void ObstacleCostFunction::setFootprint(std::vector<geometry_msgs::Point> footprint_spec) {
  footprint_spec_ = footprint_spec;
}

bool ObstacleCostFunction::prepare() {
  return true;
}

double ObstacleCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0;
  double scale = getScalingFactor(traj, scaling_speed_, max_trans_vel_, max_scaling_factor_);
  double px, py, pth;
  if (footprint_spec_.size() == 0) {
    // Bug, should never happen
    ROS_ERROR("Footprint spec is empty, maybe missing call to setFootprint?");
    return -9;
  }

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    cost = footprintCost(px, py, pth,
        scale,
        footprint_spec_,
        costmap_,
        world_model_);
  }
  return cost;
}

double ObstacleCostFunction::getScalingFactor(Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor) {
  double vmag = sqrt(traj.xv_ * traj.xv_ + traj.yv_ * traj.yv_);

  //if we're over a certain speed threshold, we'll scale the robot's
  //footprint to make it either slow down or stay further from walls
  double scale = 1.0;
  if (vmag > scaling_speed) {
    //scale up to the max scaling factor linearly... this could be changed later
    double ratio = (vmag - scaling_speed) / (max_trans_vel - scaling_speed);
    scale = max_scaling_factor * ratio + 1.0;
  }
  return scale;
}

double ObstacleCostFunction::footprintCost (
    const double& x,
    const double& y,
    const double& th,
    double scale,
    std::vector<geometry_msgs::Point>& footprint_spec,
    costmap_2d::Costmap2D* costmap,
    riskrrt_local_planner::WorldModel* world_model) {
  double cos_th = cos(th);
  double sin_th = sin(th);

  double occ_cost = 0.0;

  std::vector<geometry_msgs::Point> scaled_oriented_footprint;
  for(unsigned int i  = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::Point new_pt;
    new_pt.x = x + (scale * footprint_spec[i].x * cos_th - scale * footprint_spec[i].y * sin_th);
    new_pt.y = y + (scale * footprint_spec[i].x * sin_th + scale * footprint_spec[i].y * cos_th);
    scaled_oriented_footprint.push_back(new_pt);
    geometry_msgs::Point robot_position;
    robot_position.x = x;
    robot_position.y = y;

    //check if the footprint is legal
    double footprint_cost = world_model->footprintCost(robot_position,
    		scaled_oriented_footprint,
    		costmap->getInscribedRadius(),
    		costmap->getCircumscribedRadius());

    if (footprint_cost < 0) {
      return -6.0;
    }
    unsigned int cell_x, cell_y;

    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if ( ! costmap->worldToMap(x, y, cell_x, cell_y)) {
      return -7.0;
    }

    occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap->getCost(cell_x, cell_y)));

  }

  return occ_cost;
}

} /* namespace riskrrt_local_planner */
