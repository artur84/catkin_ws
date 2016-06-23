#include <ui_local_planner/trajectory_planner.h>

#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>
#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>

using namespace std;
using namespace costmap_2d;

namespace ui_local_planner
{

void TrajectoryPlanner::reconfigure(UILocalPlannerConfig &cfg)
{
	UILocalPlannerConfig config(cfg);

	boost::mutex::scoped_lock l(configuration_mutex_);

	acc_lim_x_ = config.acc_lim_x;
	acc_lim_y_ = config.acc_lim_y;
	acc_lim_theta_ = config.acc_lim_theta;

	max_vel_x_ = config.max_vel_x;
	min_vel_x_ = config.min_vel_x;

	max_vel_th_ = config.max_vel_theta;
	min_vel_th_ = config.min_vel_theta;
	min_in_place_vel_th_ = config.min_in_place_vel_theta;

	sim_time_ = config.sim_time;
	sim_granularity_ = config.sim_granularity;
	angular_sim_granularity_ = config.angular_sim_granularity;

	pdist_scale_ = config.pdist_scale;
	gdist_scale_ = config.gdist_scale;
	occdist_scale_ = config.occdist_scale;

	if (meter_scoring_)
	{
		//if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
		double resolution = costmap_.getResolution();
		gdist_scale_ *= resolution;
		pdist_scale_ *= resolution;
		occdist_scale_ *= resolution;
	}

	oscillation_reset_dist_ = config.oscillation_reset_dist;
	escape_reset_dist_ = config.escape_reset_dist;
	escape_reset_theta_ = config.escape_reset_theta;

	vx_samples_ = config.vx_samples;
	vtheta_samples_ = config.vtheta_samples;

	if (vx_samples_ <= 0)
	{
		config.vx_samples = 1;
		vx_samples_ = config.vx_samples;
		ROS_WARN(
				"You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
	}
	if (vtheta_samples_ <= 0)
	{
		config.vtheta_samples = 1;
		vtheta_samples_ = config.vtheta_samples;
		ROS_WARN(
				"You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
	}

	heading_lookahead_ = config.heading_lookahead;

	holonomic_robot_ = config.holonomic_robot;

	backup_vel_ = config.escape_vel;

	dwa_ = config.dwa;

	heading_scoring_ = config.heading_scoring;

	heading_scoring_timestep_ = config.heading_scoring_timestep;

	simple_attractor_ = config.simple_attractor;

	//y-vels
	string y_string = config.y_vels;
	vector<string> y_strs;
	boost::split(y_strs, y_string, boost::is_any_of(", "),
			boost::token_compress_on);

	vector<double> y_vels;
	for (vector<string>::iterator it = y_strs.begin(); it != y_strs.end(); ++it)
	{
		istringstream iss(*it);
		double temp;
		iss >> temp;
		y_vels.push_back(temp);
		//ROS_INFO("Adding y_vel: %e", temp);
	}

	y_vels_ = y_vels;

	ui_scoring_ = config.ui_scoring;
	try_user_vel_first_ = config.try_user_vel_first;
	center_dyn_win_on_user_vel_ = config.center_dyn_win_on_user_vel;
	user_vel_cost_scale_ = config.user_vel_cost_scale;
	face_dir_cost_scale_ = config.face_dir_cost_scale;
	new_user_vel_ = false;
	new_face_dir_ = false;

}

TrajectoryPlanner::TrajectoryPlanner(WorldModel& world_model,
		const Costmap2D& costmap,
		std::vector<geometry_msgs::Point> footprint_spec, double acc_lim_x,
		double acc_lim_y, double acc_lim_theta, double sim_time,
		double sim_granularity, int vx_samples, int vtheta_samples,
		double pdist_scale, double gdist_scale, double occdist_scale,
		double heading_lookahead, double oscillation_reset_dist,
		double escape_reset_dist, double escape_reset_theta,
		bool holonomic_robot, double max_vel_x, double min_vel_x,
		double max_vel_th, double min_vel_th, double min_in_place_vel_th,
		double backup_vel, bool dwa, bool heading_scoring,
		double heading_scoring_timestep, bool meter_scoring,
		bool simple_attractor, vector<double> y_vels, double stop_time_buffer,
		double sim_period, double angular_sim_granularity, bool ui_scoring,
		bool try_user_vel_first, bool center_dyn_win_on_user_vel,
		double user_vel_cost_scale, double face_dir_cost_scale) : //XXX
		path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()), goal_map_(
				costmap.getSizeInCellsX(), costmap.getSizeInCellsY()), costmap_(
				costmap), world_model_(world_model), footprint_spec_(
				footprint_spec), sim_time_(sim_time), sim_granularity_(
				sim_granularity), angular_sim_granularity_(
				angular_sim_granularity), vx_samples_(vx_samples), vtheta_samples_(
				vtheta_samples), pdist_scale_(pdist_scale), gdist_scale_(
				gdist_scale), occdist_scale_(occdist_scale), acc_lim_x_(
				acc_lim_x), acc_lim_y_(acc_lim_y), acc_lim_theta_(
				acc_lim_theta), prev_x_(0), prev_y_(0), escape_x_(0), escape_y_(
				0), escape_theta_(0), heading_lookahead_(heading_lookahead), oscillation_reset_dist_(
				oscillation_reset_dist), escape_reset_dist_(escape_reset_dist), escape_reset_theta_(
				escape_reset_theta), holonomic_robot_(holonomic_robot), max_vel_x_(
				max_vel_x), min_vel_x_(min_vel_x), max_vel_th_(max_vel_th), min_vel_th_(
				min_vel_th), min_in_place_vel_th_(min_in_place_vel_th), backup_vel_(
				backup_vel), dwa_(dwa), heading_scoring_(heading_scoring), heading_scoring_timestep_(
				heading_scoring_timestep), simple_attractor_(simple_attractor), y_vels_(
				y_vels), stop_time_buffer_(stop_time_buffer), sim_period_(
				sim_period), ui_scoring_(ui_scoring), try_user_vel_first_(
				try_user_vel_first), center_dyn_win_on_user_vel_(
				center_dyn_win_on_user_vel), user_vel_cost_scale_(
				user_vel_cost_scale), face_dir_cost_scale_(
				face_dir_cost_scale)
{
	//the robot is not stuck to begin with
	stuck_left = false;
	stuck_right = false;
	stuck_left_strafe = false;
	stuck_right_strafe = false;
	rotating_left = false;
	rotating_right = false;
	strafe_left = false;
	strafe_right = false;

	escaping_ = false;
	final_goal_position_valid_ = false;

	user_vel_cost_ = 0.0;
	setUserInput(0, 0, 0);
}

TrajectoryPlanner::~TrajectoryPlanner()
{
}

bool TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost,
		float &goal_cost, float &occ_cost, float &total_cost)
{
	MapCell cell = path_map_(cx, cy);
	MapCell goal_cell = goal_map_(cx, cy);
	if (cell.within_robot)
	{
		return false;
	}
	occ_cost = costmap_.getCost(cx, cy);
	if (cell.target_dist == path_map_.obstacleCosts()
			|| cell.target_dist == path_map_.unreachableCellCosts()
			|| occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
	{
		return false;
	}
	path_cost = cell.target_dist;
	goal_cost = goal_cell.target_dist;
	total_cost = pdist_scale_ * path_cost + gdist_scale_ * goal_cost
			+ occdist_scale_ * occ_cost;
	return true;
}

/**
 * XXX create and score a trajectory given the current pose of the robot and selected velocities
 */
void TrajectoryPlanner::generateTrajectory(double x, double y, double theta, //Initial (current real pose of the robot)
		double vx, double vy, double vtheta, //Initial (current real velocity)
		double vx_samp, double vy_samp, double vtheta_samp, //The desired velocity, what we're accelerating up to
		double acc_x, double acc_y, double acc_theta, //Acceleration limits of the robot
		double impossible_cost, Trajectory& traj)
{
	// make sure the configuration doesn't change mid run
	boost::mutex::scoped_lock l(configuration_mutex_);

	double x_i = x; //Starting (current) x location
	double y_i = y; //Starting (current) y location
	double theta_i = theta; //Starting theta location

	double vx_i, vy_i, vtheta_i; ////Initial velocities

	vx_i = vx;
	vy_i = vy;
	vtheta_i = vtheta;

	//compute the magnitude of the velocities
	double vmag = sqrt(vx_samp * vx_samp + vy_samp * vy_samp);

	//compute the number of steps we must take along this trajectory to be "safe"
	int num_steps;
	if (!heading_scoring_)
	{
		num_steps = int(
				max((vmag * sim_time_) / sim_granularity_,
						fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);
	}
	else
	{
		num_steps = int(sim_time_ / sim_granularity_ + 0.5);
	}

	//we at least want to take one step... even if we won't move, we want to score our current position
	if (num_steps == 0)
	{
		num_steps = 1;
	}

	double dt = sim_time_ / num_steps;
	double time = 0.0;

	//create a potential trajectory
	traj.resetPoints();
	traj.xv_ = vx_samp;
	traj.yv_ = vy_samp;
	traj.thetav_ = vtheta_samp;
	traj.cost_ = -1.0;

	//initialize the costs for the trajectory
	double path_dist = 0.0;
	double goal_dist = 0.0;
	double occ_cost = 0.0;
	double heading_diff = 0.0;

	for (int i = 0; i < num_steps; ++i)
	{
		//get map coordinates of a point
		unsigned int cell_x, cell_y;

		//we don't want a path that goes off the know map
		if (!costmap_.worldToMap(x_i, y_i, cell_x, cell_y))
		{
			traj.cost_ = -1.0;
			return;
		}

		//check the point on the trajectory for legality
		double footprint_cost = footprintCost(x_i, y_i, theta_i); //it has values from 0 to 255
		//ROS_INFO("x_i=%f, y_i=%f, theta_i=%f, footprint_cost=%f", x_i, y_i, theta_i,  footprint_cost );
		//if the footprint hits an obstacle this trajectory is invalid
		if (footprint_cost < 0)
		{
			traj.cost_ = -1.0;
			return;
			//TODO: Really look at getMaxSpeedToStopInTime... dues to discretization errors and high acceleration limits,
			//it can actually cause the robot to hit obstacles. There may be something to be done to fix, but I'll have to
			//come back to it when I have time. Right now, pulling it out as it'll just make the robot a bit more conservative,
			//but safe.
			/*
			 double max_vel_x, max_vel_y, max_vel_th;
			 //we want to compute the max allowable speeds to be able to stop
			 //to be safe... we'll make sure we can stop some time before we actually hit
			 getMaxSpeedToStopInTime(time - stop_time_buffer_ - dt, max_vel_x, max_vel_y, max_vel_th);

			 //check if we can stop in time
			 if(fabs(vx_samp) < max_vel_x && fabs(vy_samp) < max_vel_y && fabs(vtheta_samp) < max_vel_th){
			 ROS_ERROR("v: (%.2f, %.2f, %.2f), m: (%.2f, %.2f, %.2f) t:%.2f, st: %.2f, dt: %.2f", vx_samp, vy_samp, vtheta_samp, max_vel_x, max_vel_y, max_vel_th, time, stop_time_buffer_, dt);
			 //if we can stop... we'll just break out of the loop here.. no point in checking future points
			 break;
			 }
			 else{
			 traj.cost_ = -1.0;
			 return;
			 }
			 */
		}

		occ_cost = std::max(std::max(occ_cost, footprint_cost),
				double(costmap_.getCost(cell_x, cell_y)));

		//do we want to follow blindly
		if (simple_attractor_)
		{
			goal_dist =
					(x_i - global_plan_[global_plan_.size() - 1].pose.position.x)
							* (x_i
									- global_plan_[global_plan_.size() - 1].pose.position.x)
							+ (y_i
									- global_plan_[global_plan_.size() - 1].pose.position.y)
									* (y_i
											- global_plan_[global_plan_.size()
													- 1].pose.position.y);//linear dstance from traj starting point till the goal.
		}
		else
		{

			bool update_path_and_goal_distances = true;

			// with heading scoring, we take into account heading diff, and also only score
			// path and goal distance for one point of the trajectory
			if (heading_scoring_)
			{
				if (time >= heading_scoring_timestep_
						&& time < heading_scoring_timestep_ + dt)
				{
					heading_diff = headingDiff(cell_x, cell_y, x_i, y_i,
							theta_i);
				}
				else
				{
					update_path_and_goal_distances = false;
				}
			}

			if (update_path_and_goal_distances)
			{
				//update path and goal distances
				path_dist = path_map_(cell_x, cell_y).target_dist;
				goal_dist = goal_map_(cell_x, cell_y).target_dist;

				//if a point on this trajectory has no clear path to goal it is invalid
				if (impossible_cost <= goal_dist
						|| impossible_cost <= path_dist)
				{
					//            ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f",
					//                goal_dist, path_dist, impossible_cost);
					traj.cost_ = -2.0;
					return;
				}
			}
		}

		//the point is legal... add it to the trajectory
		traj.addPoint(x_i, y_i, theta_i);
		//calculate velocities
		vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
		vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
		vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

		//calculate positions
		x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
		y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
		theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

		//increment time
		time += dt;
	} // end for i < numsteps

	//ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
	//XXX COST FUNCTION
	double cost = -1.0;
	if (ui_scoring_)
	{
		user_vel_cost_ = userInputDiff(vx_samp, vy_samp, vtheta_samp);
		//ROS_DEBUG("Using user input vel cost with (weight, cost): %f", user_vel_cost_scale_);
		//ROS_DEBUG("Using user face dir cost with (weight, cost): %f", face_dir_cost_scale_);
		if (!heading_scoring_)
		{
			cost = pdist_scale_ * path_dist + goal_dist * gdist_scale_
					+ occdist_scale_ * occ_cost
					+ user_vel_cost_scale_ * user_vel_cost_;//TODO: Add smthg here
		}
		else
		{
			cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist
					+ 0.3 * heading_diff + goal_dist * gdist_scale_
					+ user_vel_cost_scale_ * user_vel_cost_;

		}
	}
	else
	{
		if (!heading_scoring_)
		{
			cost = pdist_scale_ * path_dist + goal_dist * gdist_scale_
					+ occdist_scale_ * occ_cost;
		}
		else
		{
			cost = occdist_scale_ * occ_cost + pdist_scale_ * path_dist
					+ 0.3 * heading_diff + goal_dist * gdist_scale_;
		}
	}
	traj.cost_ = cost; //Normal costs depend mainly on the distance to the goal and in hall_inria scenario go from 0->closer, to 5 or 6 (farther).
	ROS_DEBUG("cost: %f, vtheta_traj: %f, vtheta_user: %f", cost, vtheta_samp,
			user_vel_.angular.z);
}

double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x,
		double y, double heading)
{
	double heading_diff = DBL_MAX;
	unsigned int goal_cell_x, goal_cell_y;
	//find a clear line of sight from the robot's cell to a point on the path
	for (int i = global_plan_.size() - 1; i >= 0; --i)
	{
		if (costmap_.worldToMap(global_plan_[i].pose.position.x,
				global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y))
		{
			if (lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0)
			{
				double gx, gy;
				costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
				double v1_x = gx - x;
				double v1_y = gy - y;
				double v2_x = cos(heading);
				double v2_y = sin(heading);

				double perp_dot = v1_x * v2_y - v1_y * v2_x;
				double dot = v1_x * v2_x + v1_y * v2_y;

				//get the signed angle
				double vector_angle = atan2(perp_dot, dot);

				heading_diff = fabs(vector_angle);
				return heading_diff;

			}
		}
	}
	return heading_diff;
}

//calculate the cost of a ray-traced line
double TrajectoryPlanner::lineCost(int x0, int x1, int y0, int y1)
{
	//Bresenham Ray-Tracing
	int deltax = abs(x1 - x0);        // The difference between the x's
	int deltay = abs(y1 - y0);        // The difference between the y's
	int x = x0;                       // Start x off at the first pixel
	int y = y0;                       // Start y off at the first pixel

	int xinc1, xinc2, yinc1, yinc2;
	int den, num, numadd, numpixels;

	double line_cost = 0.0;
	double point_cost = -1.0;

	if (x1 >= x0)                 // The x-values are increasing
	{
		xinc1 = 1;
		xinc2 = 1;
	}
	else                          // The x-values are decreasing
	{
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y1 >= y0)                 // The y-values are increasing
	{
		yinc1 = 1;
		yinc2 = 1;
	}
	else                          // The y-values are decreasing
	{
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay)     // There is at least one x-value for every y-value
	{
		xinc1 = 0;           // Don't change the x when numerator >= denominator
		yinc2 = 0;                  // Don't change the y for every iteration
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;         // There are more x-values than y-values
	}
	else
	{                      // There is at least one y-value for every x-value
		xinc2 = 0;                  // Don't change the x for every iteration
		yinc1 = 0;           // Don't change the y when numerator >= denominator
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;         // There are more y-values than x-values
	}

	for (int curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		point_cost = pointCost(x, y); //Score the current point

		if (point_cost < 0)
		{
			return -1;
		}

		if (line_cost < point_cost)
		{
			line_cost = point_cost;
		}

		num += numadd;      // Increase the numerator by the top of the fraction
		if (num >= den)
		{           // Check if numerator >= denominator
			num -= den;               // Calculate the new numerator value
			x += xinc1;               // Change the x as appropriate
			y += yinc1;               // Change the y as appropriate
		}
		x += xinc2;                 // Change the x as appropriate
		y += yinc2;                 // Change the y as appropriate
	}

	return line_cost;
}

double TrajectoryPlanner::pointCost(int x, int y)
{
	unsigned char cost = costmap_.getCost(x, y);
	//if the cell is in an obstacle the path is invalid
	if (cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE
			|| cost == NO_INFORMATION)
	{
		return -1;
	}

	return cost;
}

void TrajectoryPlanner::updatePlan(
		const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists)
{
	global_plan_.resize(new_plan.size());
	for (unsigned int i = 0; i < new_plan.size(); ++i)
	{
		global_plan_[i] = new_plan[i];
	}

	if (global_plan_.size() > 0)
	{
		geometry_msgs::PoseStamped& final_goal_pose =
				global_plan_[global_plan_.size() - 1];
		final_goal_x_ = final_goal_pose.pose.position.x;
		final_goal_y_ = final_goal_pose.pose.position.y;
		final_goal_position_valid_ = true;
	}
	else
	{
		final_goal_position_valid_ = false;
	}

	if (compute_dists)
	{
		//reset the map for new operations
		path_map_.resetPathDist();
		goal_map_.resetPathDist();

		//make sure that we update our path based on the global plan and compute costs
		path_map_.setTargetCells(costmap_, global_plan_);
		goal_map_.setLocalGoal(costmap_, global_plan_);
		ROS_DEBUG("Path/Goal distance computed");
	}
}

bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta,
		double vx, double vy, double vtheta, double vx_samp, double vy_samp,
		double vtheta_samp)
{
	//Check if vx_samp,vy_samp,vtheta_samp is valid, true if valid, false otherwise

	double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp,
			vtheta_samp);

	//if the trajectory is a legal one... the check passes
	if (cost >= 0)
	{
		return true;
	}
	ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vx_samp, vy_samp,
			vtheta_samp, cost);

	//otherwise the check fails
	return false;
}

double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta,
		double vx, double vy, double vtheta, double vx_samp, double vy_samp,
		double vtheta_samp)
{
	// Scores a single trajectory returns the score.
	Trajectory t;
	double impossible_cost = path_map_.obstacleCosts();
	generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp,
			vtheta_samp, acc_lim_x_, acc_lim_y_, acc_lim_theta_,
			impossible_cost, t);

	// return the cost.
	return double(t.cost_);
}

double TrajectoryPlanner::getTrajectory(double x, double y, double theta,
		double vx, double vy, double vtheta, double vx_samp, double vy_samp,
		double vtheta_samp, Trajectory &trajectory)
{
	/*
	 * When using ui_move_base this function is used to test if the trajectory simulated using vx, and vtheta is valid.
	 */

	double impossible_cost = path_map_.obstacleCosts();
	generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp,
			vtheta_samp, acc_lim_x_, acc_lim_y_, acc_lim_theta_,
			impossible_cost, trajectory);

	// return the cost.
	return double(trajectory.cost_);
}

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i)
{
	//build the oriented footprint
	double cos_th = cos(theta_i);
	double sin_th = sin(theta_i);
	vector<geometry_msgs::Point> oriented_footprint;
	for (unsigned int i = 0; i < footprint_spec_.size(); ++i)
	{
		geometry_msgs::Point new_pt;
		new_pt.x =
				x_i
						+ (footprint_spec_[i].x * cos_th
								- footprint_spec_[i].y * sin_th);
		new_pt.y =
				y_i
						+ (footprint_spec_[i].x * sin_th
								+ footprint_spec_[i].y * cos_th);
		oriented_footprint.push_back(new_pt);
	}

	geometry_msgs::Point robot_position;
	robot_position.x = x_i;
	robot_position.y = y_i;

	//check if the footprint is legal
	double footprint_cost = world_model_.footprintCost(robot_position,
			oriented_footprint, costmap_.getInscribedRadius(),
			costmap_.getCircumscribedRadius());

	return footprint_cost;
}

void TrajectoryPlanner::getLocalGoal(double& x, double& y)
{
	x = path_map_.goal_x_;
	y = path_map_.goal_y_;
}

/************************************************************************************************************
 * Artur work
 ***********************************************************************************************************/
/* XXX create the trajectories we wish to score
 * THIS WHERE THE DYNAMIC WINDOW is created.
 *
 */
Trajectory TrajectoryPlanner::createTrajectoriesVector(double x, double y,
		double theta, double vx, double vy, double vtheta, double acc_x,
		double acc_y, double acc_theta, vector<Trajectory> & evaluated_trajs) //Vector of all the evaluated trajectories for visualizing
{
	//compute feasible velocity limits in robot space
	double max_vel_x = max_vel_x_, max_vel_theta;
	double min_vel_x, min_vel_theta;

	if (final_goal_position_valid_)
	{
		double final_goal_dist = hypot(final_goal_x_ - x, final_goal_y_ - y);
		max_vel_x = min(max_vel_x, final_goal_dist / sim_time_);
	}

	//Check if velocities are feasible under acceleration constrains
	max_vel_x = max(min(max_vel_x, vx + acc_x * sim_period_), min_vel_x_);
	min_vel_x = max(min_vel_x_, vx - acc_x * sim_period_);

	max_vel_theta = min(max_vel_th_, vtheta + acc_theta * sim_period_);
	min_vel_theta = max(min_vel_th_, vtheta - acc_theta * sim_period_);

	// By forcing this min and max values I'm overriding the previous feasible check so maybe is not a good idea.
	//however it gives more control to the user.
	if (center_dyn_win_on_user_vel_)
	{
		if (user_vel_.linear.x < 0)//if the user gives a "backward" command then the min linear vel should be negative with the value given by the user
		{
			min_vel_x = -min_vel_x;
			max_vel_x = -max_vel_x;
		}
		min_vel_x = min_vel_x_;
		max_vel_x = user_vel_.linear.x + 0.2; //TODO:fix it
		min_vel_theta = user_vel_.angular.z - fabs(min_vel_th_);
		max_vel_theta = user_vel_.angular.z + fabs(max_vel_th_);
	}
	//we want to sample the velocity space regularly
	double dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
	double dvtheta = (max_vel_theta - min_vel_theta) / (vtheta_samples_ - 1);

	double vx_samp = min_vel_x;
	double vtheta_samp = min_vel_theta;
	double vy_samp = 0.0;

	//keep track of the best trajectory seen so far
	Trajectory* best_traj = &traj_one;
	best_traj->cost_ = -1.0;

	Trajectory* comp_traj = &traj_two; //Trajectory to be compared with last best trajectory
	comp_traj->cost_ = -1.0;

	Trajectory* swap = NULL;

	//any cell with a cost greater than the size of the map is impossible
	double impossible_cost = path_map_.obstacleCosts();

	//Only when we are not in escape mode we will consider straight trajectories
	if (!escaping_)
	{
		ROS_DEBUG("min vel: %f", min_vel_x);
		//First check the user's input and publish it directly if it is valid
		if (try_user_vel_first_)
		{
			generateTrajectory(x, y, theta, vx, vy, vtheta,
					user_vel_.linear.x, 0.0, user_vel_.angular.z, acc_x,
					acc_y, acc_theta, impossible_cost, *comp_traj);
			evaluated_trajs.push_back(*comp_traj);
			if (comp_traj->cost_ >= 0) //if the trajectory is valid
				return *comp_traj;
		} //Add this trajectory to the list
		  //loop through all x velocities
		for (int i = 0; i < vx_samples_; ++i)
		{
//			vtheta_samp = 0;
//			//first sample the straight trajectory
//			generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp,
//					vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost,
//					*comp_traj);
//			evaluated_trajs.push_back(*comp_traj); //Add this trajectory to the list
//			//if the new trajectory is better... let's take it
//			if (comp_traj->cost_ >= 0
//					&& (comp_traj->cost_ < best_traj->cost_
//							|| best_traj->cost_ < 0))
//			{
//				swap = best_traj;
//				best_traj = comp_traj;
//				comp_traj = swap;
//			}

			vtheta_samp = min_vel_theta; //The dynamic window will be centered on the required user command
			//next sample all theta trajectories
			for (int j = 0; j < vtheta_samples_ - 1; ++j)
			{
				generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp,
						vy_samp, vtheta_samp, acc_x, acc_y, acc_theta,
						impossible_cost, *comp_traj);
				evaluated_trajs.push_back(*comp_traj); //Add this trajectory to the list

				//if the new trajectory is better... let's take it
				if (comp_traj->cost_ >= 0
						&& (comp_traj->cost_ < best_traj->cost_
								|| best_traj->cost_ < 0))
				{
					swap = best_traj;
					best_traj = comp_traj;
					comp_traj = swap;
				}
				vtheta_samp += dvtheta;
			}
			vx_samp += dvx;
		}

	} // end if not escaping

	//next we want to generate trajectories for rotating in place
	vtheta_samp = min_vel_theta;
	vx_samp = 0.0;
	vy_samp = 0.0;

	//let's try to rotate toward open space
	double heading_dist = DBL_MAX;

	for (int i = 0; i < vtheta_samples_; ++i)
	{
		//enforce a minimum rotational velocity because the base can't handle small in-place rotations
		double vtheta_samp_limited =
				vtheta_samp > 0 ?
						max(vtheta_samp, min_in_place_vel_th_) :
						min(vtheta_samp, -1.0 * min_in_place_vel_th_);

		generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp,
				vtheta_samp_limited, acc_x, acc_y, acc_theta, impossible_cost,
				*comp_traj);
		evaluated_trajs.push_back(*comp_traj); //Add this trajectory to the list

		//if the new trajectory is better... let's take it...
		//note if we can legally rotate in place we prefer to do that rather than move with y velocity
		if (comp_traj->cost_ >= 0
				&& (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0
						|| best_traj->yv_ != 0.0)
				&& (vtheta_samp > dvtheta || vtheta_samp < -1 * dvtheta))
		{
			double x_r, y_r, th_r;
			comp_traj->getEndpoint(x_r, y_r, th_r);
			x_r += heading_lookahead_ * cos(th_r);
			y_r += heading_lookahead_ * sin(th_r);
			unsigned int cell_x, cell_y;

			//make sure that we'll be looking at a legal cell
			if (costmap_.worldToMap(x_r, y_r, cell_x, cell_y))
			{
				double ahead_gdist = goal_map_(cell_x, cell_y).target_dist;
				if (ahead_gdist < heading_dist)
				{
					//if we haven't already tried rotating left since we've moved forward
					if (vtheta_samp < 0 && !stuck_left)
					{
						swap = best_traj;
						best_traj = comp_traj;
						comp_traj = swap;
						heading_dist = ahead_gdist;
					}
					//if we haven't already tried rotating right since we've moved forward
					else if (vtheta_samp > 0 && !stuck_right)
					{
						swap = best_traj;
						best_traj = comp_traj;
						comp_traj = swap;
						heading_dist = ahead_gdist;
					}
				}
			}
		}

		vtheta_samp += dvtheta;
	}

	//If we have a legal trajectory
	if (best_traj->cost_ >= 0)
	{
		//XXX avoid oscillations of in place rotation and in place strafing
		if (!(best_traj->xv_ > 0))
		{
			if (best_traj->thetav_ < 0)
			{
				if (rotating_right)
				{
					stuck_right = true;
				}
				rotating_left = true;
			}
			else if (best_traj->thetav_ > 0)
			{
				if (rotating_left)
				{
					stuck_left = true;
				}
				rotating_right = true;
			}
			else if (best_traj->yv_ > 0)
			{
				if (strafe_right)
				{
					stuck_right_strafe = true;
				}
				strafe_left = true;
			}
			else if (best_traj->yv_ < 0)
			{
				if (strafe_left)
				{
					stuck_left_strafe = true;
				}
				strafe_right = true;
			}

			//set the position we must move a certain distance away from
			prev_x_ = x;
			prev_y_ = y;
		}

		double dist = sqrt(
				(x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
		if (dist > oscillation_reset_dist_)
		{
			rotating_left = false;
			rotating_right = false;
			strafe_left = false;
			strafe_right = false;
			stuck_left = false;
			stuck_right = false;
			stuck_left_strafe = false;
			stuck_right_strafe = false;
		}

		dist = sqrt(
				(x - escape_x_) * (x - escape_x_)
						+ (y - escape_y_) * (y - escape_y_));
		if (dist > escape_reset_dist_
				|| fabs(angles::shortest_angular_distance(escape_theta_, theta))
						> escape_reset_theta_)
		{
			escaping_ = false;
		}

		return *best_traj;
	}

	//and finally, if we can't do anything else, we want to generate trajectories that move backwards slowly
	vtheta_samp = 0.0;
	vx_samp = backup_vel_;
	vy_samp = 0.0;
	generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp,
			vtheta_samp, acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
	evaluated_trajs.push_back(*comp_traj); //Add this trajectory to the list
	//we'll allow moving backwards slowly even when the static map shows it as blocked
	swap = best_traj;
	best_traj = comp_traj;
	comp_traj = swap;

	double dist = sqrt(
			(x - prev_x_) * (x - prev_x_) + (y - prev_y_) * (y - prev_y_));
	if (dist > oscillation_reset_dist_)
	{
		rotating_left = false;
		rotating_right = false;
		strafe_left = false;
		strafe_right = false;
		stuck_left = false;
		stuck_right = false;
		stuck_left_strafe = false;
		stuck_right_strafe = false;
	}

	//only enter escape mode when the planner has given a valid goal point
	if (!escaping_ && best_traj->cost_ > -2.0)
	{
		escape_x_ = x;
		escape_y_ = y;
		escape_theta_ = theta;
		escaping_ = true;
	}

	dist = sqrt(
			(x - escape_x_) * (x - escape_x_)
					+ (y - escape_y_) * (y - escape_y_));

	if (dist > escape_reset_dist_
			|| fabs(angles::shortest_angular_distance(escape_theta_, theta))
					> escape_reset_theta_)
	{
		escaping_ = false;
	}

	//if the trajectory failed because the footprint hits something, we're still going to back up
	if (best_traj->cost_ == -1.0)
		best_traj->cost_ = 1.0;

	return *best_traj;

}

/*
 * given the current state of the robot, find a good trajectory while displaying all the evaluated trajectories
 */
Trajectory TrajectoryPlanner::findBestPathVector(
		tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel,
		tf::Stamped<tf::Pose>& drive_velocities,
		std::vector<Trajectory>& evaluated_trajs //Vector of all the evaluated trajectories for visualizing
		)
{

	Eigen::Vector3f pos(global_pose.getOrigin().getX(),
			global_pose.getOrigin().getY(),
			tf::getYaw(global_pose.getRotation())); //This is a "simplified" version of the pose of the robot received as param
	Eigen::Vector3f vel(global_vel.getOrigin().getX(),
			global_vel.getOrigin().getY(),
			tf::getYaw(global_vel.getRotation())); //This is a "simplified" version of the velocity of the robot received as param

	//reset the map for new operations
	path_map_.resetPathDist();
	goal_map_.resetPathDist();

	//temporarily remove obstacles that are within the footprint of the robot
	std::vector<base_local_planner::Position2DInt> footprint_list =
			footprint_helper_.getFootprintCells(pos, footprint_spec_, costmap_,
					true);

	//mark cells within the initial footprint of the robot
	for (unsigned int i = 0; i < footprint_list.size(); ++i)
	{
		path_map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
	}

	//make sure that we update our path based on the global plan and compute costs
	path_map_.setTargetCells(costmap_, global_plan_);
	goal_map_.setLocalGoal(costmap_, global_plan_);
	ROS_DEBUG("Path/Goal distance computed");

	//***** IMPORTANT ****************************************************
	//XXX rollout trajectories and find the minimum cost one
	Trajectory best = createTrajectoriesVector(pos[0], pos[1], pos[2], vel[0],
			vel[1], vel[2], acc_lim_x_, acc_lim_y_, acc_lim_theta_,
			evaluated_trajs);
	ROS_DEBUG("Trajectories created");
	//*********************************************************************

	if (best.cost_ < 0)
	{
		drive_velocities.setIdentity();
	}
	else
	{
		tf::Vector3 start(best.xv_, best.yv_, 0);
		drive_velocities.setOrigin(start);
		tf::Matrix3x3 matrix;
		matrix.setRotation(tf::createQuaternionFromYaw(best.thetav_));
		drive_velocities.setBasis(matrix);
	}
	//new_user_vel_ = false;
	return best;
}
//Computes the difference of vx vy and vtheta with the current ui_vel values
double TrajectoryPlanner::userInputDiff(double vx, double vy, double vtheta)
{
	//Using only the difference with respect to the angular velocity
	double user_vel_diff = (user_vel_.angular.z - vtheta)
			* (user_vel_.angular.z - vtheta)
			+ (user_vel_.linear.x - vx) * (user_vel_.linear.x - vx);
	//double user_yaw = (face_dir_.angular.z);

	//Using the difference with both linear and angular components
//	double user_vel_diff = sqrt((user_vel_.linear.x - vx)*(user_vel_.linear.x - vx)+
//				(user_vel_.angular.z - vtheta)*(user_vel_.angular.z - vtheta));
//	ROS_INFO("user vel difference:%f,   user_x:%f,    dynwin_x:%f,   user_theta:%f,    dynwin_theta:%f",
//			user_vel_diff, user_vel_.linear.x, vx, user_vel_.angular.z, vtheta);
	//check which function fits better to user needs.
	//Here I think I could use the predicted path to have a function cost taking into account how the user
	//will change the move of the input device over time
	// 							X <--Destination
	//            .      .      .         .
	//         .       .        .           .
	//      *---------*---------*------------*
	//      T0        T1        T2           T3

	return user_vel_diff;
}

void TrajectoryPlanner::setUserInput(double vx, double vy, double vtheta)
{
	user_vel_.linear.x = vx;
	user_vel_.linear.y = vy;
	user_vel_.linear.z = 0;
	user_vel_.angular.x = 0;
	user_vel_.angular.y = 0;
	user_vel_.angular.z = vtheta;
	new_user_vel_ = true;
}

void TrajectoryPlanner::setFaceDir(double roll, double pitch, double yaw)
{
	face_dir_.linear.x = 0;
	face_dir_.linear.y = 0;
	face_dir_.linear.z = 0;
	face_dir_.angular.x = roll;
	face_dir_.angular.y = pitch;
	face_dir_.angular.z = yaw;
	new_face_dir_ = true;
}

void TrajectoryPlanner::setUserVoice(std_msgs::String &voice)
{
	user_voice_ = voice;
}

void TrajectoryPlanner::setEscapeMode(double x, double y, double theta,
		double escape_vel)
{
	backup_vel_ = escape_vel;
	//escaping_ = true;
	escape_x_ = x;
	escape_y_ = y;
	escape_theta_ = theta;
}

}
;

