#ifndef TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_H_
#define TRAJECTORY_ROLLOUT_TRAJECTORY_PLANNER_H_

#include <vector>
#include <cmath>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <riskrrt_local_planner/footprint_helper.h>

#include <riskrrt_local_planner/world_model.h>
#include <riskrrt_local_planner/trajectory.h>
#include <base_local_planner/Position2DInt.h>
#include <riskrrt_local_planner/RISKRRTLocalPlannerConfig.h>

//we'll take in a path as a vector of poses
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
//for some datatypes
#include <tf/transform_datatypes.h>

//for creating a local cost grid
#include <riskrrt_local_planner/map_cell.h>
#include <riskrrt_local_planner/map_grid.h>

namespace riskrrt_local_planner
{
/**
 * @class TrajectoryPlanner
 * @brief Computes control velocities for a robot given a costmap, a plan, and the robot's position in the world.
 */
class TrajectoryPlanner
{
	friend class TrajectoryPlannerTest; //Need this for gtest to work
public:
	/**
	 * @brief  Constructs a trajectory controller
	 * @param world_model The WorldModel the trajectory controller uses to check for collisions
	 * @param costmap A reference to the Costmap the controller should use
	 * @param footprint_spec A polygon representing the footprint of the robot. (Must be convex)
	 * @param inscribed_radius The radius of the inscribed circle of the robot
	 * @param circumscribed_radius The radius of the circumscribed circle of the robot
	 * @param acc_lim_x The acceleration limit of the robot in the x direction
	 * @param acc_lim_y The acceleration limit of the robot in the y direction
	 * @param acc_lim_theta The acceleration limit of the robot in the theta direction
	 * @param sim_time The number of seconds to "roll-out" each trajectory
	 * @param sim_granularity The distance between simulation points should be small enough that the robot doesn't hit things
	 * @param vx_samples The number of trajectories to sample in the x dimension
	 * @param vtheta_samples The number of trajectories to sample in the theta dimension
	 * @param pdist_scale A scaling factor for how close the robot should stay to the path
	 * @param gdist_scale A scaling factor for how aggresively the robot should pursue a local goal
	 * @param occdist_scale A scaling factor for how much the robot should prefer to stay away from obstacles
	 * @param heading_lookahead How far the robot should look ahead of itself when differentiating between different rotational velocities
	 * @param oscillation_reset_dist The distance the robot must travel before it can explore rotational velocities that were unsuccessful in the past
	 * @param escape_reset_dist The distance the robot must travel before it can exit escape mode
	 * @param escape_reset_theta The distance the robot must rotate before it can exit escape mode
	 * @param holonomic_robot Set this to true if the robot being controlled can take y velocities and false otherwise
	 * @param max_vel_x The maximum x velocity the controller will explore
	 * @param min_vel_x The minimum x velocity the controller will explore
	 * @param max_vel_th The maximum rotational velocity the controller will explore
	 * @param min_vel_th The minimum rotational velocity the controller will explore
	 * @param min_in_place_vel_th The absolute value of the minimum in-place rotational velocity the controller will explore
	 * @param backup_vel The velocity to use while backing up
	 * @param dwa Set this to true to use the Dynamic Window Approach, false to use acceleration limits
	 * @param heading_scoring Set this to true to score trajectories based on the robot's heading after 1 timestep
	 * @param heading_scoring_timestep How far to look ahead in time when we score heading based trajectories
	 * @param meter_scoring adapt parameters to costmap resolution
	 * @param simple_attractor Set this to true to allow simple attraction to a goal point instead of intelligent cost propagation
	 * @param angular_sim_granularity The distance between simulation points for angular velocity should be small enough that the robot doesn't hit things
	 * @param ui_scoring A bool value to say if we want to use user intentions scoring or not
	 * @param ui_cost_scale Weighting factor of the user input when evaluating the trajectories (a bigger value will give more control to the user).
	 * @param p_space_scoring
	 * @param o_space_scoring
	 * @param GP_predict
	 * @param pThreshold
	 * @param dataDirGP
	 * @param loopRate
	 * @param pGoal
	 *
	 */
	TrajectoryPlanner(WorldModel& world_model,
			const costmap_2d::Costmap2D& costmap,
			std::vector<geometry_msgs::Point> footprint_spec, double acc_lim_x =
					1.0, double acc_lim_y = 1.0, double acc_lim_theta = 1.0,
			double sim_time = 1.0, double sim_granularity = 0.025,
			int vx_samples = 20, int vtheta_samples = 20, double pdist_scale =
					0.6, double gdist_scale = 0.8, double occdist_scale = 0.2,
			double heading_lookahead = 0.325, double oscillation_reset_dist =
					0.05, double escape_reset_dist = 0.10,
			double escape_reset_theta = M_PI_2, bool holonomic_robot = true,
			double max_vel_x = 0.5, double min_vel_x = 0.1, double max_vel_th =
					1.0, double min_vel_th = -1.0, double min_in_place_vel_th =
					0.4, double backup_vel = -0.1, bool dwa = false,
			bool heading_scoring = false, double heading_scoring_timestep = 0.1,
			bool meter_scoring = true, bool simple_attractor = false,
			std::vector<double> y_vels = std::vector<double>(0),
			double stop_time_buffer = 0.2, double sim_period = 0.1,
			double angular_sim_granularity = 0.025, bool ui_scoring = true,
			double ui_cost_scale = 3.0, bool p_space_scoring = false , bool o_space_scoring = false,
			bool GP_predict = false, double pThreshold = 0.7, std::string dataDirGP = "data/result-3.dat",
			int loopRate = 2, double pGoal = 0.01);

	/**
	 * @brief  Destructs a trajectory controller
	 */
	~TrajectoryPlanner();

	/**
	 * @brief Reconfigures the trajectory planner
	 */
	void reconfigure(RISKRRTLocalPlannerConfig &cfg);

	/**
	 * @brief  Given the current position, orientation, and velocity of the robot, return a trajectory to follow
	 * @param global_pose The current pose of the robot in world space
	 * @param global_vel The current velocity of the robot in world space
	 * @param drive_velocities Will be set to velocities to send to the robot base
	 * @return The selected path or trajectory
	 */
	Trajectory findBestPath(tf::Stamped<tf::Pose> global_pose,
			tf::Stamped<tf::Pose> global_vel,
			tf::Stamped<tf::Pose>& drive_velocities);

	/**
	 * @brief  Update the plan that the controller is following
	 * @param new_plan A new plan for the controller to follow
	 * @param compute_dists Wheter or not to compute path/goal distances when a plan is updated
	 */
	void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan,
			bool compute_dists = false);

	/**
	 * @brief  Accessor for the goal the robot is currently pursuing in world corrdinates
	 * @param x Will be set to the x position of the local goal
	 * @param y Will be set to the y position of the local goal
	 */
	void getLocalGoal(double& x, double& y);

	/**
	 * @brief  Generate and score a single trajectory
	 * @param x The x position of the robot
	 * @param y The y position of the robot
	 * @param theta The orientation of the robot
	 * @param vx The x velocity of the robot
	 * @param vy The y velocity of the robot
	 * @param vtheta The theta velocity of the robot
	 * @param vx_samp The x velocity used to seed the trajectory
	 * @param vy_samp The y velocity used to seed the trajectory
	 * @param vtheta_samp The theta velocity used to seed the trajectory
	 * @return True if the trajectory is legal, false otherwise
	 */
	bool checkTrajectory(double x, double y, double theta, double vx, double vy,
			double vtheta, double vx_samp, double vy_samp, double vtheta_samp);

	/**
	 * @brief  Generate and score a single trajectory
	 * @param x The x position of the robot
	 * @param y The y position of the robot
	 * @param theta The orientation of the robot
	 * @param vx The x velocity of the robot
	 * @param vy The y velocity of the robot
	 * @param vtheta The theta velocity of the robot
	 * @param vx_samp The x velocity used to seed the trajectory
	 * @param vy_samp The y velocity used to seed the trajectory
	 * @param vtheta_samp The theta velocity used to seed the trajectory
	 * @return The score (as double)
	 */
	double scoreTrajectory(double x, double y, double theta, double vx,
			double vy, double vtheta, double vx_samp, double vy_samp,
			double vtheta_samp);

	/**
	 * @brief Compute the components and total cost for a map grid cell
	 * @param cx The x coordinate of the cell in the map grid
	 * @param cy The y coordinate of the cell in the map grid
	 * @param path_cost Will be set to the path distance component of the cost function
	 * @param goal_cost Will be set to the goal distance component of the cost function
	 * @param occ_cost Will be set to the costmap value of the cell
	 * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
	 * @return True if the cell is traversible and therefore a legal location for the robot to move to
	 */
	bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost,
			float &occ_cost, float &total_cost);
	/*****************************************************************************************
	 * Artur work
	 *****************************************************************************************/
	/**
	 * @brief  Given the current position, orientation, and velocity of the robot, return a trajectory to follow
	 * @param global_pose The current pose of the robot in world space
	 * @param global_vel The current velocity of the robot in world space
	 * @param drive_velocities Will be set to velocities to send to the robot base
	 * @param evaluated_trajs A vector with all the trajectories that were evaluated
	 * @return The selected path or trajectory
	 */
	Trajectory findBestPathVector(tf::Stamped<tf::Pose> global_pose,
			tf::Stamped<tf::Pose> global_vel,
			tf::Stamped<tf::Pose>& drive_velocities,
			std::vector<Trajectory>& evaluated_trajs //Vector of all the evaluated trajectories for visualizing
			);

	/**
	 * @brief  Sets the ui_vel_ member with the given vx vy and vtheta values, all other values are set to 0.
	 * @param vx vy vz Are the current speeds selected by the user
	 */
	void setUiVel(double vx, double vy, double vtheta);
private:
	/**
	 * @brief  Create the trajectories we wish to explore, score them, and return the best option
	 * @param x The x position of the robot
	 * @param y The y position of the robot
	 * @param theta The orientation of the robot
	 * @param vx The x velocity of the robot
	 * @param vy The y velocity of the robot
	 * @param vtheta The theta velocity of the robot
	 * @param acc_x The x acceleration limit of the robot
	 * @param acc_y The y acceleration limit of the robot
	 * @param acc_theta The theta acceleration limit of the robot
	 * @param evaluated_trajs A vector with all the trajectories that were evaluated
	 * @return
	 */
	Trajectory createTrajectoriesVector(double x, double y, double theta,
			double vx, double vy, double vtheta, double acc_x, double acc_y,
			double acc_theta, std::vector<Trajectory>& evaluated_trajs //Vector of all the evaluated trajectories for visualizing
			);

	/**
	 * @brief  Create the trajectories we wish to explore, score them, and return the best option
	 * @param x The x position of the robot
	 * @param y The y position of the robot
	 * @param theta The orientation of the robot
	 * @param vx The x velocity of the robot
	 * @param vy The y velocity of the robot
	 * @param vtheta The theta velocity of the robot
	 * @param acc_x The x acceleration limit of the robot
	 * @param acc_y The y acceleration limit of the robot
	 * @param acc_theta The theta acceleration limit of the robot
	 * @return
	 */
	Trajectory createTrajectories(double x, double y, double theta, double vx,
			double vy, double vtheta, double acc_x, double acc_y,
			double acc_theta);

	/**
	 * @brief  Generate and score a single trajectory
	 * @param x The x position of the robot
	 * @param y The y position of the robot
	 * @param theta The orientation of the robot
	 * @param vx The x velocity of the robot
	 * @param vy The y velocity of the robot
	 * @param vtheta The theta velocity of the robot
	 * @param vx_samp The x velocity used to seed the trajectory
	 * @param vy_samp The y velocity used to seed the trajectory
	 * @param vtheta_samp The theta velocity used to seed the trajectory
	 * @param acc_x The x acceleration limit of the robot
	 * @param acc_y The y acceleration limit of the robot
	 * @param acc_theta The theta acceleration limit of the robot
	 * @param impossible_cost The cost value of a cell in the local map grid that is considered impassable
	 * @param traj Will be set to the generated trajectory with its associated score
	 */
	void generateTrajectory(double x, double y, double theta, double vx,
			double vy, double vtheta, double vx_samp, double vy_samp,
			double vtheta_samp, double acc_x, double acc_y, double acc_theta,
			double impossible_cost, Trajectory& traj);

	/**
	 * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
	 * @param x_i The x position of the robot
	 * @param y_i The y position of the robot
	 * @param theta_i The orientation of the robot
	 * @return
	 */
	double footprintCost(double x_i, double y_i, double theta_i);

	riskrrt_local_planner::FootprintHelper footprint_helper_;

	MapGrid path_map_; ///< @brief The local map grid where we propagate path distance
	MapGrid goal_map_; ///< @brief The local map grid where we propagate goal distance
	const costmap_2d::Costmap2D& costmap_; ///< @brief Provides access to cost map information
	riskrrt_local_planner::WorldModel& world_model_; ///< @brief The world model that the controller uses for collision detection

	std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot

	std::vector<geometry_msgs::PoseStamped> global_plan_; ///< @brief The global path for the robot to follow

	bool stuck_left, stuck_right; ///< @brief Booleans to keep the robot from oscillating during rotation
	bool rotating_left, rotating_right; ///< @brief Booleans to keep track of the direction of rotation for the robot

	bool stuck_left_strafe, stuck_right_strafe; ///< @brief Booleans to keep the robot from oscillating during strafing
	bool strafe_right, strafe_left; ///< @brief Booleans to keep track of strafe direction for the robot

	bool escaping_; ///< @brief Boolean to keep track of whether we're in escape mode
	bool meter_scoring_;

	double goal_x_, goal_y_; ///< @brief Storage for the local goal the robot is pursuing

	double final_goal_x_, final_goal_y_; ///< @brief The end position of the plan.
	bool final_goal_position_valid_; ///< @brief True if final_goal_x_ and final_goal_y_ have valid data.  Only false if an empty path is sent.

	double sim_time_; ///< @brief The number of seconds each trajectory is "rolled-out"
	double sim_granularity_; ///< @brief The distance between simulation points
	double angular_sim_granularity_; ///< @brief The distance between angular simulation points

	int vx_samples_; ///< @brief The number of samples we'll take in the x dimenstion of the control space
	int vtheta_samples_; ///< @brief The number of samples we'll take in the theta dimension of the control space

	double pdist_scale_, gdist_scale_, occdist_scale_; ///< @brief Scaling factors for the controller's cost function
	double acc_lim_x_, acc_lim_y_, acc_lim_theta_; ///< @brief The acceleration limits of the robot

	double prev_x_, prev_y_; ///< @brief Used to calculate the distance the robot has traveled before reseting oscillation booleans
	double escape_x_, escape_y_, escape_theta_; ///< @brief Used to calculate the distance the robot has traveled before reseting escape booleans

	Trajectory traj_one, traj_two; ///< @brief Used for scoring trajectories

	double heading_lookahead_; ///< @brief How far the robot should look ahead of itself when differentiating between different rotational velocities
	double oscillation_reset_dist_; ///< @brief The distance the robot must travel before it can explore rotational velocities that were unsuccessful in the past
	double escape_reset_dist_, escape_reset_theta_; ///< @brief The distance the robot must travel before it can leave escape mode
	bool holonomic_robot_; ///< @brief Is the robot holonomic or not?

	double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_,
			min_in_place_vel_th_; ///< @brief Velocity limits for the controller

	double backup_vel_; ///< @brief The velocity to use while backing up

	bool dwa_;  ///< @brief Should we use the dynamic window approach?
	bool heading_scoring_; ///< @brief Should we score based on the rollout approach or the heading approach
	bool ui_scoring_; ///< @brief Should we use user_intentions scoring
	double heading_scoring_timestep_; ///< @brief How far to look ahead in time when we score a heading
	bool simple_attractor_; ///< @brief Enables simple attraction to a goal point

	std::vector<double> y_vels_; ///< @brief Y velocities to explore

	double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
	double sim_period_; ///< @brief The number of seconds to use to compute max/min vels for dwa

	boost::mutex configuration_mutex_;
	double ui_diff_cost_; ///< @brief The diference between the velocity asked by the user and the one beeing tested from the dynamic window
	double ui_cost_scale_; ///< @brief A weighting factor for the user cost
	geometry_msgs::Twist ui_vel_; /// @brief The user intended/selected speed
	double pThreshold_; /// @brief Threshold for the probability of collision: configurations with bigger P are discarded
	bool p_space_scoring_; /// @brief If we want to respect personal space of humans around the robot
	bool o_space_scoring_; /// @brief to take into account o-space (interaction space).
	bool GP_predict_; /// @brief If it is True we use Gaussian Processes as prediction, if not, we just use linear prediction.
	std::string dataDirGP_; /// @brief Database for typical trajectories used for GP computing
	int loopRate_; /// @brief rate in hz of the loop of the planning (int) 1 Hz doesn't work
	double pGoal_; /// @brief Probability of bias to the goal during the Growing.
	struct dataParam riskrrt_params_; /// @brief An structure containning some parameters to initialize riskrrtplanner
	Wheelchair riskrrt_robot_; /// @brief The robot_object used by riskrrt

	LocalOccupancyGrid  riskrrt_occ_grid_; ///@brief The local occupancy grid used by riskrrt
	std::vector <PedestrianPredictor*> ped_predict_vect_; ///@brief Pedestrian predictor vector
	Loader* riskrrt_gp_load_; ///@brief This is to load the file with learned GP
	Pos riskrrt_goal_;
	State riskrrt_robot_origin_;
	Trajectory riskrrt_path_;
	std::vector <double> ped_predict_time_vect_;///@brief time vector for prediction

	/**
	 * @brief  Compute x position based on velocity
	 * @param  xi The current x position
	 * @param  vx The current x velocity
	 * @param  vy The current y velocity
	 * @param  theta The current orientation
	 * @param  dt The timestep to take
	 * @return The new x position
	 */
	inline double computeNewXPosition(double xi, double vx, double vy,
			double theta, double dt)
	{
		return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
	}

	/**
	 * @brief  Compute y position based on velocity
	 * @param  yi The current y position
	 * @param  vx The current x velocity
	 * @param  vy The current y velocity
	 * @param  theta The current orientation
	 * @param  dt The timestep to take
	 * @return The new y position
	 */
	inline double computeNewYPosition(double yi, double vx, double vy,
			double theta, double dt)
	{
		return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
	}

	/**
	 * @brief  Compute orientation based on velocity
	 * @param  thetai The current orientation
	 * @param  vth The current theta velocity
	 * @param  dt The timestep to take
	 * @return The new orientation
	 */
	inline double computeNewThetaPosition(double thetai, double vth, double dt)
	{
		return thetai + vth * dt;
	}

	//compute velocity based on acceleration
	/**
	 * @brief  Compute velocity based on acceleration
	 * @param vg The desired velocity, what we're accelerating up to
	 * @param vi The current velocity
	 * @param a_max An acceleration limit
	 * @param  dt The timestep to take
	 * @return The new velocity
	 */
	inline double computeNewVelocity(double vg, double vi, double a_max,
			double dt)
	{
		if ((vg - vi) >= 0)
		{
			return std::min(vg, vi + a_max * dt);
		}
		return std::max(vg, vi - a_max * dt);
	}

	void getMaxSpeedToStopInTime(double time, double& vx, double& vy,
			double& vth)
	{
		vx = acc_lim_x_ * std::max(time, 0.0);
		vy = acc_lim_y_ * std::max(time, 0.0);
		vth = acc_lim_theta_ * std::max(time, 0.0);
	}

	double lineCost(int x0, int x1, int y0, int y1);
	double pointCost(int x, int y);

	double headingDiff(int cell_x, int cell_y, double x, double y,
			double heading);
	double userVelDiff(double vx, double vy, double vtheta);

};
}
;

#endif
